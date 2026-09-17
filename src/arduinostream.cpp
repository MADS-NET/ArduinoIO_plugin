/*
  ____  _
 / ___|| |_ _ __ ___  __ _ _ __ ___
 \___ \| __| '__/ _ \/ _` | '_ ` _ \
  ___) | |_| | |  __/ (_| | | | | | |
 |____/ \__|_|  \___|\__,_|_| |_| |_|

# UsbstreamPlugin: a Source Plugin for bulk USB streaming
# Hand-written companion to arduinousb.cpp / usb_driver.hpp.
# NOTICE: MADS Version 2.4.3
*/
// Mandatory included headers
#include <source.hpp>
#include <nlohmann/json.hpp>
#include <pugg/Kernel.h>
#include <map>
#include <vector>
#include <stdio.h>

// other includes as needed here
#include <arduino_driver/Device.h>
#include <arduino_driver/Enumerator.h>
#include <arduino_driver/Errors.h>
#include <arduino_driver/Protocol.h>
#include "usb_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <future>
#include <iomanip>
#include <optional>
#include <sstream>
#include <span>
#include <thread>

// Load the namespaces
using namespace std;
using namespace ArduinoDriver;
using json = nlohmann::json;

// TimeUnwrapper: turns Sample::t_us (device micros(), a uint32_t that wraps
// every ~71.6 minutes) into a monotonically increasing uint64_t, anchored to
// a DeviceTime::micros64 reading taken once in set_params(), before the
// stream starts.
//
// Plain, dependency-free struct on purpose: the test main() at the bottom of
// this file exercises it directly, without opening a device.
struct TimeUnwrapper {
  // Seeds the unwrapper from the anchor's 64-bit device microsecond clock.
  // If the very first sample handed to unwrap() reads lower than the
  // anchor's low 32 bits, the stream must have wrapped between the anchor
  // and that first sample, so the high word is bumped once, right away.
  void seed(std::uint64_t anchor_us) {
    high = static_cast<std::uint32_t>(anchor_us >> 32);
    anchor_low = static_cast<std::uint32_t>(anchor_us & 0xFFFFFFFFull);
    first = true;
  }

  // Unwraps one device timestamp. Must be called with values that, modulo
  // 2^32, increase monotonically (true of a live stream's t_us sequence).
  std::uint64_t unwrap(std::uint32_t t) {
    if (first) {
      if (t < anchor_low) {
        ++high;
      }
      first = false;
    } else if (t < last) {
      ++high;
    }
    last = t;
    return (static_cast<std::uint64_t>(high) << 32) | t;
  }

  std::uint32_t high{0};
  std::uint32_t anchor_low{0};
  std::uint32_t last{0};
  bool first{true};
};

// SessionTimeline: places the records of every stream session (the first
// one, and each one a restart opens) on the single t_us timeline the plugin
// publishes, which never goes backwards.
//
// Within a session, t_us is the unwrapped device clock plus a constant
// offset (0 for the first session). A session that finds the board still
// running keeps the offset, so t_us stays the board's own clock across the
// restart. A board that was reset (unplugged, power-cycled) restarted its
// clock near zero: the offset is then recomputed so that t_us continues
// where the host clock says it should, and the jump in t_us across the gap
// matches the time that actually went by.
//
// Plain integers only, so the test main() can exercise it without a device.
struct SessionTimeline {
  // Starts a session from its anchor: the device clock (micros64) as read at
  // host steady-clock time host_us, before the session's STREAM_START.
  void start_session(std::uint64_t device_us, std::int64_t host_us) {
    const auto dev = static_cast<std::int64_t>(device_us);
    reset = false;
    if (started) {
      const std::int64_t host_elapsed = host_us - prev_host_us;
      const std::int64_t dev_elapsed =
          dev - static_cast<std::int64_t>(prev_device_us);
      // A board that kept running advanced its clock with the host's, within
      // crystal tolerance (200 ppm) and anchor uncertainty (50 ms).
      reset = dev_elapsed < host_elapsed - host_elapsed / 5000 - 50000;
      if (reset) {
        offset = static_cast<std::int64_t>(prev_device_us) + offset +
                 host_elapsed - dev;
      }
      // Every record of the new session is no older than its anchor, so this
      // keeps t_us strictly increasing whatever the two clocks did.
      if (have_last && dev + offset <= static_cast<std::int64_t>(last_t_us)) {
        offset = static_cast<std::int64_t>(last_t_us) + 1 - dev;
      }
    }
    started = true;
    prev_device_us = device_us;
    prev_host_us = host_us;
  }

  // Maps an unwrapped device timestamp of the current session onto t_us.
  std::uint64_t map(std::uint64_t device_us) {
    last_t_us = static_cast<std::uint64_t>(
        static_cast<std::int64_t>(device_us) + offset);
    have_last = true;
    return last_t_us;
  }

  std::int64_t offset{0};
  std::uint64_t last_t_us{0}; // last value map() returned
  bool have_last{false};
  bool reset{false}; // the last start_session() found the board reset
  std::uint64_t prev_device_us{0};
  std::int64_t prev_host_us{0};
  bool started{false};
};

// Stream counters summed over every session: each Stream starts its own
// StreamStats from zero.
struct StreamTotals {
  StreamTotals &operator+=(const StreamStats &s) {
    seq_gaps += s.seq_gaps;
    host_drops += s.host_drops;
    device_overruns += s.device_overruns;
    resyncs += s.resyncs;
    stale_records += s.stale_records;
    records_received += s.records_received;
    return *this;
  }

  std::uint64_t seq_gaps{0};
  std::uint64_t host_drops{0};
  std::uint64_t device_overruns{0};
  std::uint64_t resyncs{0};
  std::uint64_t stale_records{0};
  std::uint64_t records_received{0};
};

// Formats a system_clock time point as an ISO-8601 UTC string with
// millisecond precision, e.g. "2026-09-16T10:11:12.345Z". Uses gmtime_s on
// Windows and gmtime_r elsewhere, per the project's portability rule.
static string format_iso8601_utc(std::chrono::system_clock::time_point tp) {
  using namespace std::chrono;
  auto since_epoch = tp.time_since_epoch();
  auto us = duration_cast<microseconds>(since_epoch);
  std::time_t secs = duration_cast<seconds>(us).count();
  long long ms = duration_cast<milliseconds>(us).count() % 1000;
  if (ms < 0) ms += 1000;

  std::tm tm_utc{};
#if defined(_WIN32)
  gmtime_s(&tm_utc, &secs);
#else
  gmtime_r(&secs, &tm_utc);
#endif

  char date_buf[32];
  std::strftime(date_buf, sizeof(date_buf), "%Y-%m-%dT%H:%M:%S", &tm_utc);
  char out_buf[40];
  std::snprintf(out_buf, sizeof(out_buf), "%s.%03lldZ", date_buf, ms);
  return string(out_buf);
}

// One streamed channel's output shape, indexed by its position `j` within a
// stream record (i.e. _stream->pins()[j], the same order Stream::read()
// copies samples in). Precomputed once in set_params() so get_output() can
// index straight into it instead of doing a _pin_modes lookup and a map
// insertion per sample.
struct Channel {
  string key;  // to_string(pin), precomputed once
  bool analog; // true: goes under "analog" (as volts or raw); false: "digital"
};

// Plugin class: streams pins in ANALOG_IN / INPUT* mode over the bulk IN
// endpoint (ArduinoDriver::Stream) instead of polling them one control
// transfer at a time (that is what arduinousb.cpp's UsbsourcePlugin does).
// See usb_driver.hpp for the USBDriver mixin this class shares with it.
//
// When the stream fails (a USB transfer error, the board unplugged), the
// plugin publishes what the dead stream had already decoded, then reopens
// the board and starts a new session on a background thread, retrying with
// backoff until the board answers again (setting `restart`). The thread owns
// _dev and _stream while it runs, so get_output() never blocks on USB.
class UsbstreamPlugin : public Source<json>, public USBDriver {

public:
  using Source::Source; // inherit constructors

  // A pending restart owns _dev and _stream on its own thread: wait for it
  // first. Then stop the Stream before the USBDriver base destroys the Device
  // it belongs to (member destruction order already guarantees this, but
  // reset explicitly to make the requirement hold even if that ever changes).
  ~UsbstreamPlugin() override {
    if (_restart.valid()) {
      _restart.wait();
    }
    _stream.reset();
  }

  string kind() override { return PLUGIN_NAME; }

  void set_params(const json &params) override {
    Source::set_params(params);

    _params["serial"] = "";
    _params["pin_modes"] = json::object();
    _params["pin_modes"]["15"] = "ANALOG";
    _params["sample_rate"] = 1000.0;
    _params["buffer_size"] = 0;
    _params["chunk_size"] = 0;
    _params["volts"] = true;
    _params["restart"] = true;
    _params["max_restarts"] = 0;

    _params.merge_patch(params);
    // merge_patch merges JSON objects key by key; a user-supplied pin_modes
    // table must REPLACE the default {"15": "ANALOG"}, not merge with it.
    if (params.contains("pin_modes")) {
      _params["pin_modes"] = params["pin_modes"];
    }

    try {
      read_pin_modes(_params["pin_modes"], _allowed_pin_modes);
      if (_pin_modes.empty()) {
        throw runtime_error("pin_modes must not be empty: at least one "
                             "streamed pin is required");
      }

      double sample_rate = _params["sample_rate"].get<double>();
      if (sample_rate < 0.0) {
        throw runtime_error("sample_rate must be >= 0 (got " +
                             to_string(sample_rate) + ")");
      }
      std::chrono::microseconds period{0};
      int period_us = 0;
      if (sample_rate > 0.0) {
        long long p = llround(1.0e6 / sample_rate);
        if (p < static_cast<long long>(StreamMinPeriodUs) || p > 65535) {
          double max_hz = 1.0e6 / static_cast<double>(StreamMinPeriodUs);
          double min_hz = 1.0e6 / 65535.0;
          ostringstream oss;
          oss << "sample_rate out of range: valid interval is "
              << fixed << setprecision(3) << min_hz << " .. " << max_hz
              << " Hz (0 = free-running), got " << sample_rate << " Hz";
          throw runtime_error(oss.str());
        }
        period_us = static_cast<int>(p);
        period = std::chrono::microseconds(period_us);
      }

      long long buffer_size = _params["buffer_size"].get<long long>();
      if (buffer_size < 0) {
        throw runtime_error("buffer_size must be >= 0 (got " +
                             to_string(buffer_size) + ")");
      }
      if (buffer_size == 0) {
        double rate_for_auto = sample_rate > 0.0 ? sample_rate : 10000.0;
        buffer_size =
            std::max<long long>(1024, static_cast<long long>(std::ceil(rate_for_auto)));
      }

      long long chunk_size = _params["chunk_size"].get<long long>();
      if (chunk_size < 0) {
        throw runtime_error("chunk_size must be >= 0 (got " +
                             to_string(chunk_size) + ")");
      }
      if (chunk_size > buffer_size) {
        throw runtime_error("chunk_size (" + to_string(chunk_size) +
                             ") must be <= buffer_size (" +
                             to_string(buffer_size) + ")");
      }

      long long max_restarts = _params["max_restarts"].get<long long>();
      if (max_restarts < 0) {
        throw runtime_error("max_restarts must be >= 0 (got " +
                             to_string(max_restarts) + ")");
      }

      _cfg = StreamConfig{};
      for (const auto &[pin, mode] : _pin_modes) {
        _cfg.pins.push_back(pin);
      }
      _cfg.period = period;
      _cfg.flags = 0;
      _cfg.queue_capacity = static_cast<size_t>(buffer_size);

      _serial = find_serial(_params["serial"].get<string>());
      start_timeline(start_session());

      // Precompute the per-channel layout once, from the Stream's own pin
      // order (== _cfg.pins, but this is the authoritative source): the hot
      // path in get_output() then indexes straight into _channels[j] instead
      // of looking `s.pin` up in _pin_modes for every sample.
      const auto &stream_pins = _stream->pins();
      _channels.clear();
      _channels.reserve(stream_pins.size());
      for (uint8_t pin : stream_pins) {
        _channels.push_back(
            Channel{to_string(pin), _pin_modes.at(pin) == PinMode::AnalogIn});
      }

      _n_pins = _channels.size();
      _staging.assign(static_cast<size_t>(buffer_size) * _n_pins, Sample{});
      _staging_t.assign(static_cast<size_t>(buffer_size), 0);
      _staged = 0;

      _sample_rate = sample_rate;
      _period_us = period_us;
      _buffer_size = static_cast<size_t>(buffer_size);
      _chunk_size = static_cast<size_t>(chunk_size);
      _volts = _params["volts"].get<bool>();
      _restart_enabled = _params["restart"].get<bool>();
      _max_restarts = static_cast<uint64_t>(max_restarts);
    } catch (const exception &e) {
      _error = e.what();
      cerr << e.what() << endl;
      _init_error = true;
    }
  }

  // Never blocks: drains whatever the Stream worker already decoded with
  // timeout 0, then either publishes a chunk or returns retry. A restart
  // runs on its own thread; while it does, this only publishes records that
  // are already staged.
  return_type get_output(json &out, vector<unsigned char> * /*blob*/ = nullptr) override {
    if (_init_error) return return_type::critical;
    out.clear();
    next_loop_duration = std::chrono::milliseconds(0);

    try {
      // 1. Collect a finished restart attempt, or start the next one once
      // the backoff after a failed attempt has expired.
      if (_restart.valid() &&
          _restart.wait_for(std::chrono::seconds(0)) == std::future_status::ready &&
          !finish_restart()) {
        return return_type::error;
      }
      if (!_restart.valid() && !_stream &&
          std::chrono::steady_clock::now() >= _next_attempt) {
        launch_restart();
      }

      // 2. Drain the Stream's queue without blocking. A stopped Stream still
      // hands out what it decoded before stopping, so only once that is all
      // staged does the plugin restart it (or give up on it).
      if (!_restart.valid() && _stream && _fatal.empty()) {
        const bool alive = _stream->running();
        if (drain() && !alive) {
          const string why = _stream->error();
          _failure = why.empty() ? string("unknown reason") : why;
          if (_restart_enabled &&
              (_max_restarts == 0 || _restarts < _max_restarts)) {
            _base += _stream->stats();
            cerr << "arduinostream: stream stopped (" << _failure
                 << "), restarting" << endl;
            launch_restart();
          } else {
            _fatal = "stream stopped: " + _failure;
            if (_restart_enabled) {
              _fatal += " (max_restarts = " + to_string(_max_restarts) +
                        " reached)";
            }
          }
        }
      }

      size_t records_staged = _staged / _n_pins;
      if (!_fatal.empty() && records_staged == 0) {
        _error = _fatal;
        return return_type::critical;
      }
      // A stream that will not come back publishes its last records even
      // when they are fewer than chunk_size.
      if (records_staged == 0 ||
          (_chunk_size > 0 && records_staged < _chunk_size && _fatal.empty())) {
        return return_type::retry;
      }

      size_t K = (_chunk_size > 0 && records_staged >= _chunk_size)
                     ? _chunk_size
                     : records_staged;

      // 3. Build columns from the first K records. One native vector per
      // channel, pre-reserved and filled by index (no _pin_modes lookup and
      // no map insertion per sample: at 10 kHz x 8 channels that used to be
      // ~160k map ops/tick). Moved into `out` once each at the end, since
      // frames may hold up to 10k records and growing a json array one
      // push_back() at a time is far slower than building a vector first.
      vector<uint64_t> t_us_vec(_staging_t.begin(),
                                _staging_t.begin() + static_cast<std::ptrdiff_t>(K));
      vector<vector<double>> analog_volts_cols(_n_pins);
      vector<vector<uint16_t>> analog_raw_cols(_n_pins);
      vector<vector<int>> digital_cols(_n_pins);
      for (size_t j = 0; j < _n_pins; ++j) {
        if (_channels[j].analog) {
          if (_volts) analog_volts_cols[j].reserve(K);
          else analog_raw_cols[j].reserve(K);
        } else {
          digital_cols[j].reserve(K);
        }
      }

      for (size_t i = 0; i < K; ++i) {
        const Sample *record = &_staging[i * _n_pins];
        for (size_t j = 0; j < _n_pins; ++j) {
          const Sample &s = record[j];
          if (_channels[j].analog) {
            if (_volts) analog_volts_cols[j].push_back(s.volts);
            else analog_raw_cols[j].push_back(s.raw);
          } else {
            digital_cols[j].push_back(s.raw != 0 ? 1 : 0);
          }
        }
      }

      uint64_t t_first = t_us_vec.front();
      uint64_t t_last = t_us_vec.back();

      out["t_us"] = std::move(t_us_vec);
      for (size_t j = 0; j < _n_pins; ++j) {
        const string &key = _channels[j].key;
        if (_channels[j].analog) {
          if (_volts) out["analog"][key] = std::move(analog_volts_cols[j]);
          else out["analog"][key] = std::move(analog_raw_cols[j]);
        } else {
          out["digital"][key] = std::move(digital_cols[j]);
        }
      }

      // 4. QoS: rate actually achieved, loss and restart counters (delta
      // since the previous frame, plus running totals) and publish latency.
      double rate_hz = 0.0;
      if (K >= 2 && t_last > t_first) {
        rate_hz = static_cast<double>(K - 1) * 1.0e6 /
                  static_cast<double>(t_last - t_first);
      }

      // While a restart runs, its thread owns _stream: the dead session's
      // counters are already in _base.
      StreamTotals totals = _base;
      if (_stream && !_restart.valid()) {
        totals += _stream->stats();
      }
      // device_overruns are also counted in seq_gaps (see StreamStats), so
      // "lost records" below sums seq_gaps + host_drops only, not overruns.
      uint64_t d_overruns = totals.device_overruns - _last_totals.device_overruns;
      uint64_t d_seq_gaps = totals.seq_gaps - _last_totals.seq_gaps;
      uint64_t d_host_drops = totals.host_drops - _last_totals.host_drops;
      uint64_t d_resyncs = totals.resyncs - _last_totals.resyncs;
      uint64_t d_restarts = _restarts - _last_restarts;
      uint64_t d_gap_us = _gap_us - _last_gap_us;
      uint64_t d_gap_records = _gap_records - _last_gap_records;

      int64_t delta_us =
          static_cast<int64_t>(t_last) - static_cast<int64_t>(_anchor_t_us);
      auto host_time_last = _anchor.host_time + std::chrono::microseconds(delta_us);
      double latency_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - host_time_last)
                              .count();

      json qos;
      qos["records"] = K;
      qos["rate_hz"] = rate_hz;
      qos["seq_gaps"] = d_seq_gaps;
      qos["host_drops"] = d_host_drops;
      qos["device_overruns"] = d_overruns;
      qos["resyncs"] = d_resyncs;
      qos["restarts"] = d_restarts;
      qos["gap_us"] = d_gap_us;
      qos["totals"] = {{"seq_gaps", totals.seq_gaps},
                        {"host_drops", totals.host_drops},
                        {"device_overruns", totals.device_overruns},
                        {"resyncs", totals.resyncs},
                        {"stale_records", totals.stale_records},
                        {"records_received", totals.records_received},
                        {"restarts", _restarts},
                        {"gap_us", _gap_us}};
      qos["latency_ms"] = latency_ms;
      out["qos"] = std::move(qos);
      out["time_ref"] = _time_ref;

      _last_totals = totals;
      _last_restarts = _restarts;
      _last_gap_us = _gap_us;
      _last_gap_records = _gap_records;

      // 5. Shift the leftover (unconsumed) records to the front.
      size_t consumed = K * _n_pins;
      std::copy(_staging.begin() + static_cast<std::ptrdiff_t>(consumed),
                _staging.begin() + static_cast<std::ptrdiff_t>(_staged),
                _staging.begin());
      std::copy(_staging_t.begin() + static_cast<std::ptrdiff_t>(K),
                _staging_t.begin() + static_cast<std::ptrdiff_t>(records_staged),
                _staging_t.begin());
      _staged -= consumed;

      size_t remaining_records = _staged / _n_pins;
      if ((_chunk_size > 0 && remaining_records >= _chunk_size) ||
          (!_fatal.empty() && remaining_records > 0)) {
        next_loop_duration = std::chrono::milliseconds(1);
      }

      if (!_agent_id.empty()) out["agent_id"] = _agent_id;

      vector<string> notes;
      if (d_restarts > 0) {
        ostringstream oss;
        if (d_restarts > 1) {
          oss << "stream restarted " << d_restarts << " times, last after: ";
        } else {
          oss << "stream restarted after: ";
        }
        oss << _failure << " (gap " << fixed << setprecision(1)
            << static_cast<double>(d_gap_us) / 1000.0 << " ms";
        if (_period_us > 0) {
          oss << ", ~" << d_gap_records << " records missed";
        }
        oss << ")";
        notes.push_back(oss.str());
      }
      uint64_t lost = d_seq_gaps + d_host_drops;
      if (lost > 0) {
        ostringstream oss;
        oss << "lost " << lost << " records (seq gaps " << d_seq_gaps
            << ", host drops " << d_host_drops << ", device overruns "
            << d_overruns << ")";
        notes.push_back(oss.str());
      }
      if (!notes.empty()) {
        _error = notes.front();
        for (size_t i = 1; i < notes.size(); ++i) {
          _error += "; " + notes[i];
        }
        return return_type::warning;
      }
      return return_type::success;
    } catch (const exception &e) {
      _error = e.what();
      return return_type::error;
    }
  }

  // Must not throw, and must not touch _dev when _init_error is set: MADS
  // calls info() once, right after set_params(), even when set_params()
  // failed.
  map<string, string> info() override {
    if (_init_error) {
      return {{"error", _error}};
    }

    map<string, string> result;
    result["board"] = string(board_name(_dev->info().board_id));
    result["serial"] = _serial.empty() ? string("(none: first device)") : _serial;
    result["pin modes"] = _params["pin_modes"].dump();
    result["sample_rate"] =
        (_sample_rate > 0.0 ? to_string(_sample_rate) + " Hz" : "0 (free-running)");
    result["period_us"] =
        (_period_us > 0 ? to_string(_period_us) : "0 (free-running)");
    result["buffer_size"] = to_string(_buffer_size);
    result["chunk_size"] = to_string(_chunk_size);
    result["volts"] = _volts ? "true" : "false";
    result["restart"] =
        !_restart_enabled ? string("off")
        : _max_restarts == 0
            ? string("on, unlimited")
            : "on, at most " + to_string(_max_restarts);

    double rate_for_note = _sample_rate > 0.0 ? _sample_rate : 10000.0;
    double buffer_ms = static_cast<double>(_buffer_size) * 1000.0 / rate_for_note;
    if (buffer_ms < 200.0) {
      ostringstream oss;
      oss << "buffer_size covers only " << fixed << setprecision(1) << buffer_ms
          << " ms at " << rate_for_note << " Hz; consider raising buffer_size";
      result["note"] = oss.str();
    }

    return result;
  };

private:
  // The serial number of the board to stream from, so that a restart reopens
  // that same board even when `serial` is empty ("first device"). Returns ""
  // when no board is identified, or the first one has no readable serial:
  // open() then falls back to the first device, and reports why none opened.
  string find_serial(const string &configured) {
    if (!configured.empty()) {
      return configured;
    }
    for (const auto &device :
         ArduinoDriver::list_devices(_ctx, ArduinoDriver::EnumerateOptions{})) {
      if (device.identified) {
        return device.serial;
      }
    }
    return "";
  }

  // Opens the board and starts a stream session: (re)creates _dev and
  // _stream from _serial, _pin_modes and _cfg, and returns the clock anchor
  // read just before STREAM_START. Runs in set_params() for the first
  // session and on the restart thread for every later one.
  DeviceTime start_session() {
    _stream.reset(); // a dead stream: joins its worker, STREAM_STOP if possible
    _dev.reset();
    open(_serial);
    if (!_dev->info().streaming()) {
      throw runtime_error(
          "Board " + string(board_name(_dev->info().board_id)) +
          " does not support bulk streaming (USBIO_FLAG_STREAMING not set)");
    }
    apply_pin_modes(); // a replugged board has lost them
    // Anchor device time to host time BEFORE starting the stream: once the
    // Stream runs, every other Device call (including read_time()) throws
    // DeviceBusy.
    DeviceTime anchor = _dev->read_time();
    _stream.emplace(_dev->start_stream(_cfg));
    return anchor;
  }

  // Puts a new session on the t_us timeline and publishes its anchor as
  // time_ref. Main thread only.
  void start_timeline(const DeviceTime &anchor) {
    using namespace std::chrono;
    _timeline.start_session(
        anchor.micros64,
        duration_cast<microseconds>(anchor.host_time.time_since_epoch()).count());
    _unwrapper.seed(anchor.micros64);
    _anchor = anchor;
    _anchor_t_us = static_cast<uint64_t>(
        static_cast<int64_t>(anchor.micros64) + _timeline.offset);

    // Take the steady/system clock pair together, so the anchor's steady
    // host_time can be mapped to a system_clock time for the ISO-8601 string.
    auto steady_now = steady_clock::now();
    auto system_now = system_clock::now();
    auto anchor_system =
        system_now + duration_cast<microseconds>(anchor.host_time - steady_now);
    _time_ref = json{{"t_us", _anchor_t_us},
                     {"device_us", anchor.micros64},
                     {"host", format_iso8601_utc(anchor_system)},
                     {"uncertainty_us", anchor.round_trip.count() / 2}};
  }

  // Moves whatever the Stream has decoded into the staging buffers, without
  // blocking, and gives each record its t_us. Returns true when the Stream's
  // queue is empty, false when staging filled up first.
  bool drain() {
    while (_staged < _staging.size()) {
      std::span<Sample> free_tail(_staging.data() + _staged,
                                  _staging.size() - _staged);
      const size_t n = _stream->read(free_tail, std::chrono::milliseconds(0));
      if (n == 0) {
        return true;
      }
      // read() copies whole records only.
      for (size_t i = _staged; i < _staged + n; i += _n_pins) {
        const bool had_last = _timeline.have_last;
        const uint64_t prev_t = _timeline.last_t_us;
        const uint64_t t = _timeline.map(_unwrapper.unwrap(_staging[i].t_us));
        if (_gap_pending) {
          // First record after a restart: measure the hole it closes.
          _gap_pending = false;
          if (had_last) {
            const uint64_t gap = t - prev_t;
            _gap_us += gap;
            if (_period_us > 0) {
              const uint64_t period = static_cast<uint64_t>(_period_us);
              const uint64_t steps = (gap + period / 2) / period;
              _gap_records += steps > 0 ? steps - 1 : 0;
            }
          }
        }
        _staging_t[i / _n_pins] = t;
      }
      _staged += n;
    }
    return false;
  }

  void launch_restart() {
    _restart = std::async(std::launch::async, [this] { return start_session(); });
  }

  // Collects a finished restart attempt. On success the new session goes on
  // the timeline; on failure the next attempt is scheduled with backoff
  // (0.5, 1, 2, 4, then every 5 s), _error says why, and false is returned.
  bool finish_restart() {
    DeviceTime anchor;
    try {
      anchor = _restart.get();
    } catch (const exception &e) {
      ++_failed_attempts;
      const unsigned doublings = std::min(_failed_attempts - 1, 4u);
      const auto delay = std::min(std::chrono::milliseconds(500 << doublings),
                                  std::chrono::milliseconds(5000));
      _next_attempt = std::chrono::steady_clock::now() + delay;
      ostringstream oss;
      oss << "stream restart attempt " << _failed_attempts << " failed: "
          << e.what() << "; next attempt in " << fixed << setprecision(1)
          << static_cast<double>(delay.count()) / 1000.0
          << " s (stream stopped: " << _failure << ")";
      _error = oss.str();
      cerr << "arduinostream: " << _error << endl;
      return false;
    }
    _failed_attempts = 0;
    ++_restarts;
    start_timeline(anchor);
    _gap_pending = true;
    cerr << "arduinostream: stream restarted"
         << (_timeline.reset ? " (the board was reset: t_us bridged with host time)"
                             : "")
         << endl;
    return true;
  }

  const vector<string> _allowed_pin_modes{"ANALOG", "INPUT", "PULLUP", "PULLDOWN"};

  optional<Stream> _stream;
  StreamConfig _cfg;
  string _serial;
  vector<Sample> _staging;
  vector<uint64_t> _staging_t; // t_us of each staged record
  vector<Channel> _channels;
  size_t _staged{0};
  size_t _n_pins{0};
  size_t _buffer_size{0};
  size_t _chunk_size{0};
  double _sample_rate{1000.0};
  int _period_us{0};
  bool _volts{true};

  DeviceTime _anchor{};      // anchor of the current session
  uint64_t _anchor_t_us{0};  // _anchor.micros64 on the t_us timeline
  TimeUnwrapper _unwrapper{};
  SessionTimeline _timeline{};
  json _time_ref;

  StreamTotals _base{};        // counters of the sessions that ended
  StreamTotals _last_totals{}; // totals as of the previous frame

  bool _restart_enabled{true};
  uint64_t _max_restarts{0};
  std::future<DeviceTime> _restart; // valid while an attempt runs or awaits collection
  std::chrono::steady_clock::time_point _next_attempt{};
  unsigned _failed_attempts{0};
  string _failure; // why the last stream stopped
  string _fatal;   // set when a stopped stream will not be restarted
  uint64_t _restarts{0};
  uint64_t _last_restarts{0};
  uint64_t _gap_us{0};
  uint64_t _last_gap_us{0};
  uint64_t _gap_records{0};
  uint64_t _last_gap_records{0};
  bool _gap_pending{false};
};


/*
  ____  _             _             _      _
 |  _ \| |_   _  __ _(_)_ __     __| |_ __(_)_   _____ _ __
 | |_) | | | | |/ _` | | '_ \   / _` | '__| \ \ / / _ \ '__|
 |  __/| | |_| | (_| | | | | | | (_| | |  | |\ V /  __/ |
 |_|   |_|\__,_|\__, |_|_| |_|  \__,_|_|  |_| \_/ \___|_|
                |___/
Enable the class as plugin
*/
MADS_REGISTER_PLUGINS(UsbstreamPlugin)


/*
                  _
  _ __ ___   __ _(_)_ __
 | '_ ` _ \ / _` | | '_ \
 | | | | | | (_| | | | | |
 |_| |_| |_|\__,_|_|_| |_|

For testing purposes, when directly executing the plugin.

Always runs the TimeUnwrapper and SessionTimeline checks (no hardware
needed). With --offline, stops there. With --soak SECONDS, runs soak() below
instead of the hardware checks. Otherwise looks for a streaming-capable
device and, if found, exercises get_output() over the bulk stream at
chunk_size = 0 and then at chunk_size = 50; if no device streams, prints
SKIPPED and exits 0.
*/

// Streams pins 15 and 16 at 10 kHz for `seconds`, calling get_output() every
// 100 ms as an agent would, and prints every warning and error. Checks what
// must hold across stream restarts: no critical, t_us strictly increasing
// from frame to frame, qos.totals never decreasing. Meant for a link that
// fails (e.g. a Portenta H7 plugged straight into a Mac), and for unplugging
// and replugging the board while it runs.
static int soak(double seconds) {
  using namespace std::chrono;
  UsbstreamPlugin plugin;
  json params;
  params["pin_modes"]["15"] = "ANALOG";
  params["pin_modes"]["16"] = "ANALOG";
  params["sample_rate"] = 10000.0;
  plugin.set_params(params);

  const auto start = steady_clock::now();
  const auto deadline = start + duration_cast<steady_clock::duration>(
                                    duration<double>(seconds));
  auto stamp = [&] {
    ostringstream oss;
    oss << fixed << setprecision(1)
        << duration<double>(steady_clock::now() - start).count() << " s";
    return oss.str();
  };

  bool ok = true;
  uint64_t frames = 0, records = 0, warnings = 0, errors = 0, last_t = 0;
  json last_totals;
  while (steady_clock::now() < deadline) {
    std::this_thread::sleep_for(milliseconds(100));
    json out;
    const return_type rt = plugin.get_output(out);
    if (rt == return_type::critical) {
      cerr << stamp() << " FAIL: critical: " << plugin.error() << endl;
      ok = false;
      break;
    }
    if (rt == return_type::error) {
      ++errors;
      cout << stamp() << " error: " << plugin.error() << endl;
      continue;
    }
    if (rt == return_type::retry) {
      continue;
    }
    if (rt == return_type::warning) {
      ++warnings;
      cout << stamp() << " warning: " << plugin.error() << endl;
    }
    ++frames;
    for (const auto &tv : out["t_us"]) {
      const uint64_t t = tv.get<uint64_t>();
      if (records > 0 && t <= last_t) {
        cerr << stamp() << " FAIL: t_us went from " << last_t << " to " << t
             << endl;
        ok = false;
      }
      last_t = t;
      ++records;
    }
    const json &totals = out["qos"]["totals"];
    if (!last_totals.is_null()) {
      for (const auto &[key, value] : totals.items()) {
        if (value.get<uint64_t>() < last_totals[key].get<uint64_t>()) {
          cerr << stamp() << " FAIL: qos.totals." << key << " decreased" << endl;
          ok = false;
        }
      }
    }
    last_totals = totals;
  }

  cout << "SOAK: " << frames << " frames, " << records << " records, "
       << warnings << " warnings, " << errors << " errors; totals "
       << last_totals.dump() << endl;
  cout << (ok ? "SOAK PASSED" : "SOAK FAILED") << endl;
  return ok ? 0 : 1;
}

int main(int argc, char const *argv[]) {
  bool offline = false;
  double soak_seconds = 0.0;
  for (int i = 1; i < argc; ++i) {
    if (string(argv[i]) == "--offline") offline = true;
    if (string(argv[i]) == "--soak" && i + 1 < argc) soak_seconds = stod(argv[++i]);
  }

  bool all_ok = true;
  auto check = [&](const string &name, bool cond) {
    cout << (cond ? "PASS" : "FAIL") << ": " << name << endl;
    if (!cond) all_ok = false;
  };

  // ---- TimeUnwrapper checks (pure, no hardware) --------------------------
  {
    TimeUnwrapper u;
    u.seed(0);
    uint64_t a = u.unwrap(10);
    uint64_t b = u.unwrap(20);
    uint64_t c = u.unwrap(30);
    check("TimeUnwrapper: plain increasing values", a == 10 && b == 20 && c == 30);
  }
  {
    TimeUnwrapper u;
    u.seed(0);
    uint64_t a = u.unwrap(0xFFFFFF00u);
    uint64_t b = u.unwrap(0x10u);
    check("TimeUnwrapper: wrap bumps the high word",
          a == 0xFFFFFF00ull && b == ((uint64_t{1} << 32) | 0x10ull));
  }
  {
    TimeUnwrapper u;
    uint64_t anchor = (uint64_t{5} << 32) | 0xFFFFFFF0u;
    u.seed(anchor);
    uint64_t a = u.unwrap(0x20u);
    check("TimeUnwrapper: seeding edge case (first sample past a wrap)",
          a == ((uint64_t{6} << 32) | 0x20ull));
  }
  {
    TimeUnwrapper u;
    uint64_t anchor = (uint64_t{5} << 32) | 0x100u;
    u.seed(anchor);
    uint64_t a = u.unwrap(0x200u);
    check("TimeUnwrapper: no spurious wrap at seed",
          a == ((uint64_t{5} << 32) | 0x200ull));
  }

  // ---- SessionTimeline checks (pure, no hardware) ------------------------
  {
    SessionTimeline tl;
    tl.start_session(1'000'000, 0);
    check("SessionTimeline: first session is the device clock",
          tl.map(1'000'100) == 1'000'100 && !tl.reset);
  }
  {
    // The board kept running through a 2 s outage: t_us stays its clock.
    SessionTimeline tl;
    tl.start_session(1'000'000, 0);
    tl.map(5'000'000);
    tl.start_session(7'000'000, 6'000'000);
    check("SessionTimeline: restart on a running board keeps the device clock",
          !tl.reset && tl.map(7'000'100) == 7'000'100);
  }
  {
    // An hour at 83 ppm of drift is not a reset.
    SessionTimeline tl;
    tl.start_session(0, 0);
    tl.map(3'599'000'000);
    tl.start_session(3'599'700'000, 3'600'000'000);
    check("SessionTimeline: clock drift within tolerance is not a reset",
          !tl.reset && tl.map(3'599'700'100) == 3'599'700'100);
  }
  {
    // Replugged: the board rebooted and its clock restarted at 2 s, 31 s
    // after the previous anchor. t_us continues on the host's timeline.
    SessionTimeline tl;
    tl.start_session(60'000'000, 0);
    tl.map(90'000'000);
    tl.start_session(2'000'000, 31'000'000);
    check("SessionTimeline: a board reset bridges the gap with host time",
          tl.reset && tl.map(2'000'000) == 91'000'000 &&
              tl.map(2'000'100) == 91'000'100);
  }
  {
    // Host time says less went by than the old session's own records:
    // t_us must still move forward.
    SessionTimeline tl;
    tl.start_session(1'000'000, 0);
    tl.map(11'000'000);
    tl.start_session(500'000, 5'000'000);
    check("SessionTimeline: t_us never goes backwards",
          tl.reset && tl.map(500'000) == 11'000'001);
  }

  if (!all_ok) {
    cerr << "Pure checks FAILED" << endl;
    return 1;
  }
  cout << "All pure checks PASSED" << endl;

  if (offline) {
    return 0;
  }
  if (soak_seconds > 0.0) {
    return soak(soak_seconds);
  }

  // ---- Hardware checks (skipped when no streaming device is attached) ---
  bool have_device = true;
  try {
    string devices = USBDriver::list_devices();
    if (devices.empty()) have_device = false;
  } catch (const ArduinoDriver::Error &e) {
    have_device = false;
  } catch (const exception &e) {
    have_device = false;
  }

  if (!have_device) {
    cout << "SKIPPED: no streaming device" << endl;
    return 0;
  }

  // Scope 1: chunk_size = 0 (publish everything staged each tick).
  {
    UsbstreamPlugin plugin;
    json params;
    params["pin_modes"]["15"] = "ANALOG";
    params["sample_rate"] = 1000.0;
    params["chunk_size"] = 0;
    plugin.set_params(params);

    vector<json> frames;
    bool hw_ok = true;

    for (int i = 0; i < 12 && hw_ok; ++i) {
      if (i > 0) std::this_thread::sleep_for(std::chrono::milliseconds(100));
      json out;
      return_type rt = plugin.get_output(out);
      if (rt == return_type::critical) {
        // Only a genuine "this board doesn't stream" is a SKIP. Any other
        // critical on the very first call (board already in use by another
        // process, pin 15 not analog-capable, bad params, ...) is a real
        // test failure, not something to wave away.
        if (i == 0 &&
            plugin.error().find("does not support bulk streaming") != string::npos) {
          cout << "SKIPPED: no streaming device (" << plugin.error() << ")" << endl;
          return 0;
        }
        cerr << "FAIL: get_output() returned critical: " << plugin.error() << endl;
        hw_ok = false;
        break;
      }
      if (rt == return_type::error) {
        cerr << "FAIL: get_output() returned error: " << plugin.error() << endl;
        hw_ok = false;
        break;
      }
      if (rt == return_type::success || rt == return_type::warning) {
        frames.push_back(out);
      }
      // retry: nothing staged yet, keep going.
    }

    if (hw_ok && frames.empty()) {
      cerr << "FAIL: chunk_size=0 published no frames in ~1.1 s at 1 kHz" << endl;
      hw_ok = false;
    }

    uint64_t total_records = 0, first_t = 0, last_t = 0, prev_t = 0;
    bool first_sample = true;
    for (const auto &f : frames) {
      if (!hw_ok) break;
      if (!f.contains("qos") || !f["qos"].contains("records")) {
        cerr << "FAIL: frame missing qos.records" << endl;
        hw_ok = false;
        break;
      }
      size_t records = f["qos"]["records"].get<size_t>();
      if (!f.contains("analog") || !f["analog"].contains("15") ||
          f["analog"]["15"].size() != records || f["t_us"].size() != records) {
        cerr << "FAIL: analog[15]/t_us size does not match qos.records" << endl;
        hw_ok = false;
        break;
      }
      for (const auto &tv : f["t_us"]) {
        uint64_t t = tv.get<uint64_t>();
        if (!first_sample && t <= prev_t) {
          cerr << "FAIL: t_us is not strictly increasing" << endl;
          hw_ok = false;
          break;
        }
        if (first_sample) {
          first_t = t;
          first_sample = false;
        }
        prev_t = t;
        last_t = t;
        ++total_records;
      }
    }

    if (hw_ok) {
      if (total_records < 2 || last_t <= first_t) {
        cerr << "FAIL: not enough records to measure the achieved rate" << endl;
        hw_ok = false;
      } else {
        double rate = static_cast<double>(total_records - 1) * 1.0e6 /
                      static_cast<double>(last_t - first_t);
        double rel_err = std::fabs(rate - 1000.0) / 1000.0;
        cout << "SUMMARY: chunk_size=0 frames=" << frames.size()
             << " records=" << total_records << " rate=" << rate << " Hz" << endl;
        if (rel_err > 0.05) {
          cerr << "FAIL: achieved rate " << rate << " Hz is off target 1000 Hz by "
               << (rel_err * 100.0) << "%" << endl;
          hw_ok = false;
        }
      }
    }

    check("hardware: streaming at 1 kHz, chunk_size=0", hw_ok);
  } // plugin (and its Stream/Device) released here

  // Scope 2: chunk_size = 50 (every published frame has exactly 50 records).
  {
    UsbstreamPlugin plugin;
    json params;
    params["pin_modes"]["15"] = "ANALOG";
    params["sample_rate"] = 1000.0;
    params["chunk_size"] = 50;
    plugin.set_params(params);

    vector<json> frames;
    bool hw_ok = true;

    for (int i = 0; i < 12 && hw_ok; ++i) {
      if (i > 0) std::this_thread::sleep_for(std::chrono::milliseconds(100));
      json out;
      return_type rt = plugin.get_output(out);
      if (rt == return_type::critical || rt == return_type::error) {
        cerr << "FAIL: get_output() returned " << (rt == return_type::critical ? "critical" : "error")
             << ": " << plugin.error() << endl;
        hw_ok = false;
        break;
      }
      if (rt == return_type::success || rt == return_type::warning) {
        frames.push_back(out);
      }
    }

    if (hw_ok && frames.empty()) {
      cerr << "FAIL: chunk_size=50 published no frames" << endl;
      hw_ok = false;
    }

    for (const auto &f : frames) {
      if (!hw_ok) break;
      size_t records = f["qos"]["records"].get<size_t>();
      if (records != 50 || f["t_us"].size() != 50 || f["analog"]["15"].size() != 50) {
        cerr << "FAIL: chunk_size=50 frame does not have exactly 50 records" << endl;
        hw_ok = false;
      }
    }

    if (hw_ok) {
      cout << "SUMMARY: chunk_size=50 frames=" << frames.size() << ", all exactly 50 records"
           << endl;
    }

    check("hardware: streaming at 1 kHz, chunk_size=50", hw_ok);
  }

  if (!all_ok) {
    cerr << "Hardware checks FAILED" << endl;
    return 1;
  }
  cout << "All checks PASSED" << endl;
  return 0;
}
