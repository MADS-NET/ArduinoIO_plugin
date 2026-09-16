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
class UsbstreamPlugin : public Source<json>, public USBDriver {

public:
  using Source::Source; // inherit constructors

  // A Stream must not outlive the Device owned by the USBDriver base.
  // Member destruction order already guarantees this (derived members are
  // destroyed before base subobjects), but reset explicitly to make the
  // requirement hold even if that ever changes.
  ~UsbstreamPlugin() override { _stream.reset(); }

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

    _params.merge_patch(params);
    // merge_patch merges JSON objects key by key; a user-supplied pin_modes
    // table must REPLACE the default {"15": "ANALOG"}, not merge with it.
    if (params.contains("pin_modes")) {
      _params["pin_modes"] = params["pin_modes"];
    }

    try {
      open(_params["serial"].get<string>());

      if (!_dev->info().streaming()) {
        throw runtime_error(
            "Board " + string(board_name(_dev->info().board_id)) +
            " does not support bulk streaming (USBIO_FLAG_STREAMING not set)");
      }

      read_pin_modes(_params["pin_modes"], _allowed_pin_modes);
      if (_pin_modes.empty()) {
        throw runtime_error("pin_modes must not be empty: at least one "
                             "streamed pin is required");
      }
      apply_pin_modes();

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

      // Anchor device time to host time BEFORE starting the stream: once the
      // Stream runs, every other Device call (including read_time()) throws
      // DeviceBusy. Take the steady/system clock pair together, right after,
      // so the anchor's steady host_time can be mapped to a system_clock
      // time for the ISO-8601 string below.
      DeviceTime anchor = _dev->read_time();
      auto steady_now = std::chrono::steady_clock::now();
      auto system_now = std::chrono::system_clock::now();
      _anchor = anchor;
      auto anchor_system =
          system_now + std::chrono::duration_cast<std::chrono::microseconds>(
                            anchor.host_time - steady_now);
      _time_ref = json{{"device_us", anchor.micros64},
                        {"host", format_iso8601_utc(anchor_system)},
                        {"uncertainty_us", anchor.round_trip.count() / 2}};

      vector<uint8_t> pins;
      pins.reserve(_pin_modes.size());
      for (const auto &[pin, mode] : _pin_modes) {
        pins.push_back(pin);
      }

      StreamConfig cfg;
      cfg.pins = pins;
      cfg.period = period;
      cfg.flags = 0;
      cfg.queue_capacity = static_cast<size_t>(buffer_size);

      _stream.emplace(_dev->start_stream(cfg));

      // Precompute the per-channel layout once, from the Stream's own pin
      // order (== `pins` above, but this is the authoritative source): the
      // hot path in get_output() then indexes straight into _channels[j]
      // instead of looking `s.pin` up in _pin_modes for every sample.
      const auto &stream_pins = _stream->pins();
      _channels.clear();
      _channels.reserve(stream_pins.size());
      for (uint8_t pin : stream_pins) {
        _channels.push_back(
            Channel{to_string(pin), _pin_modes.at(pin) == PinMode::AnalogIn});
      }

      _n_pins = pins.size();
      _staging.assign(static_cast<size_t>(buffer_size) * _n_pins, Sample{});
      _staged = 0;

      _unwrapper.seed(anchor.micros64);
      _last_stats = StreamStats{};

      _sample_rate = sample_rate;
      _period_us = period_us;
      _buffer_size = static_cast<size_t>(buffer_size);
      _chunk_size = static_cast<size_t>(chunk_size);
      _volts = _params["volts"].get<bool>();
    } catch (const exception &e) {
      _error = e.what();
      cerr << e.what() << endl;
      _init_error = true;
    }
  }

  // Never blocks: drains whatever the Stream worker already decoded with
  // timeout 0, then either publishes a chunk or returns retry.
  return_type get_output(json &out, vector<unsigned char> * /*blob*/ = nullptr) override {
    if (_init_error) return return_type::critical;
    if (!_stream->running()) {
      _error = "stream stopped (device unplugged?)";
      return return_type::critical;
    }

    out.clear();

    try {
      // 1. Drain the Stream's internal queue without blocking.
      while (_staged < _staging.size()) {
        std::span<Sample> free_tail(_staging.data() + _staged,
                                    _staging.size() - _staged);
        size_t n = _stream->read(free_tail, std::chrono::milliseconds(0));
        if (n == 0) break;
        _staged += n;
      }

      size_t records_staged = _staged / _n_pins;
      if (records_staged == 0 ||
          (_chunk_size > 0 && records_staged < _chunk_size)) {
        return return_type::retry;
      }

      size_t K = _chunk_size > 0 ? _chunk_size : records_staged;

      // 2. Build columns from the first K records. One native vector per
      // channel, pre-reserved and filled by index (no _pin_modes lookup and
      // no map insertion per sample: at 10 kHz x 8 channels that used to be
      // ~160k map ops/tick). Moved into `out` once each at the end, since
      // frames may hold up to 10k records and growing a json array one
      // push_back() at a time is far slower than building a vector first.
      vector<uint64_t> t_us_vec(K);
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
        t_us_vec[i] = _unwrapper.unwrap(record[0].t_us);
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

      // 3. QoS: rate actually achieved, loss counters (delta since the
      // previous frame, plus running totals) and publish latency.
      double rate_hz = 0.0;
      if (K >= 2 && t_last > t_first) {
        rate_hz = static_cast<double>(K - 1) * 1.0e6 /
                  static_cast<double>(t_last - t_first);
      }

      StreamStats stats = _stream->stats();
      // device_overruns are also counted in seq_gaps (see StreamStats), so
      // "lost records" below sums seq_gaps + host_drops only, not overruns.
      uint32_t d_overruns = stats.device_overruns - _last_stats.device_overruns;
      uint64_t d_seq_gaps = stats.seq_gaps - _last_stats.seq_gaps;
      uint64_t d_host_drops = stats.host_drops - _last_stats.host_drops;
      uint64_t d_resyncs = stats.resyncs - _last_stats.resyncs;

      int64_t delta_us =
          static_cast<int64_t>(t_last) - static_cast<int64_t>(_anchor.micros64);
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
      qos["totals"] = {{"seq_gaps", stats.seq_gaps},
                        {"host_drops", stats.host_drops},
                        {"device_overruns", stats.device_overruns},
                        {"resyncs", stats.resyncs},
                        {"records_received", stats.records_received}};
      qos["latency_ms"] = latency_ms;
      out["qos"] = std::move(qos);
      out["time_ref"] = _time_ref;

      _last_stats = stats;

      // 4. Shift the leftover (unconsumed) samples to the front.
      size_t consumed = K * _n_pins;
      std::copy(_staging.begin() + consumed, _staging.begin() + _staged,
                _staging.begin());
      _staged -= consumed;

      size_t remaining_records = _staged / _n_pins;
      next_loop_duration = (_chunk_size > 0 && remaining_records >= _chunk_size)
                                ? std::chrono::milliseconds(1)
                                : std::chrono::milliseconds(0);

      if (!_agent_id.empty()) out["agent_id"] = _agent_id;

      uint64_t lost = d_seq_gaps + d_host_drops;
      if (lost > 0) {
        ostringstream oss;
        oss << "lost " << lost << " records (seq gaps " << d_seq_gaps
            << ", host drops " << d_host_drops << ", device overruns "
            << d_overruns << ")";
        _error = oss.str();
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
    result["serial"] = _params.value("serial", "");
    result["pin modes"] = _params["pin_modes"].dump();
    result["sample_rate"] =
        (_sample_rate > 0.0 ? to_string(_sample_rate) + " Hz" : "0 (free-running)");
    result["period_us"] =
        (_period_us > 0 ? to_string(_period_us) : "0 (free-running)");
    result["buffer_size"] = to_string(_buffer_size);
    result["chunk_size"] = to_string(_chunk_size);
    result["volts"] = _volts ? "true" : "false";

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
  const vector<string> _allowed_pin_modes{"ANALOG", "INPUT", "PULLUP", "PULLDOWN"};

  optional<Stream> _stream;
  vector<Sample> _staging;
  vector<Channel> _channels;
  size_t _staged{0};
  size_t _n_pins{0};
  size_t _buffer_size{0};
  size_t _chunk_size{0};
  double _sample_rate{1000.0};
  int _period_us{0};
  bool _volts{true};

  DeviceTime _anchor{};
  TimeUnwrapper _unwrapper{};
  StreamStats _last_stats{};
  json _time_ref;
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

Always runs the TimeUnwrapper checks (no hardware needed). With --offline,
stops there. Otherwise looks for a streaming-capable device and, if found,
exercises get_output() over the bulk stream at chunk_size = 0 and then at
chunk_size = 50; if no device streams, prints SKIPPED and exits 0.
*/
int main(int argc, char const *argv[]) {
  bool offline = false;
  for (int i = 1; i < argc; ++i) {
    if (string(argv[i]) == "--offline") offline = true;
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

  if (!all_ok) {
    cerr << "Pure checks FAILED" << endl;
    return 1;
  }
  cout << "All pure checks PASSED" << endl;

  if (offline) {
    return 0;
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
