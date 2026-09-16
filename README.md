# ArduinoIO_plugin

A MADS agent using <https://github.com/pbosetti/ArduinoDriver> for zero-programming Arduino I/O.

ArduinoDriver provides both a firmware for most Arduino boards and a user land C++20 driver (plus CLI commands) for directly tapping into I/O pins (including analog and PWM and DAC) via USB interface rather than through Serial Port. This allows to select pins and operations from `mads.ini` plugin settings, and also exploiting higher rates of USB ports wrt serials.


This is a Source plugin for [MADS](https://github.com/MADS-NET/MADS). 


*Required MADS version: 2.4.3.*


## Supported platforms

Currently, the supported platforms are:

* **Linux** 
* **MacOS**
* **Windows**

## Boards

Look at <https://github.com/pbosetti/ArduinoDriver> for supported boards. The plugin is expected to work with all boards supported by ArduinoDriver.

In the library manager of Arduino IDE, search for "UsbIo" and install it. Then, upload the firmware to your board:

```cpp
#include <UsbIo.h>

void setup() {
  UsbIo.begin();
}

void loop() {
  UsbIo.poll();
}
```


## Installation

Linux and MacOS:

```bash
cmake -Bbuild -DCMAKE_INSTALL_PREFIX="$(mads -p)"
cmake --build build -j4
sudo cmake --install build
```

Windows:

```powershell
cmake -Bbuild -DCMAKE_INSTALL_PREFIX="$(mads -p)"
cmake --build build --config Release
cmake --install build --config Release
```


## INI settings

The plugin supports the following settings in the INI file:

```ini
[arduinousb_source]
period = 100
pin_modes = {"1"="PULLDOWN", "2"="PULLDOWN", "3"="PULLUP", "15"="ANALOG"}
# serial = ""   # USB serial number to open; empty (default) = first device found

[arduinousb_sink]
pin_modes = {"1"="OUTPUT", "2"="OUTPUT", "4"="PWM"}
# serial = ""   # USB serial number to open; empty (default) = first device found
```

All settings are optional; if omitted, the default values are used.

**NOTE**: by default, the plugin connects to the first available Arduino board found on the USB bus. Set the optional `serial` key to the board's USB serial number to pick a specific one when more than one is attached. if you **install** the plugin (`cmake --install build`), it also installs the `arduino-io` command line utility, which can be used to test the plugin and to check the connected board. Just type `arduino-io` to see the available commands.


## Executable demo

<Explain what happens if the test executable is run>

---

## `arduinostream` plugin: bulk USB streaming

`arduinousb_source` polls pins one control transfer at a time, once per MADS
tick — a few hundred Hz at most. Boards that advertise
`USBIO_FLAG_STREAMING` (e.g. the Portenta H7) also expose continuous
sampling over a bulk USB endpoint, at rates up to 10 kHz. `arduinostream.plugin`
(class `UsbstreamPlugin`, driver name `arduinostream`) drives that path: it
samples the configured pins on the device at a fixed rate, buffers the
decoded samples on the host, and publishes them in chunks at the slower MADS
tick rate, together with QoS metadata (loss counters, achieved rate,
latency).

**Caveat — one streaming agent per board.** Starting a stream marks the
device "busy" so that every *other call this same process* makes to it
throws — but that guard is per-process, inside one `Device` object; a second
MADS agent is a separate process with its own `Device` and does not see it.
In practice, another agent pointed at the *same* board either fails to open
it at all (its USB interface is already claimed by the streaming process),
or — if it does get through — its `PIN_MODE` on a streamed pin, or a
`RESET`, stops this plugin's stream (this plugin's next `get_output()` then
returns `critical`). Run only one of
`arduinousb_source`/`arduinousb_sink`/`arduinostream` against a given board
at a time — see the comment next to `[arduinostream]` in `director.toml`.

### INI settings

```ini
[arduinostream]
period = 100
sample_rate = 5000
pin_modes = {"15"="ANALOG", "16"="ANALOG"}
# buffer_size = 0   # records held on the host; 0 = auto (>= 1024, or ~1 s of data)
# chunk_size = 0    # records per published frame; 0 = publish everything staged each tick
# volts = true      # analog samples as volts (true) or raw ADC codes (false)
# serial = ""       # USB serial number to open; empty = first device found
```

| Key | Type | Default | Meaning |
|---|---|---|---|
| `serial` | string | `""` | USB serial number to open; empty = first identified device. |
| `pin_modes` | table | `{"15"="ANALOG"}` | Pins to stream, and their mode. Allowed: `ANALOG`, `INPUT`, `PULLUP`, `PULLDOWN`. At most `info().stream_max_channels` pins; replaces (does not merge with) the default when given. |
| `sample_rate` | double, Hz | `1000` | Device sampling rate. `0` = free-running (device samples as fast as it can). Otherwise converted to a period clamped to 100..65535 µs (about 15.3 Hz .. 10 kHz); out-of-range values are rejected at startup. |
| `buffer_size` | int, records | `0` (auto) | Host-side queue capacity (`StreamConfig::queue_capacity`). `0` = `max(1024, ceil(sample_rate * 1 s))`, assuming 10000 Hz when free-running. On overflow the oldest record is dropped and counted as a host drop. |
| `chunk_size` | int, records | `0` | `0` = publish every record staged since the last tick (a variable-size frame). `N` = publish exactly `N` records per frame, holding any extra for the next tick. |
| `volts` | bool | `true` | Analog samples as volts (`true`) or raw ADC codes (`false`). |

Note that `period` here is the agent's MADS loop period (how often
`get_output()` runs) — a reserved key handled by the host agent, unrelated to
`sample_rate`, which is the device's own sampling clock.

### Example output frame

```json
{
  "t_us": [1000000, 1001000, 1002000],
  "analog": {
    "15": [1.643, 1.646, 1.641],
    "16": [0.021, 0.020, 0.021]
  },
  "qos": {
    "records": 3,
    "rate_hz": 1000.0,
    "seq_gaps": 0,
    "host_drops": 0,
    "device_overruns": 0,
    "resyncs": 0,
    "totals": {
      "seq_gaps": 0,
      "host_drops": 0,
      "device_overruns": 0,
      "resyncs": 0,
      "stale_records": 0,
      "records_received": 15003
    },
    "latency_ms": 2.4
  },
  "time_ref": {
    "device_us": 998531,
    "host": "2026-09-16T10:11:12.345Z",
    "uncertainty_us": 350
  },
  "agent_id": "arduinostream"
}
```

- `t_us`: device `micros()` for each record, unwrapped past the
  ~71.6-minute `uint32` rollover into a monotonically increasing `uint64`.
- `analog` / `digital`: one array per streamed pin, `qos.records` values
  long, aligned with `t_us`. Pins in `ANALOG` mode go under `analog`; pins in
  `INPUT`/`PULLUP`/`PULLDOWN` mode go under `digital` as 0/1.
- `qos.records`: number of records in this frame.
- `qos.rate_hz`: the rate actually achieved across this frame's records,
  `(records - 1) / (t_us[last] - t_us[first])`; `0` when fewer than 2
  records or they share one timestamp.
- `qos.seq_gaps` / `qos.host_drops` / `qos.device_overruns` / `qos.resyncs`:
  **changes since the previous frame** — records lost between the device and
  the host, records the host had to drop because `buffer_size` overflowed,
  device-side ADC/USB overruns, and byte-stream resyncs, respectively.
  Device overruns are already counted inside `seq_gaps`, so do not add them
  again when computing "records lost".
- `qos.totals`: the same four counters as running totals since the stream
  started, plus `records_received` and `stale_records` (records left over in
  the board's USB endpoint by an earlier session, which the driver
  recognises by their timestamp and discards).
- `qos.latency_ms`: how far behind wall-clock time the last sample in this
  frame is by the time the frame is built.
- `time_ref`: anchors the device clock to the host clock, captured once when
  the stream starts. `device_us` is the device's `micros()` at that instant,
  `host` its ISO-8601 UTC equivalent (millisecond precision), and
  `uncertainty_us` is half the round-trip time of the control transfer used
  to measure it. The absolute time of sample `i` is
  `time_ref.host + (t_us[i] - time_ref.device_us)` microseconds.

### Loss and errors

If `seq_gaps` or `host_drops` increased since the previous frame,
`get_output()` returns `warning` — the frame is still published, with a
message describing the loss attached under the `warning` key — instead of
`success`. If the stream stops (the device is unplugged, or a USB transfer
fails), `get_output()` returns `critical` with the driver's reason, e.g.
`stream stopped: bulk IN transfer failed: LIBUSB_ERROR_IO`, and the agent
stops; restarting the agent (e.g. `relaunch = true` in `director.toml`) opens
a clean session.

### Troubleshooting: Portenta H7 and High Speed USB

The Portenta H7 streams over a USB **High Speed** (480 Mbit/s) link, which is
sensitive to signal quality. Plugged straight into a Mac's USB-C port, streams
were seen to die every few seconds with `LIBUSB_ERROR_IO` (macOS: "device not
responding"), and occasionally the board stopped answering altogether until
it was replugged, with the sketch itself still running. Larger bulk packets
made it worse, pointing at bit errors on the link rather than at the firmware.
Connecting the board through a **powered USB hub** removed the problem
entirely: 10 x 60 s at 10 kHz with no failures. If you see these symptoms,
put a powered hub between the computer and the board, and prefer a short
cable.

### Tips

- At high sample rates a chunk can carry thousands of records per tick; set
  `wire_format = "msgpack"` on the agent (or fleet-wide under `[agents]`) to
  keep the payload compact.
- Size `buffer_size` to absorb at least one MADS `period` worth of samples
  at `sample_rate`, with margin; `info()` (printed at startup) adds a note
  when it covers less than 200 ms.

---