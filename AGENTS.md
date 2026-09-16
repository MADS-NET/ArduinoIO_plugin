# ArduinoIO_plugin — working notes for AI coding agents

This project builds **two** MADS plugin binaries from `src/`. Both link
`ArduinoDriver::arduino_driver`, which requires `cxx_std_20`; CMake
propagates that requirement to every target that links it, so both plugins
are actually built as **C++20** even though `CMakeLists.txt` sets
`CMAKE_CXX_STANDARD 17` as the project-wide default.

Framework context — the plugin lifecycle, the order in which the host agent
calls each method, what every return value does, how settings reach the plugin,
topics, blobs, testing, deployment and protocol migration — is in the
`mads-plugin` skill under [`.claude/skills/mads-plugin/`](.claude/skills/mads-plugin/SKILL.md).
**Read `SKILL.md` before changing plugin code**, and follow its `reference/`
files rather than inferring host behaviour from the template comments.

Regenerate that skill for a newer MADS with `mads plugin --update`.

## Plugins in this project

Both plugins share one mixin, [`src/usb_driver.hpp`](src/usb_driver.hpp)
(class `USBDriver`): it owns the `ArduinoDriver::Context`/`Device`, opens the
device (`open(serial)`, called from `set_params()`, never from the
constructor — MADS constructs the plugin object before any settings exist),
and centralizes pin-mode handling (`read_pin_modes()` validates and records
into `_pin_modes`; `apply_pin_modes()` sends `PIN_MODE` for each of them).

### `arduinousb.plugin` — polled digital/analog I/O

- Source: [`src/arduinousb.cpp`](src/arduinousb.cpp)
- Behavior: **source**, class `UsbsourcePlugin` — polls the configured pins
  once per MADS tick over a USB control transfer each (a few hundred Hz at
  most).
- Behavior: **sink**, class `UsbsinkPlugin` — writes `digital`/`pwm`/`dac`
  values from the input frame to pins configured in the matching mode.
- Both classes are registered under the CMake target name `arduinousb`
  (`MADS_REGISTER_PLUGINS(UsbsourcePlugin, UsbsinkPlugin)`, driver name =
  `PLUGIN_NAME` = `"arduinousb"`). **Note:** `kind()` on both classes
  currently returns a hardcoded literal (`"usbsource"` / `"usbsink"`)
  instead of `PLUGIN_NAME`; in protocol P8 this only produces a startup
  warning (`kind()` no longer selects the settings section — the *agent
  name* does), but it is a latent inconsistency worth fixing if
  `arduinousb.cpp` is touched again.
- INI section: named after the agent (`-n`), e.g. `[arduinousb_source]`,
  `[arduinousb_sink]` in `mads.ini`.

| Key | Type | Default | Meaning |
|---|---|---|---|
| `serial` | string | `""` | USB serial number to open; empty = first device found. |
| `pin_modes` | table | source: `{"1"="PULLDOWN"}`; sink: `{}` | Pin → mode. Source allows `INPUT`, `PULLUP`, `PULLDOWN`, `ANALOG`; sink allows `OUTPUT`, `PWM`, `DAC`. |

Source output frame: `{"digital": {"<pin>": 0|1, ...}, "analog": {"<pin>": <volts>, ...}, "agent_id": "..."}`
— one scalar reading per configured pin, once per tick. Sink input frame:
`{"digital": {"<pin>": true|false}, "pwm": {"<pin>": <0..1>}, "dac": {"<pin>": <volts>}}`.

### `arduinostream.plugin` — bulk USB streaming

- Source: [`src/arduinostream.cpp`](src/arduinostream.cpp)
- Behavior: **source**, class `UsbstreamPlugin`, registered alone
  (`MADS_REGISTER_PLUGINS(UsbstreamPlugin)`; `kind()` correctly returns
  `PLUGIN_NAME`, i.e. the CMake target name `arduinostream`).
- On boards that expose `USBIO_FLAG_STREAMING` (e.g. Portenta H7), samples
  the configured pins on the device at a fixed rate (up to 10 kHz) over the
  bulk IN endpoint (`ArduinoDriver::Device::start_stream()` /
  `ArduinoDriver::Stream`), buffers decoded samples on the host without
  blocking (`Stream::read(span, 0ms)` drained in `get_output()`), and
  publishes them in chunks with QoS metadata (loss counters, achieved rate,
  latency). See the README's `arduinostream` section for the full settings
  table, an example frame and the QoS field meanings — keep the two in
  sync.
- INI section: `[arduinostream]` (or whatever `-n` selects).
- Key internal pieces: `TimeUnwrapper` (turns the device's wrapping 32-bit
  `micros()` into a monotonic 64-bit timestamp, seeded from a `DeviceTime`
  anchor taken in `set_params()` before the stream starts — `read_time()`
  throws `DeviceBusy` once a `Stream` is running); `_staging`, a flat
  `vector<Sample>` drained from the `Stream` queue every tick and shifted
  left after each published chunk.
- **Caveat**: starting a stream marks the `Device` "busy", but that guard is
  per-process (inside this plugin's own `Device` object) — a second MADS
  agent is a separate process with its own `Device` and does not get
  `DeviceBusy` for free. In practice it either fails to open the same board
  at all (the USB interface is already claimed) or, if it does open it, its
  `PIN_MODE`/`RESET` on the board stops this plugin's stream (the next
  `get_output()` then returns `critical`). Do not run `arduinousb_source` /
  `arduinousb_sink` and `arduinostream` against the same board at the same
  time (see the comment in `director.toml`).

## Output frames

See each plugin's subsection above, and the README for `arduinostream`'s
full frame example (`t_us`, `analog`/`digital`, `qos`, `time_ref`).

## Conventions for this project

- C++20 in practice (see the note at the top of this file), built with
  CMake/Ninja; LLVM formatting, two-space indent.
- `CamelCase` for classes and namespaces, `snake_case` for methods and
  variables, `_leading_underscore` for private members, declared last.
- Do not add third-party dependencies without asking. `nlohmann/json` and
  `ArduinoDriver::arduino_driver` are already available via `FetchContent`,
  and the plugin base classes also provide a `SerialPort` helper in
  `serialport.hpp` (unused by these two plugins).
- All plugin logic must be reachable from the test `main()` at the bottom of
  each source file, and that test must be deterministic and self-checking:
  assert on both the returned status and the payload, and exit non-zero on
  failure. `arduinostream.cpp`'s `main()` also accepts `--offline`, which
  runs only its hardware-free `TimeUnwrapper` checks and skips anything that
  opens a USB device — use that flag whenever a real board must not be
  touched.
- An exception must never escape a plugin method: catch it, set `_error` and
  return `return_type::error` (or `critical` in `set_params()`'s effect —
  see `reference/return-types.md`).
- Never block, sleep or busy-wait inside a plugin method. `arduinostream`'s
  `get_output()` drains its stream with an explicit `0ms` timeout for this
  reason.

## Build and test

```sh
# Once ArduinoDriver v0.2.2+ is tagged and published:
cmake -Bbuild -DCMAKE_INSTALL_PREFIX="$(mads -p)"

# Until then, against a local ArduinoDriver checkout:
cmake -Bbuild -DFETCHCONTENT_SOURCE_DIR_ARDUINODRIVER=/path/to/ArduinoDriver

cmake --build build -j4
./build/arduinousb.plugin                  # standalone test driver (opens a real board)
./build/arduinostream.plugin --offline     # standalone test driver, no hardware
./build/arduinostream.plugin               # same, plus hardware checks if a streaming board is attached
mads inspect_plugin build/arduinousb.plugin
mads inspect_plugin build/arduinostream.plugin
mads source build/arduinousb.plugin -n arduinousb_source
mads source build/arduinostream.plugin
```

On macOS both `.plugin` files are also directly-executable test drivers (see
`add_plugin()` in `CMakeLists.txt`); on Linux/Windows the test driver is a
separate `<name>` / `<name>.exe` binary built alongside the `.plugin`
shared library.
