# ArduinoIO_plugin

A MADS agent using <https://github.com/pbosetti/ArduinoDriver> for zero-programming Arduino I/O.

ArduinoDriver provides both a firmware for most Arduino boards and a user land C++20 driver (plus CLI commands) for directly tapping into I/O pins (including analog and PWM and DAC) via USB interface rather than through Serial Port. This allows to select pins and operations from `mads.ini` plugin settings, and also exploiting higher rates of USB ports wrt serials.

Expected release: Fall 2026

This is a Source plugin for [MADS](https://github.com/MADS-NET/MADS). 

<provide here some introductory info>

*Required MADS version: 2.4.3.*


## Supported platforms

Currently, the supported platforms are:

* **Linux** 
* **MacOS**
* **Windows**


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
[usbsource]
# Describe the settings available to the plugin
```

All settings are optional; if omitted, the default values are used.




## Executable demo

<Explain what happens if the test executable is run>

---