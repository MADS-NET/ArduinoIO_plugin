#pragma once

#include <arduino_driver/Device.h>
#include <arduino_driver/Enumerator.h>
#include <arduino_driver/Errors.h>
#include <arduino_driver/Protocol.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// USBDriver: shared helper wrapping one ArduinoDriver::Device.
//
// This is a mixin base, meant to be inherited alongside Source<json> /
// Sink<json> (see arduinousb.cpp) or Filter<json> by any plugin class that
// needs a UsbIo device. It owns the libusb Context and the open Device, and
// centralizes pin-mode validation so every derived plugin parses/applies
// "pin_modes" the same way.
//
// IMPORTANT: the constructor does NOT open a device. MADS constructs the
// plugin object before any settings exist (see plugin lifecycle docs), so a
// constructor that opens the device would throw at plugin-load time, before
// set_params() gets a chance to catch anything. Call open() from
// set_params() instead, inside a try/catch.
class USBDriver {
public:
  // Allowed values of a "pin_modes" table entry.
  const std::map<std::string, ArduinoDriver::PinMode> modes{
      {"INPUT", ArduinoDriver::PinMode::Input},
      {"OUTPUT", ArduinoDriver::PinMode::Output},
      {"PULLUP", ArduinoDriver::PinMode::InputPullup},
      {"PULLDOWN", ArduinoDriver::PinMode::InputPulldown},
      {"ANALOG", ArduinoDriver::PinMode::AnalogIn},
      {"PWM", ArduinoDriver::PinMode::Pwm},
      {"DAC", ArduinoDriver::PinMode::Dac}};

  // Does nothing: no device is opened here. See open().
  USBDriver() = default;

  ~USBDriver() {
    // A Device must not outlive the Context it was opened from. Declaration
    // order below already destroys _dev before _ctx, but reset explicitly
    // (in this order) so the requirement holds even if that ever changes.
    _dev.reset();
    _ctx.reset();
  }

  // Opens the device: the first identified one when `serial` is empty,
  // otherwise the one carrying that USB serial number. Called from
  // set_params(), once settings are known. Lets ArduinoDriver::Error (e.g.
  // DeviceNotFound) and anything else propagate; the caller is expected to
  // catch, record _error and set _init_error.
  void open(const std::string &serial) {
    if (serial.empty()) {
      _dev = std::make_unique<ArduinoDriver::Device>(
          ArduinoDriver::open_first(_ctx));
    } else {
      _dev = std::make_unique<ArduinoDriver::Device>(
          ArduinoDriver::open_by_serial(_ctx, serial));
    }
  }

  // Lists identified UsbIo devices on the bus, one line each. No device
  // needs to be open for this (it uses its own throwaway Context).
  static std::string list_devices() {
    auto ctx = std::make_shared<ArduinoDriver::Context>();
    std::string result;
    ArduinoDriver::EnumerateOptions enum_options;
    enum_options.probe = true;
    enum_options.probe_timeout = std::chrono::milliseconds(500);
    auto devices = ArduinoDriver::list_devices(ctx, enum_options);
    for (const auto &dev : devices) {
      if (!dev.identified) {
        continue;
      }
      result += "- " + std::string(ArduinoDriver::board_name(dev.info->board_id)) +
                ", Serial: " + dev.serial +
                ", VID: " + std::to_string(dev.vid) +
                ", PID: " + std::to_string(dev.pid) + " " + "\n";
    }
    return result;
  }

protected:
  // Validates `obj` (a pin -> mode string dictionary) against
  // `allowed_modes` and records the result into _pin_modes. Pure validation:
  // no USB traffic happens here, so it is safe to call before the device is
  // open. Call apply_pin_modes() afterwards to actually configure the pins.
  // Throws std::runtime_error on any validation failure.
  void read_pin_modes(const nlohmann::json &obj,
                      const std::vector<std::string> &allowed_modes) {
    if (!obj.is_object()) {
      throw std::runtime_error("Pin modes must be a dictionary");
    }
    for (const auto &[pin, mode] : obj.items()) {
      // read the pin number and mode from the json object
      if (!mode.is_string()) {
        throw std::runtime_error("Pin mode must be a string");
      }
      std::string mode_str = mode.get<std::string>();
      if (modes.find(mode_str) == modes.end()) {
        throw std::runtime_error("Invalid pin mode: " + mode_str);
      }
      if (pin.empty() || !std::all_of(pin.begin(), pin.end(), ::isdigit)) {
        throw std::runtime_error("Pin must be a number: " + pin);
      }
      if (std::find(allowed_modes.begin(), allowed_modes.end(), mode_str) ==
          allowed_modes.end()) {
        throw std::runtime_error("Pin mode not allowed: " + mode_str);
      }
      int pin_num = std::stoi(pin);
      if (pin_num > 255) {
        throw std::runtime_error("Pin number out of range: " + pin);
      }
      _pin_modes[static_cast<std::uint8_t>(pin_num)] = modes.at(mode_str);
    }
  }

  // Sends PIN_MODE to the device for every entry read_pin_modes() recorded.
  // Precondition: the device is open (open() has already succeeded).
  void apply_pin_modes() {
    for (const auto &[pin, mode] : _pin_modes) {
      _dev->pin_mode(pin, mode);
    }
  }

  std::shared_ptr<ArduinoDriver::Context> _ctx =
      std::make_shared<ArduinoDriver::Context>();
  std::unique_ptr<ArduinoDriver::Device> _dev;
  std::map<std::uint8_t, ArduinoDriver::PinMode> _pin_modes{};
  bool _init_error{false};
};
