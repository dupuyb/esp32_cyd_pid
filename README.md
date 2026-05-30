# Esp32_CYD_Pid

ESP32 CYD project with:
- LVGL touch GUI
- DHT22 temperature/humidity reading
- PID control loop
- PWM output for VMC/fan driving
- Backlight dimming on screen saver (GPIO21)
- Esp32_Framework integration (WiFi manager, web tools, websocket stack, embedded HTML pipeline)

## Relation with Esp32_Framework

This project relies on Esp32_Framework as its web and configuration backbone.

Main integration points:
- Library dependency from GitHub in platformio.ini
- Build-time script from framework:
  - .pio/libdeps/esp32dev/Esp32_Framework/extra_script.py
- Generated web assets configured through custom keys:
  - custom_in_html
  - custom_out_h
  - custom_out_zip
- Runtime framework object in firmware:
  - FrameWeb frame;
- Framework lifecycle:
  - frame.setup() in setup()
  - frame.loop() in loop()

In short, Esp32_CYD_Pid provides the control logic (sensor + PID + UI), while Esp32_Framework provides the web runtime and configuration tooling.

## Features

- Real-time display of:
  - temperature (C)
  - humidity (%)
  - setpoint (C)
  - PWM output (%)
  - PID state (ON/OFF)
  - network info (IP/MAC/time)
- Touch slider for setpoint (25.0 C to 45.0 C)
- On-screen tuning controls for Kp / Ki / Kd
- Manual VMC switch behavior when PID is disabled
- Screen saver page with automatic backlight dimming (20%) and restore on touch (100%)
- Screen saver is blocked during startup while MAC label is still visible (initialization guard)
- Dynamic externalHtmlTools content showing:
  - temperatures
  - V.M.C state
  - PWM speed in %

## Project Structure

- src/main.cpp: hardware init, LVGL loop, DHT sampling, PID compute, PWM drive, Esp32_Framework lifecycle
- src/gui.h: UI widgets, callbacks, labels updates, PID tuning controls
- include/User_Setup.h: TFT_eSPI display configuration
- include/lv_conf.h: LVGL configuration
- platformio.ini: environment, dependencies, ports, build flags, framework script wiring
- prebuild.py: optional local helper script for LVGL/TFT config copy (kept for fallback usage)
- doc/: hardware and merged schema assets

## Hardware Mapping (current firmware)

- Display resolution: 240x320 (LVGL rotated to landscape)
- Touch (XPT2046):
  - IRQ: GPIO36
  - MOSI: GPIO32
  - MISO: GPIO39
  - CLK: GPIO25
  - CS: GPIO33
- DHT22 data pin: GPIO27
- PWM output pin: GPIO22
- TFT backlight pin: GPIO21 (LEDC PWM)
- VMC active output pin: GPIO26
- Optional RGB LED pins (board dependent): GPIO4 / GPIO16 / GPIO17

## Build and Upload

From this folder:

```bash
pio run -e esp32dev
pio run -e esp32dev -t upload
pio device monitor -b 115200
```

If `pio` is not available in your shell, use VS Code PlatformIO tasks:
- PlatformIO: Build (Esp32_CYD_Pid)
- PlatformIO: Upload (Esp32_CYD_Pid)

Optional cleanup:

```bash
pio run -e esp32dev -t clean
```

## Esp32_Framework Setup Notes

This project expects Esp32_Framework to be fetched automatically by PlatformIO via lib_deps.

Recommended first build sequence:
1. Run pio run -e esp32dev once to install all dependencies.
2. Verify that framework files are present under .pio/libdeps/esp32dev/Esp32_Framework.
3. Build again after any framework update.

If you update Esp32_Framework behavior and see web mismatches, run:

```bash
pio run -e esp32dev -t clean
pio run -e esp32dev
```

## PlatformIO Notes

Current environment is configured in platformio.ini:
- board: esp32dev
- framework: arduino
- partition table: min_spiffs.csv
- monitor speed: 115200
- extra script: Esp32_Framework extra_script.py
- custom_in_html: framework HTML source used by script pipeline
- custom_out_h: generated C++ output consumed at build/runtime

Dependencies include LVGL, TFT_eSPI, DHTesp, XPT2046_Touchscreen, WiFiManager, WebSockets, ArduinoJson, and Esp32_Framework.

## Runtime Behavior

1. Boot initializes LVGL, touchscreen, DHT22, PWM and Esp32_Framework.
2. GUI is created with 4 panels:
   - Temperature
   - PID
   - VMC
   - Access/Network
3. DHT22 is sampled periodically (guarded timing).
4. PID computes output percentage from setpoint vs measured temperature.
5. PWM duty is updated and mirrored in the UI.
6. externalHtmlTools content is refreshed with live process values.
7. Time and network labels are refreshed in loop.
8. While the MAC label is visible (startup/incomplete init), screen saver activation is intentionally blocked.
9. After initialization is complete, screen saver is activated after inactivity timeout and dims backlight to 20%.
10. Any touch restores dashboard page and backlight to 100%.

## Framework Callbacks

The following callbacks are present and reserved for framework events:
- saveConfigCallback()
- webSocketEvent(...)
- configModeCallback(...)

They are currently minimal and can be extended for project-specific web behavior.

## PID Controls

- Kp, Ki, Kd are editable from GUI buttons.
- PID ON:
  - output is continuously computed from control error
- PID OFF:
  - manual VMC switch forces output to 0% or 100%

## Troubleshooting

- Upload fails:
  - verify upload_port in platformio.ini
  - check USB cable and serial adapter
- Serial monitor unreadable:
  - ensure monitor speed is 115200
- Framework pages not showing expected tools data:
  - verify frame.loop() is called continuously in loop()
  - verify externalHtmlTools assignment happens after control values are updated
  - clean and rebuild to regenerate framework web assets
- Backlight does not dim in screen saver:
  - verify TFT backlight is wired to GPIO21 on your CYD variant
  - verify no other library code overrides GPIO21 duty-cycle after setup
- Screen saver never starts:
  - check whether MAC label remains visible (network/time init not finalized)
  - verify update_access_network_labels() hides MAC once IP and time are valid
- Touch coordinates feel mirrored:
  - review touchscreen rotation/calibration in src/main.cpp
- No DHT values:
  - verify DHT22 wiring and GPIO27
- PWM not driving actuator:
  - verify GPIO22 and active line GPIO26 behavior on your board

## Developer Notes

- A previous formatting attempt with macOS indent can break C++ source formatting.
- Prefer clang-format or astyle for full-file C/C++ formatting operations.
- Keep local/editor/build folders out of Git:
  - .pio/
  - .vscode/
  - .history/
