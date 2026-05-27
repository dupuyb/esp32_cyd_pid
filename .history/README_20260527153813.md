# Esp32_CYD_Pid

ESP32 CYD project with:
- LVGL touch GUI
- DHT22 temperature/humidity reading
- PID control loop
- PWM output for VMC/fan driving
- FrameWeb integration (WiFi, config page, websocket stack)

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

## Project Structure

- src/main.cpp: hardware init, LVGL loop, DHT sampling, PID compute, PWM drive, FrameWeb lifecycle
- src/gui.h: UI widgets, callbacks, labels updates, PID tuning controls
- include/User_Setup.h: TFT_eSPI display configuration
- include/lv_conf.h: LVGL configuration
- platformio.ini: environment, dependencies, ports, build flags, pre-build scripts

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
- VMC active output pin: GPIO26
- Optional RGB LED pins (board dependent): GPIO4 / GPIO16 / GPIO17

## Build and Upload

From this folder:

```bash
pio run -e esp32dev
pio run -e esp32dev -t upload
pio device monitor -b 115200
```

Optional cleanup:

```bash
pio run -e esp32dev -t clean
```

## PlatformIO Notes

Current environment is configured in platformio.ini:
- board: esp32dev
- framework: arduino
- partition table: min_spiffs.csv
- monitor speed: 115200
- extra script: Esp32_Framework extra_script.py (HTML/Web asset pipeline)

Dependencies include LVGL, TFT_eSPI, DHTesp, XPT2046_Touchscreen, WiFiManager, WebSockets, and Esp32_Framework.

## Runtime Behavior

1. Boot initializes LVGL, touchscreen, DHT22, PWM and FrameWeb.
2. GUI is created with 4 panels:
   - Temperature
   - PID
   - VMC
   - Access/Network
3. DHT22 is sampled periodically (guarded timing).
4. PID computes output percentage from setpoint vs measured temperature.
5. PWM duty is updated and mirrored in the UI.
6. Time and network labels are refreshed in loop.

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
- Touch coordinates feel mirrored:
  - review touchscreen rotation/calibration in src/main.cpp
- No DHT values:
  - verify DHT22 wiring and GPIO27
- PWM not driving actuator:
  - verify GPIO22 and active line GPIO26 behavior on your board

## Developer Notes

- A previous formatting attempt with macOS indent can break C++ source formatting.
- Prefer clang-format or astyle for full-file C/C++ formatting operations.
