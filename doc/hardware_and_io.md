# Hardware and I/O Specification

## Target Hardware

- Board: ESP32 CYD
- Display: 240x320 TFT
- Touch controller: XPT2046
- Temperature sensor: DHT22
- Fan output: PWM on LEDC
- Backlight control: PWM on LEDC
- UI stack: LVGL + TFT_eSPI
- Web dashboard: FrameWeb + WebSocket

## Display and Touch

| Function | Component | Pin / Interface | Notes |
| --- | --- | --- | --- |
| TFT display | TFT_eSPI | board-defined SPI/display wiring | LVGL display is rotated to landscape |
| Touch IRQ | XPT2046 IRQ | GPIO36 | T_IRQ |
| Touch MOSI | XPT2046 MOSI | GPIO32 | T_DIN |
| Touch MISO | XPT2046 MISO | GPIO39 | T_OUT |
| Touch CLK | XPT2046 CLK | GPIO25 | T_CLK |
| Touch CS | XPT2046 CS | GPIO33 | T_CS |

## Control and Sensor I/O

| Function | Pin | Mode | Notes |
| --- | --- | --- | --- |
| DHT22 data | GPIO27 | Digital input | Temperature and humidity sensor |
| Fan PWM output | GPIO22 | LEDC PWM | Cooling/ventilation output |
| Backlight PWM | GPIO21 | LEDC PWM | Screen brightness control |
| RGB LED red | GPIO4 | Digital output | Active-low board LED wiring |
| RGB LED green | GPIO16 | Digital output | Active-low board LED wiring |
| RGB LED blue | GPIO17 | Digital output | Active-low board LED wiring |

## PWM Configuration

| Item | Value |
| --- | --- |
| Fan PWM channel | 0 |
| Backlight PWM channel | 1 |
| PWM frequency | 20 kHz |
| PWM resolution | 8 bits |
| Fan output range | 0% to 100% |
| Backlight active brightness | 100% |
| Backlight screensaver brightness | 20% |

## Firmware Behavior

- The touch controller is scanned through the XPT2046 SPI bus.
- The DHT22 is sampled periodically and used as the PID process variable.
- The fan output is driven by LEDC PWM on GPIO22.
- The backlight is dimmed automatically by the screensaver logic.
- The local graph is rendered in the browser and receives single temperature/PWM samples over WebSocket.

## Notes

- The physical display is portrait-oriented, but the GUI is configured for landscape coordinates.
- The chart history is reconstructed on the browser side, so the WebSocket payload stays small.
- Pin assignments are taken from `src/main.cpp` and the current GUI wiring in `src/gui.cpp`.
