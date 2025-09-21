# Copilot Instructions for AI Agents

## Project Overview
This project is an embedded firmware for an ESP32-based system, structured for PlatformIO. It controls hardware components (CAN, I2C, SPI, PWM, WiFi, Webserver) and provides a web interface for configuration and data display.

## Key Architecture & Components
- **src/**: Main application logic (`main.cpp`, `option1.cpp`). Entry point is `main.cpp`.
- **lib/option1/**: Hardware abstraction and drivers (e.g., LCD, Touch, CAN, GPIO Expander).
- **Webserver**: Uses ESP32's WiFi in Access Point mode. Serves HTML pages and handles configuration via HTTP POST.
- **Persistent Storage**: Uses `Preferences` (NVS) for settings like WiFi credentials and feature toggles.
- **data/**: Static files (e.g., images) for the webserver, uploaded to LittleFS/SPIFFS via PlatformIO.

## Developer Workflows
- **Build & Upload**: Use PlatformIO (`pio run`, `pio upload`).
- **Filesystem Upload**: `pio run --target uploadfs` to upload `data/` to ESP32 flash.
- **Serial Debugging**: Monitor with `pio device monitor` (default 115200 baud).
- **Webserver Testing**: Connect to ESP32 AP (default SSID: `SMS REVO SL`), access via `http://192.168.4.1/`.

## Project-Specific Patterns
- **Settings Pages**: Web forms POST to `/einstellungen`, values are validated (e.g., WiFi password ≥8 chars) and stored in NVS. Changes require ESP32 reboot to take effect.
- **Interrupts**: ISRs must not use I2C/SPI/Serial; set flags and handle logic in `loop()`.
- **PWM**: Use `analogWrite` for backlight; for advanced control, use `ledcWrite` after proper setup.
- **I2C Register Access**: Always set register address with `Wire.beginTransmission`/`Wire.write`/`Wire.endTransmission(false)`, then read with `Wire.requestFrom`/`Wire.read`.
- **Webserver Handlers**: Register all endpoints explicitly; use `onNotFound` for 404 handling.

## Integration & External Dependencies
- **PlatformIO**: All builds, uploads, and monitor tasks are via PlatformIO.
- **ESP32 Arduino Core**: Uses standard libraries for WiFi, WebServer, Preferences, etc.
- **LittleFS/SPIFFS**: For serving static web content.

## Examples
- To add a new web page: Register a handler in `init_wifi()` and serve HTML via `kart_server.send()`.
- To persist a setting: Use `ESP_storage.putString("KEY", value)` and retrieve with `getString`.
- To add a hardware feature: Implement in `lib/option1/`, expose via `option1.cpp`, and call from `main.cpp`.

## Conventions
- Use `String` for web form values and NVS storage.
- All hardware pin mappings and constants are in `option1.hpp`.
- Keep ISRs minimal—no blocking or hardware access.

---

For questions about unclear workflows or missing patterns, please request clarification from the user.
