# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-based air quality monitor using a BME680 sensor with the Bosch BSEC library. Reads temperature, pressure, humidity, gas resistance, IAQ score, CO2 equivalent, and breath VOC. Sends data via UDP broadcast (port 8089) in InfluxDB line protocol format. WiFi is configured automatically via WiFiManager (captive portal).

## Build & Upload

This is an **Arduino IDE project** (single `.ino` sketch). Two build methods are available:

### CLI via `build.sh` (recommended)

Requires `arduino-cli` and `cppcheck` (`brew install arduino-cli cppcheck`). See `build.sh` header for full setup instructions.

- `./build.sh compile` — compile the sketch
- `./build.sh upload` — compile and upload to the board
- `./build.sh monitor` — open serial monitor at 115200 baud
- `./build.sh lint` — run cppcheck static analysis
- `./build.sh all` — compile, upload, and monitor

Override the serial port with `PORT=/dev/tty.xxx ./build.sh upload`.

### VSCode via Arduino Community Edition extension

Install `vscode-arduino.vscode-arduino-community` (community fork of the deprecated Microsoft extension). It reads the existing `.vscode/arduino.json`. Use Ctrl+Shift+P → "Arduino: Verify" or "Arduino: Upload".

### Config

- **Board**: Adafruit HUZZAH32 Feather (`esp32:esp32:featheresp32`)
- **Serial port**: `/dev/tty.usbserial-0160E5AB` (configured in `.vscode/arduino.json`, overridable via `PORT` env var in CLI)
- **Upload speed**: 921600 baud
- **Build output**: `build/` directory

### Linting

Run `./build.sh lint` to run cppcheck. Suppression rules for Arduino-specific false positives are in `cppcheck-suppressions.txt`.

## Architecture

The entire application is in `ESP32-BME680-Air-Quality.ino` — a single-file Arduino sketch:

- **`setup()`**: Initializes Serial (115200), I2C (`Wire`), BME680 via BSEC library (I2C address `0x77`), subscribes to 10 BSEC virtual sensors at low-power sample rate, then connects WiFi via WiFiManager
- **`loop()`**: Every 5 minutes — reads all BSEC sensor outputs, converts temp C→F, reads battery voltage from GPIO35 ADC, prints readings to Serial, sends to InfluxDB via UDP broadcast to `255.255.255.255:8089`
- **`checkIaqSensorStatus()`**: Checks both BSEC and BME680 status codes; halts with LED blink on fatal errors, prints warnings otherwise
- **`errLeds()`**: Rapid LED blink to indicate error state

## Key Dependencies

Libraries must be installed in `~/Documents/Arduino/libraries/`:

- **BSEC_Software_Library** — Bosch closed-source air quality library (provides IAQ, CO2, VOC calculations from raw BME680 data)
- **Adafruit_BME680_Library** + **Adafruit_Unified_Sensor** + **Adafruit_BusIO** — sensor drivers and I2C abstraction
- **WiFiManager** (`tzapu/WiFiManager`) — auto WiFi configuration via captive portal
- **AsyncUDP** — included with ESP32 Arduino core, not a separate install

## Hardware Notes

- BME680 uses I2C secondary address (`BME680_I2C_ADDR_SECONDARY` = `0x77`); Adafruit breakout boards use this by default
- Battery voltage: read from GPIO35 analog, formula: `(raw * 2.0 / 4096.0) * 3.3`
- `SEALEVELPRESSURE_HPA` is hardcoded to `1013.25` — altitude calculation needs calibration
- The InfluxDB measurement name and location tag are hardcoded in the UDP send (`bme680,location=363Office`)
- UDP target IP/port are configurable via `INFLUX_HOST` and `INFLUX_PORT` defines (default: broadcast `255.255.255.255:8089`)

## Docker Infrastructure

A Docker-based data collection and visualization stack lives in `docker/`:

- **VictoriaMetrics** — time-series database that receives InfluxDB line protocol via UDP on port 8089
- **Grafana** — dashboards for visualizing sensor data, auto-provisioned with a VictoriaMetrics datasource and Air Quality dashboard

See `docker/README.md` for setup and usage. Key commands:

- `cd docker && docker compose up -d` — start the stack
- Grafana: `http://localhost:3000` (admin/admin)
- VictoriaMetrics UI: `http://localhost:8428/vmui/`

Docker on macOS requires setting `INFLUX_HOST` in the firmware to the Mac's IP (broadcast UDP doesn't reach Docker containers).
