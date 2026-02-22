# ESP32 BME680 Air Quality Monitor

ESP32-based household air quality monitor using a BME680 sensor with the Bosch BSEC library. Reads temperature, pressure, humidity, gas resistance, IAQ score, CO2 equivalent, and breath VOC. Sends data via UDP in InfluxDB line protocol format for collection by VictoriaMetrics, InfluxDB, or Telegraf.

## Hardware Required

- [Adafruit HUZZAH32 ESP32 Feather](https://www.adafruit.com/product/3405)
- [Adafruit BME680 Breakout](https://www.adafruit.com/product/3660)

### Wiring (I2C)

| BME680 | HUZZAH32 |
|--------|----------|
| VIN    | 3V       |
| GND    | GND      |
| SCK    | SCL      |
| SDI    | SDA      |

The BME680 breakout uses I2C address `0x77` by default.

## Software Prerequisites

```bash
# macOS
brew install arduino-cli cppcheck

# Configure arduino-cli
arduino-cli config init
arduino-cli config add board_manager.additional_urls \
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32

# Install libraries
arduino-cli lib install "BSEC Software Library"
arduino-cli lib install "Adafruit BME680 Library"
arduino-cli lib install "Adafruit Unified Sensor"
arduino-cli lib install "Adafruit BusIO"
arduino-cli lib install "WiFiManager"
```

## Build & Upload

```bash
./build.sh compile    # compile only
./build.sh upload     # compile + upload
./build.sh monitor    # serial monitor (115200 baud)
./build.sh test       # run unit tests (host, no hardware needed)
./build.sh lint       # cppcheck static analysis
./build.sh all        # compile, upload, and monitor
```

Override the serial port: `PORT=/dev/tty.xxx ./build.sh upload`

## First-Time WiFi Setup

1. Power on the device
2. Connect to the `AirQuality-Setup` WiFi network from your phone or laptop
3. A captive portal opens automatically. Configure:
   - **WiFi network** and password
   - **UDP Host IP** (default: `255.255.255.255` broadcast)
   - **UDP Port** (default: `8089`)
   - **Read Interval** in seconds (default: `300` = 5 minutes; minimum: `10`)
4. Save — the device connects and starts sending data

All settings are stored in NVS and persist across reboots.

## Reconfiguring

Hold the **BOOT button** (GPIO0) on the HUZZAH32 Feather at any time to re-enter the setup portal. This lets you change WiFi, UDP target, and read interval without reflashing.

The device is also reachable at `airquality.local` via mDNS.

## Data Collection

A Docker-based VictoriaMetrics + Grafana stack is included for collecting and visualizing sensor data:

```bash
cd docker && docker compose up -d
```

- **Grafana**: http://localhost:3000 (admin/admin)
- **VictoriaMetrics**: http://localhost:8428/vmui/

See [docker/README.md](docker/README.md) for full setup instructions.

> **Note:** Docker on macOS cannot receive broadcast UDP. Set the UDP Host IP to your Mac's IP address (`ipconfig getifaddr en0`) via the config portal.

## LED Status

| LED State    | Meaning              |
|-------------|----------------------|
| ON (solid)  | WiFi connected       |
| OFF         | WiFi disconnected    |
| Rapid blink | Sensor error (halted)|

## Sensor Notes

### 48-Hour Burn-In

The BME680 requires approximately 48 hours of continuous operation for the BSEC algorithm to fully calibrate. During this period, IAQ readings may be inaccurate. The `IAQAccuracy` field indicates calibration status:

| IAQAccuracy | Meaning                          |
|-------------|----------------------------------|
| 0           | Stabilizing / unreliable         |
| 1           | Low accuracy                     |
| 2           | Medium accuracy                  |
| 3           | High accuracy (fully calibrated) |

BSEC calibration state is automatically saved to NVS every 6 hours and restored on reboot, so you don't lose calibration progress after a power cycle.

### Temperature Offset

A 2.0 C offset (`BSEC_TEMP_OFFSET`) is applied to compensate for self-heating from the ESP32 mounted near the sensor. Adjust this value in the source if your mounting arrangement differs.

### IAQ Score Ranges

| IAQ Range | Air Quality  |
|-----------|-------------|
| 0-50      | Excellent    |
| 51-100    | Good         |
| 101-150   | Moderate     |
| 151-200   | Unhealthy    |
| 201-300   | Poor         |
| 301-500   | Hazardous    |

### UDP Payload Fields

The device sends InfluxDB line protocol with these fields:

`Temp`, `TempF`, `hPa`, `RH`, `VOCOhms`, `Altitude`, `Vraw`, `Voltage`, `IAQ`, `IAQAccuracy`, `StaticIAQ`, `CO2`, `BreathVOC`, `CompTemp`, `CompRH`

## Reference Documentation

- [BME680 Datasheet (Bosch)](docs/BME680-datasheet.pdf)
- [Adafruit BME680 Guide](docs/adafruit-bme680-guide.pdf)

## License

BSD 2-Clause. Based on work by Limor Fried & Kevin Townsend for [Adafruit Industries](https://www.adafruit.com/). Designed to work with the [Adafruit BME680 Breakout](https://www.adafruit.com/product/3660). Please support Adafruit and open-source hardware by purchasing products from Adafruit.
