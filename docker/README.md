# Docker Infrastructure for Air Quality Monitoring

VictoriaMetrics + Grafana stack that receives UDP data from the ESP32 BME680 sensor.

## Prerequisites

- [Docker Desktop](https://www.docker.com/products/docker-desktop/)

## Quick Start

```bash
cd docker
docker compose up -d
```

## Access

- **Grafana**: http://localhost:3000 (default login: `admin` / `admin`)
- **VictoriaMetrics UI**: http://localhost:8428/vmui/
- **UDP ingestion**: port `8089` (InfluxDB line protocol)

## Verify Data Ingestion

Send a test data point from the terminal:

```bash
echo 'bme680,location=test Temp=22.5,TempF=72.5,hPa=1013.25,RH=45.0,VOCOhms=80000,Altitude=100,Vraw=2048,Voltage=3.3' | nc -u -w1 localhost 8089
```

Then open http://localhost:8428/vmui/ and query `bme680_TempF` to confirm it arrived.

## ESP32 Firmware Configuration

Docker on macOS doesn't support UDP broadcast reception. You need to set `UDP_HOST` in the firmware to your Mac's IP address:

1. Find your Mac's IP: `ipconfig getifaddr en0`
2. Edit `ESP32-BME680-Air-Quality.ino` and update `UDP_HOST`:
   ```cpp
   #define UDP_HOST 192, 168, 1, 100  // your Mac's IP
   ```
3. Upload the firmware

> **Note:** The firmware sends InfluxDB line protocol over UDP. VictoriaMetrics, InfluxDB, and Telegraf all accept this format.

## Data Persistence

Named Docker volumes (`vmdata` and `grafana_data`) persist data across container restarts:

```bash
# Stop containers (data is preserved in volumes)
docker compose down

# Stop and delete all data
docker compose down -v
```

Back up the VictoriaMetrics data volume:

```bash
docker run --rm -v docker_vmdata:/data -v $(pwd):/backup alpine tar czf /backup/vmdata-backup.tar.gz -C /data .
```
