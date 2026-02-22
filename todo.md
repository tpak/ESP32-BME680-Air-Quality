- re configure the influx server
- [x] set it up locally on the mac in docker (see `docker/` directory — VictoriaMetrics + Grafana stack)

[x] follow up on sea level pressure compensation
  - Fixed pressureToAltitude() to use hypsometric formula (returns meters)
  - Added pressureToSeaLevel() inverse formula
  - Altitude configurable via WiFiManager captive portal (stored in NVS)
  - SeaLevelHPa field added to UDP payload
  - Fixed Pa→hPa conversion (BSEC returns Pa, not hPa)
