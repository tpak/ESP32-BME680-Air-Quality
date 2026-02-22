- re configure the influx server
- [x] set it up locally on the mac in docker (see `docker/` directory — VictoriaMetrics + Grafana stack)

[] follow up on sea level pressure compensation
  // fix this to adjust for actual altitude
  // float altitude = (bme.readAltitude(SEALEVELPRESSURE_HPA));
  float altitude = (pressure / 1013.25);
