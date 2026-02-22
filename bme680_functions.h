#ifndef BME680_FUNCTIONS_H
#define BME680_FUNCTIONS_H

#include <cmath>
#include <cstdio>

#ifndef SEALEVELPRESSURE_HPA
#define SEALEVELPRESSURE_HPA (1013.25)
#endif

inline float celsiusToFahrenheit(float tempC) {
  return (tempC * (9.0f / 5.0f)) + 32.0f;
}

inline float adcToVoltage(int rawAdc) {
  return ((float(rawAdc) * 2.0f) / 4096.0f) * 3.3f;
}

inline float pressureToAltitude(float pressureHpa) {
  return 44330.0f * (1.0f - powf(pressureHpa / SEALEVELPRESSURE_HPA, 0.1903f));
}

inline float pressureToSeaLevel(float pressureHpa, float altitudeM) {
  return pressureHpa / powf(1.0f - (altitudeM / 44330.0f), 5.255f);
}

inline int buildUdpPayload(char* buf, size_t bufLen,
                            float tempC, float tempF, float pressure,
                            float humidity, float gas, float altitude,
                            float seaLevelHPa,
                            int voltageRaw, float voltage,
                            float iaq, int iaqAccuracy, float staticIaq,
                            float co2, float breathVoc,
                            float compTemp, float compRH) {
  return snprintf(buf, bufLen,
    "bme680,location=363Office "
    "Temp=%.2f,TempF=%.2f,hPa=%.2f,RH=%.2f,"
    "VOCOhms=%.2f,Altitude=%.2f,SeaLevelHPa=%.2f,"
    "Vraw=%d,Voltage=%.2f,"
    "IAQ=%.2f,IAQAccuracy=%d,StaticIAQ=%.2f,"
    "CO2=%.2f,BreathVOC=%.2f,CompTemp=%.2f,CompRH=%.2f",
    tempC, tempF, pressure, humidity,
    gas, altitude, seaLevelHPa,
    voltageRaw, voltage,
    iaq, iaqAccuracy, staticIaq,
    co2, breathVoc, compTemp, compRH);
}

#endif // BME680_FUNCTIONS_H
