#ifndef BME680_FUNCTIONS_H
#define BME680_FUNCTIONS_H

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
  return pressureHpa / SEALEVELPRESSURE_HPA;
}

inline int buildUdpPayload(char* buf, size_t bufLen,
                            float tempC, float tempF, float pressure,
                            float humidity, float gas, float altitude,
                            int voltageRaw, float voltage,
                            float iaq, int iaqAccuracy, float staticIaq,
                            float co2, float breathVoc,
                            float compTemp, float compRH) {
  return snprintf(buf, bufLen,
    "bme680,location=363Office "
    "Temp=%.2f,TempF=%.2f,hPa=%.2f,RH=%.2f,"
    "VOCOhms=%.2f,Altitude=%.2f,Vraw=%d,Voltage=%.2f,"
    "IAQ=%.2f,IAQAccuracy=%d,StaticIAQ=%.2f,"
    "CO2=%.2f,BreathVOC=%.2f,CompTemp=%.2f,CompRH=%.2f",
    tempC, tempF, pressure, humidity,
    gas, altitude, voltageRaw, voltage,
    iaq, iaqAccuracy, staticIaq,
    co2, breathVoc, compTemp, compRH);
}

#endif // BME680_FUNCTIONS_H
