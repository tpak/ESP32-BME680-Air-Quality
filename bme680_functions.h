#ifndef BME680_FUNCTIONS_H
#define BME680_FUNCTIONS_H

#ifdef ARDUINO
#include <Arduino.h>
#else
#include "test/arduino_string_compat.h"
#endif

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

inline String buildUdpPayload(float tempC, float tempF, float pressure,
                               float humidity, float gas, float altitude,
                               int voltageRaw, float voltage,
                               float iaq, int iaqAccuracy, float staticIaq,
                               float co2, float breathVoc,
                               float compTemp, float compRH) {
  return String("bme680,location=363Office Temp=") + String(tempC) +
         ",TempF=" + String(tempF) + ",hPa=" + String(pressure) +
         ",RH=" + String(humidity) + ",VOCOhms=" + String(gas) +
         ",Altitude=" + String(altitude) + ",Vraw=" + String(voltageRaw) +
         ",Voltage=" + String(voltage) +
         ",IAQ=" + String(iaq) + ",IAQAccuracy=" + String(iaqAccuracy) +
         ",StaticIAQ=" + String(staticIaq) + ",CO2=" + String(co2) +
         ",BreathVOC=" + String(breathVoc) +
         ",CompTemp=" + String(compTemp) + ",CompRH=" + String(compRH);
}

#endif // BME680_FUNCTIONS_H
