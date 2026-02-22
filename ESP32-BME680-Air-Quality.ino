// core Library:
#include <Arduino.h>
#include "AsyncUDP.h" // from ESP32 library - comment out for 8266
#include <Preferences.h>

// 3rd party libraries:
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

// sensor libraries:
#include <Adafruit_BME680.h>
#include <Adafruit_Sensor.h>
#include <bme68x_defs.h>
#include <bme68x.h>
#include <bsec.h>

#include "bme680_functions.h"

// Adafruit Huzzah32 ESP32 Feather info:
// https://learn.adafruit.com/adafruit-huzzah32-esp32-feather

// Adafruit BME680 using I2C -- see Adafruit examples for other methods

/***************************************************************************
  Copyright 2020, Chris Tirpak
  This is a sketch for the BME680 gas, humidity, temperature & pressure sensor
  that will send the reading to an influx database for storage and analysis.

  It is based off of the work referenced below.

  BSD 2 clause license, all text above must be included in any redistribution

  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/3660

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

// todo add a way to calibrate for this - google around there was an example
#define SEALEVELPRESSURE_HPA (1013.25)

// UDP target for time-series database (e.g. VictoriaMetrics, InfluxDB).
// Sends InfluxDB line protocol over UDP.
// Default is broadcast (255.255.255.255) which works when a listener is on
// the same network. Docker on macOS cannot receive broadcast UDP, so set
// this to your Mac's IP address (e.g. 192, 168, 1, 100).
#define UDP_HOST 255, 255, 255, 255
#define UDP_PORT 8089

// Temperature offset to compensate for ESP32 self-heating (degrees C).
// The BME680 mounted near the ESP32 reads ~2C higher than ambient.
#define BSEC_TEMP_OFFSET 2.0f

// How often to save BSEC calibration state to NVS (milliseconds).
// Default: every 6 hours. Frequent saves are unnecessary since NVS
// has limited write endurance (~100K cycles).
#define BSEC_SAVE_INTERVAL_MS (6UL * 60 * 60 * 1000)

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
void loadBsecState(void);
void saveBsecState(void);

// Create an object of the class Bsec
Bsec iaqSensor;

String output;

AsyncUDP udp;

Preferences preferences;
unsigned long lastBsecSave = 0;

void setup()
{

  Serial.begin(115200);
  while (!Serial)
    delay(2000); // Wait for serial to be ready
  // delay for humans to catch up over on the serial port
  delay(5000);
  Serial.println();
  Serial.println(F("serial init complete"));
  Serial.println(F("bme680station debug"));
  Serial.println(F("bme680station version 0.2.0"));
  Serial.flush();

  Wire.begin();

  if (iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire) != BSEC_OK) {
    Serial.println(F("Failed to initialize BME680 sensor"));
    checkIaqSensorStatus();
    while (1) {
      errLeds();
    }
  }
  output = "\n\rBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();

  // Apply temperature offset to compensate for ESP32 self-heating
  iaqSensor.setTemperatureOffset(BSEC_TEMP_OFFSET);
  Serial.print(F("Temperature offset applied: "));
  Serial.print(BSEC_TEMP_OFFSET);
  Serial.println(F(" C"));

  // Restore BSEC calibration state from NVS (if previously saved)
  loadBsecState();

  // countdown a brief delay again for the humans on the serial port
  for (int waitSeconds = 5; waitSeconds > 0; waitSeconds--)
  {
    output = "[SETUP] WAIT " + String(waitSeconds) + "...";
    Serial.println(output);
    Serial.flush();
    delay(1000);
  }

  bsec_virtual_sensor_t sensorList[10] = {
      BSEC_OUTPUT_RAW_TEMPERATURE,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_HUMIDITY,
      BSEC_OUTPUT_RAW_GAS,
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_STATIC_IAQ,
      BSEC_OUTPUT_CO2_EQUIVALENT,
      BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();
  // initialize the sensor

  pinMode(LED_BUILTIN, OUTPUT);

  WiFiManager wifiManager;
  // Uncomment and run it once, if you want to erase all the stored information
  // wifiManager.resetSettings();

  // fetches ssid and pass from eeprom and tries to connect
  // if it does not connect it starts an access point with the specified name
  // and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("AutoConnectAP"))
  {
    Serial.println("failed to connect and hit timeout");
    ESP.restart();
    delay(1000);
  }

  // if you get here you have connected to the WiFi
  Serial.println("WifFi connected.");

  // print the wifi info to serial
  printWifiStatusToSerial();
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH); // led on while we process the BME680
  unsigned long time_trigger = millis();

  float tempC = 0.00;
  float tempF = 0.00;
  float pressure = 0.00;
  float humidity = 0.00;
  float gas = 0.00;
  float iaq = 0.00;
  int iaqAccuracyVal = 0;
  float compTemp = 0.00;
  float compRH = 0.00;
  float iaqStatic = 0.00;
  float iaqCO2 = 0.00;
  float breath = 0.00;

  if (iaqSensor.run())
  {
    tempC = (iaqSensor.rawTemperature);
    tempF = celsiusToFahrenheit(tempC);
    pressure = (iaqSensor.pressure);
    humidity = (iaqSensor.rawHumidity);
    gas = (iaqSensor.gasResistance);
    iaq = (iaqSensor.iaq);
    iaqAccuracyVal = (iaqSensor.iaqAccuracy);
    compTemp = (iaqSensor.temperature);
    compRH = (iaqSensor.humidity);
    iaqStatic = (iaqSensor.staticIaq);
    iaqCO2 = (iaqSensor.co2Equivalent);
    breath = (iaqSensor.breathVocEquivalent);
  }
  else
  {
    checkIaqSensorStatus();
  }

  // fix this to adjust for actual altitude
  // float altitude = (bme.readAltitude(SEALEVELPRESSURE_HPA));
  float altitude = pressureToAltitude(pressure);

  // get battery voltage
  int voltageRaw = analogRead(35);
  float voltage = adcToVoltage(voltageRaw);

  String serialOutput =
      String("Elapsed ms = ") +
      String(time_trigger) + "," +
      String("Temperature in C = ") +
      String(tempC) + "," +
      String("Temperature in F = ") +
      String(tempF) + "," +
      String("Pressure = ") +
      String(pressure) + "," +
      String("Humidity = ") +
      String(humidity) + "," +
      String("Gas = ") +
      String(gas) + "," +
      String("IAQ = ") +
      String(iaq) + "," +
      String("IAQ Accuracy = ") +
      String(iaqAccuracyVal) + "," +
      String("Comp Temp C = ") +
      String(compTemp) + "," +
      String("Comp Humidity = ") +
      String(compRH) + "," +
      String("IAQ Static = ") +
      String(iaqStatic) + "," +
      String("IAQ CO2 = ") +
      String(iaqCO2) + "," +
      String("Breath VOC = ") +
      String(breath) + "," +
      String("Altitude = ") +
      String(altitude) + "," +
      String("Voltage RAW = ") +
      String(voltageRaw) + "," +
      String("Voltage = ") +
      String(voltage);

  Serial.println(serialOutput);

  // Send InfluxDB line protocol over UDP to time-series database
  if (udp.connect(IPAddress(UDP_HOST), UDP_PORT))
  {
    delay(50); // Needed cause sometimes no Delay = can't send UDP packages fast enough
    String udpPayload = buildUdpPayload(tempC, tempF, pressure, humidity, gas,
                                         altitude, voltageRaw, voltage,
                                         iaq, iaqAccuracyVal, iaqStatic,
                                         iaqCO2, breath, compTemp, compRH);

    udp.print(String(udpPayload));
    delay(50);
  }

  // Periodically save BSEC calibration state to NVS
  if (millis() - lastBsecSave >= BSEC_SAVE_INTERVAL_MS) {
    saveBsecState();
    lastBsecSave = millis();
  }

  digitalWrite(LED_BUILTIN, LOW); // turn the LED off
  delay(5 * (60 * 1000));         // minutes
  // delay(10 * 1000); // seconds
}

void printWifiStatusToSerial()
{
  byte mac[6];
  WiFi.macAddress(mac);
  output = "Wifi connection info:\n\r";
  output += "MAC: ";
  output += String(mac[5], HEX) + ":";
  output += String(mac[4], HEX) + ":";
  output += String(mac[3], HEX) + ":";
  output += String(mac[2], HEX) + ":";
  output += String(mac[1], HEX) + ":";
  output += String(mac[0], HEX) + "\n\r";
  output += "Wifi Status:\n\r";
  output += "Status: " + String(WiFi.status()) + "\n\r";
  output += "SSID: " + WiFi.SSID() + "\n\r";
  output += "BSSID: " + WiFi.BSSIDstr() + "\n\r";
  output += "Channel: " + String(WiFi.channel()) + "\n\r";
  output += "IP address: " + WiFi.localIP().toString() + "\n\r";
  output += "Subnet mask: " + WiFi.subnetMask().toString() + "\n\r";
  output += "Gateway IP: " + WiFi.gatewayIP().toString() + "\n\r";
  output += "DNS: " + WiFi.dnsIP().toString() + "\n\r";
  output += "Signal strength (RSSI): " + String(WiFi.RSSI()) + "\n\r";
  Serial.print(output);
}

void loadBsecState(void)
{
  preferences.begin("bsec", true); // read-only
  size_t stateLen = preferences.getBytesLength("state");
  if (stateLen == BSEC_MAX_STATE_BLOB_SIZE) {
    uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE];
    preferences.getBytes("state", bsecState, BSEC_MAX_STATE_BLOB_SIZE);
    iaqSensor.setState(bsecState);
    Serial.println(F("BSEC state restored from NVS"));
  } else {
    Serial.println(F("No saved BSEC state found — starting fresh calibration"));
  }
  preferences.end();
}

void saveBsecState(void)
{
  uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE];
  iaqSensor.getState(bsecState);
  preferences.begin("bsec", false); // read-write
  preferences.putBytes("state", bsecState, BSEC_MAX_STATE_BLOB_SIZE);
  preferences.end();
  Serial.println(F("BSEC state saved to NVS"));
}

// Helper function definitions
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK)
  {
    if (iaqSensor.status < BSEC_OK)
    {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    }
    else
    {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK)
  {
    if (iaqSensor.bme680Status < BME680_OK)
    {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    }
    else
    {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}

void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}
