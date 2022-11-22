
#include <Arduino.h>
#include <Adafruit_BME680.h>
#include <Adafruit_Sensor.h>
#include <bme68x_defs.h>
#include <bme68x.h>
#include <WiFi.h> // from ESP32 library - comment out for 8266
#include <WiFiMulti.h> // from ESP32 library - comment out for 8266
//#include <HTTPClient.h>
#include "AsyncUDP.h"  // from ESP32 library - comment out for 8266

// begin undo this block for feather 8266 based board
// #include <BearSSLHelpers.h>
// #include <CertStoreBearSSL.h>
// #include <ESP8266WiFi.h>
// #include <ESP8266WiFiAP.h>
// #include <ESP8266WiFiGeneric.h>
// #include <ESP8266WiFiMulti.h>
// #include <ESP8266WiFiScan.h>
// #include <ESP8266WiFiSTA.h>
// #include <ESP8266WiFiType.h>
// #include <WiFiClient.h>
// #include <WiFiClientSecure.h>
// #include <WiFiClientSecureAxTLS.h>
// #include <WiFiClientSecureBearSSL.h>
// #include <WiFiServer.h>
// #include <WiFiServerSecure.h>
// #include <WiFiServerSecureAxTLS.h>
// #include <WiFiServerSecureBearSSL.h>
// #include <WiFiUdp.h>
// end this block for feather 8266 based board

//#include <SPI.h>
//#include <Wire.h>



// undo this block for feather 8266 based board
// #include <WiFiUdp.h>
// end this block for feather 8266 based board

// esp32 and esp8266 network info 
// https://diyprojects.io/esp32-how-to-connect-local-wifi-network-arduino-code/
// or
// https://diyprojects.io/esp8266-web-client-tcp-ip-communication-examples-esp8266wifi-esp866httpclient/

// https://learn.adafruit.com/adafruit-huzzah32-esp32-feather


/***************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor
  that will send the reading to an influx database for

  It is based off of the work referenced below.
  Written by Chris Tirpak 
  BSD license, all text above must be included in any redistribution
   
  This is a library for the BME680 gas, humidity, temperature & pressure sensor

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


//todo add a way to calibrate for this - google around there was an example
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C -- see Adafruit examples for other methods

WiFiMulti wifiMulti;
// ESP8266WiFiMulti wifiMulti;
AsyncUDP udp;
// WiFiUDP udp;

void setup() {
  
  Serial.begin(115200);
  while (!Serial);
  // delay while serial realy gets started
  delay(5000);
  Serial.println();
  Serial.println(F("bme680station  debug"));
  Serial.flush();

  // countdown a brief delay 
  for(int waitSeconds = 4; waitSeconds > 0; waitSeconds--) {
        Serial.printf("[SETUP] WAIT %d...\n", waitSeconds);
        Serial.flush();
        delay(1000);
    }
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  // configure the BME680 sensor
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // BME680 oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  // configure at least one wifi AP to select from 
  // todo will make this configurable, etc 
  wifiMulti.addAP("BringBeerTo363", "BudLightBeer");

  if((wifiMulti.run() == WL_CONNECTED)) {
    printWifiStatusToSerial();
  }

}

void loop() {
  // blink while we process the BME680
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW

  // grab and process a reading from the BME680 
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  float tempC = (bme.temperature);
  float tempF = ((tempC * (9.0/5.0)) + 32.0);
  float pressure = (bme.pressure / 100.0);
  float humidity = (bme.humidity);
  float gas = (bme.gas_resistance / 1000.0);
  float altitude = (bme.readAltitude(SEALEVELPRESSURE_HPA));
  int voltageRaw = analogRead(35);
  float voltage = ((( float(voltageRaw) * 2.0) / 4096.0) * 3.3);

  Serial.println();
  Serial.print("Temperature = ");
  Serial.print(tempC);
  Serial.print(" *C  or ");
  Serial.print(tempF);
  Serial.println(" *F");
  
  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(gas);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(altitude);
  Serial.println(" m");
  
  Serial.print("Voltage RAW from pin: ");
  Serial.print(voltageRaw);
  Serial.print("  voltage = ");
  Serial.println(voltage, 3);

  // here is the wire format for the influxDB we are using
  // bme680,location=bakerz Temp=26.43,TempF=79.57,hPa=771.98,RH=21.08,VOCKOhms=101.33,Altitude=2236.10,Vraw=2367,Voltage=3.812

  // conect UDP to InfluxDB server and send data 
  
  //if(udp.beginPacket(IPAddress(255,255,255,255), 8089)) {
  if(udp.connect(IPAddress(255,255,255,255), 8089)) {
      //Serial.println("UDP connected"); // Debug Only
      delay(50); //Needed cause sometimes no Delay = can't send UDP packages fast enough
      
      String influxData = ("bme680,location=363Office Temp=" + String(tempC) + ",TempF=" +String(tempF) + ",hPa=" + String(pressure) + \
      ",RH=" +String(humidity) + ",VOCOhms=" +String(gas) + ",Altitide=" + String(altitude) + ",Vraw=" + String(voltageRaw) + \
      ",Voltage=" + String(voltage));
      
      Serial.println("send influxdata over UDP:");
      Serial.println(influxData);
      Serial.println();

      udp.print(String(influxData));
      // udp.endPacket();

      delay(50); //Not really sure if needed....
  }

  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(10000);
}

void printWifiStatusToSerial() {
    // I let Copilot re-do this for me
    // print the SSID of the network you're attached to:
    Serial.println();
    Serial.println(F("Wifi connection info:"));
    byte mac[6];
    WiFi.macAddress(mac);
    Serial.print(F("MAC: "));
    Serial.print(mac[5], HEX);
    Serial.print(":");
    Serial.print(mac[4], HEX);
    Serial.print(":");
    Serial.print(mac[3], HEX);
    Serial.print(":");
    Serial.print(mac[2], HEX);
    Serial.print(":");
    Serial.print(mac[1], HEX);
    Serial.print(":");
    Serial.println(mac[0], HEX);
    Serial.println(F("Wifi Status:"));
    Serial.print(F("Status: "));
    Serial.println(WiFi.status());
    Serial.print(F("SSID: "));
    Serial.println(WiFi.SSID());
    Serial.print(F("BSSID: "));
    Serial.println(WiFi.BSSIDstr());
    Serial.print(F("Channel: "));
    Serial.println(WiFi.channel());
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("Subnet mask: "));
    Serial.println(WiFi.subnetMask());
    Serial.print(F("Gateway IP: "));
    Serial.println(WiFi.gatewayIP());
    Serial.print(F("DNS: "));
    Serial.println(WiFi.dnsIP());
    Serial.print(F("Signal strength (RSSI): "));
    Serial.println(WiFi.RSSI());
    Serial.println();
}
