#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

/***************************************************************************
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

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.println(F("BME680 test"));
  Serial.println();
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop() {

  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW
  float tempC = (bme.temperature);
  float tempF = ((tempC * (9.0/5.0)) + 32.0);
  float pressure = (bme.pressure / 100.0);
  float humidity = (bme.humidity);
  float gas = (bme.gas_resistance / 1000.0);
  float altitude = (bme.readAltitude(SEALEVELPRESSURE_HPA));
  int voltageRaw = analogRead(35);
  float voltage = ((( float(voltageRaw) * 2.0) / 4096.0) * 3.3);

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
  Serial.println();
  
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(1000);
}
