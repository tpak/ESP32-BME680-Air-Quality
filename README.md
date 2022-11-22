# ESP32 and BME680 Air Quality Monitor sketch
Simple Arduino / ESP32 sketch to monitor temps and air quality
- connects to wifi
- sends data out on UDP
- the data can be picked up by Influx DB 
# boards and sensors
- Adafruit HUZZAH32 Feather (ESP32)
    - ESP32 Feather 16MB Flash WiFi + BT
- Adafruit BME680 breakout board
    - We need to use the BSEC Air Quality library (closed source) in order to get accurate air quality readings. This does not work with all boards but should work with ESP32.
    - https://learn.adafruit.com/adafruit-bme680-humidity-temperature-barometic-pressure-voc-gas/bsec-air-quality-library
    - we may also need to ahve a look at a different BME680 library from dfrobot
    - https://wiki.dfrobot.com/BME680_Environmental_Sensor_Module_SKU_SEN0375#target_0
