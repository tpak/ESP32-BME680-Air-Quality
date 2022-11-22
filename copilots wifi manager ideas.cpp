Synthesizing 10/10 solutions

=======

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  // Uncomment and run it once, if you want to erase all the stored information
  //wifiManager.resetSettings();
  // Automatically connect using saved credentials,
  // if connection fails, it starts an access point with the specified name
  // here  "AutoConnectAP"
  // and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  // printWifiStatusToSerial();
  // delay(1000);

=======

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  // Uncomment and run it once, if you want to erase all the stored information
  // wifiManager.resetSettings();
  // Automatically connect using saved credentials,
  // if connection fails, it starts an access point with the specified name
  // here  "AutoConnectAP"
  // and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(1000);
  }

  // if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  // print the local IP address
  Serial.println(WiFi.localIP());
  Serial.println();

  // todo add a way to calibrate for this - google around there was an example
  // set the sea level pressure to 1013.25 hPa
  bme.setSeaLevelPressure(SEALEVELPRESSURE_HPA);


=======

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  // Uncomment and run it once, if you want to erase all the stored information
  //wifiManager.resetSettings();
  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect("AutoConnectAP");
  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  // // configure the udp client 
  // udp.begin(UDP_PORT);

  // // configure the udp client 
  // udp.begin(UDP_PORT);
  // udp.beginPacket("

=======

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  // Uncomment and run it once, if you want to erase all the stored information
  // wifiManager.resetSettings();

  //set custom ip for portal
  //wifiManager.setAPStaticIPConfig(IPAddress(10, 0, 1, 1), IPAddress(10, 0, 1, 1), IPAddress(255, 255, 255, 0));

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if(!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  // print the WiFi connection info to the serial console
  printWifiStatusToSerial();
  Serial.println("Starting UDP");
  udp.begin(8888);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());
  Serial.println("Waiting for sync");
  // wait for time to be set
  //setSyncProvider(RTC.get); // the function to get the time from the RTC
  if(timeStatus()!= timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");

  // print the sensor details to the serial console
  Serial.println(F("BME680:"));
  Serial.print(F("Chip ID: 0x")); Serial.println(bme.chip_id(), 16);
  Serial.print(F("Version: 0x")); Serial.println(bme.version(), 16);
  Serial.println(F("Calibration data:

=======

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  // Uncomment and run it once, if you want to erase all the stored information
  // wifiManager.resetSettings();
  // Automatically connect using saved credentials,
  // if connection fails, it starts an access point with the specified name
  // here  "AutoConnectAP"
  // and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  // if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  printWifiStatusToSerial();

  // set up udp
  udp.begin(9999);
  //udp.beginPacket("

=======

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  // Uncomment and run it once, if you want to erase all the stored information
  //wifiManager.resetSettings();

  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if(!wifiManager.autoConnect("AutoConnectAP")) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  // print the wifi info to serial
  printWifiStatusToSerial();
  // print the local IP address to serial
  Serial.println(WiFi.localIP());

  // set up the udp
  if (udp.connect("

=======

  // WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset settings - for testing
  //wifiManager.resetSettings();

  //set custom ip for portal
  //wifiManager.setAPStaticIPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if(!wifiManager.autoConnect()) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(1000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  // print the WiFi details to the serial monitor
  printWifiStatusToSerial();

  // start the UDP client
  // udp.begin(udpPort);
  // udp.beginPacket(udpRemoteIp, udpRemotePort);
  // udp.write("Hello World!");
  // udp.endPacket();

  // start the UDP client
  // todo make this configurable
  udp.begin(5000);
  udp.onPacket([](AsyncUDPPacket packet) {
    Serial.printf("UDP Packet Type: %d, From: %s:%d, To: %s:%d, Length: %u, Data: %s

=======

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wm;
  // Uncomment and run it once, if you want to erase all the stored information
  //wm.resetSettings();
  // Automatically connect using saved credentials,
  // if connection fails, it starts an access point with the specified name
  // here  "AutoConnectAP"
  // and goes into a blocking loop awaiting configuration
  wm.autoConnect("AutoConnectAP");

  // if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  // todo add a way to configure this
  udp.begin(4444);
  Serial.println("UDP listening on IP: ");
  Serial.println(WiFi.localIP());
  Serial.flush();

=======

  // WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset saved settings
  // wifiManager.resetSettings();

  //set custom ip for portal
  // wifiManager.setAPStaticIPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //set minimum quality of signal so it ignores AP's under that quality
  //defaults to 8%
  // wifiManager.setMinimumSignalQuality();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  // wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if(!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  // if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  // configure the UDP client
  udp.begin(1234);

=======

  // WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wm;
  //reset saved settings
  // wm.resetSettings();
  //set config save notify callback
  wm.setSaveConfigCallback(saveConfigCallback);
  //set static ip
  // wm.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  wm.autoConnect("AutoConnectAP");
  //or use this for auto generated name ESP + ChipID
  //wm.autoConnect();
  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  // print the network info
  printWifiStatusToSerial();

  // start the udp server
  // todo make this configurable, etc 
  udp.begin(8888);
  // udp.beginMulticast(WiFi.localIP(), IPAddress(239, 255, 255, 250), 8888);

  // print the network info
  printWifiStatusToSerial();

  // countdown a brief delay 
  for(int waitSeconds = 5; waitSeconds > 0; waitSeconds--) {
        Serial.printf("[SETUP] WAIT %d...\n", waitSeconds);
        Serial.flush();
        delay(1000);
    }

  // print the network info
  printWifiStatusToSerial();

  // print the sensor info
  printSensorStatusToSerial();
