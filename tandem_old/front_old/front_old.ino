/*
 * This code sends weight data to a Flask app via MQTT and receives instructions from the app to adjust the position of the tandem axle.
 * 1. The tandem starts at position 8 for the drive pin and 18 for the rear pin. Calibration is performed initially.
 * 2. The code prompts the user to calibrate the drive and rear weight sensors. Follow the instructions in the Serial monitor.
 * 3. After calibration, the weights are sent to the Flask app, where they can be monitored.
 * 4. When the Flask app detects an overweight condition on either the drive or rear axle, it sends new pin positions here through the callback function, and the tandem axle is adjusted accordingly.
 * 
 * Note: There is a 5-second delay after receiving instructions to process the action. You can modify the Flask app to send instructions more quickly.
 * 
 * To run the code, replace the ssid and password with your WiFi name and password, and change mqtt_server to the IP address of the device running the Flask app.
 * The libraries used in this code are open-source and their copyrights are reserved for their respective authors.
 * The main code presented here is developed by us.
 */
// lnclude libraries
  #include <WiFi.h>
  #include <PubSubClient.h>
  #include <ArduinoJson.h>
  #include <HX711_ADC.h>
  #include <ESP32Servo.h>
  #include <cstdlib>
  #include <ESPAsyncWebServer.h>
  #include <Preferences.h>
// setup the wifi and mqtt
  const char *ssid = "ESP32_AP";
  const char *password = "123456789";
  // IP address of the Access Point (AP)
  IPAddress local_ip(192,168,4,1);
  IPAddress gateway(192,168,4,1);
  IPAddress subnet(255,255,255,0);
  const char* mqtt_server = "192.168.4.2";
  const int mqtt_port = 1883;
  const char* mqtt_topic_drive = "outTopic/node2";
  const char* mqtt_topic_steer = "outTopic/node1";
  WiFiClient espClient;
  PubSubClient client(espClient);
  AsyncWebServer server(80);
  Preferences preferences;
// weight sensors pins and Setup
  const int HX711_dout_1 = 15; // mcu > HX711 dout pin
  const int HX711_sck_1 = 2;  // mcu > HX711 sck pin
  HX711_ADC driveaxile(HX711_dout_1, HX711_sck_1);
  unsigned long t = 0;
  float drive_weight=0;

  int trigPin=  25 ;// Arduino pin tied to trigger pin on the ultrasonic sensor.
  int echoPin =   26;// Arduino pin tied to echo pin on the ultrasonic sensor.

// define variables
  long duration;
  float temp_steer = 0;
  float temp_drive = 0;
  unsigned long lastReconnectAttempt = 0;
// define servos 
  static const int servoPIN = 14;
  Servo servoPINS;



bool reconnect() {
  if (client.connect("ESP1")) {
    Serial.println("MQTT connected");
    return true;
  } else {
    Serial.print("MQTT connection failed, rc=");
    Serial.println(client.state());
    return false;
  }
}
//pins
void lockpin() {
    servoPINS.write(120);
}

void unlockpin() {
    servoPINS.write(0);
}
// read the ultrasonic data  **feedback for the dc motors**
float getdistance() {
  // Trigger ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Measure echo duration
  duration = pulseIn(echoPin, HIGH);
  // Calculate distance in cm with two decimal places
  float distance = duration * 0.034 / 2.0;
  // Return distance
  return distance;
}
void setup_wifi() {
  // Setting up the AP
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
}
void setupServer() {
    server.on("/lock", HTTP_GET, [](AsyncWebServerRequest *request){
      lockpin();
      request->send(200, "text/plain", "Lockpin activated");
    });

    server.on("/unlock", HTTP_GET, [](AsyncWebServerRequest *request){
      unlockpin();
      request->send(200, "text/plain", "Unlockpin activated");
    });
    server.on("/set_ip", HTTP_POST, [](AsyncWebServerRequest *request){
      if (request->hasParam("ip_address", true)) {
        String ip_address = request->getParam("ip_address", true)->value();
        mqtt_server = ip_address.c_str(); // Update MQTT server address
        request->send(200, "text/plain", "MQTT IP Address updated to: " + ip_address);
      } else {
        request->send(400, "text/plain", "Missing IP Address parameter");
      }
    });
    // Calibration route
  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("esp") && request->hasParam("type") && request->hasParam("weight")) {
      int esp = request->getParam("esp")->value().toInt();
      String type = request->getParam("type")->value();
      float Known_weight = request->getParam("weight")->value().toFloat();
        if (esp == 1 && type == "drive") {
         driveaxile.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
        float driveCalValue = driveaxile.getNewCalibration(Known_weight); // Get and set the new calibration value
        // Save the new calibration values to EEPROM
        preferences.begin("calibration", false);
        preferences.putFloat("driveCalValue", driveCalValue);
        preferences.end();
        request->send(200, "text/plain", "drive axle calibrated with factor: " + String(driveCalValue));
      } else {
        request->send(400, "text/plain", "Invalid parameters");
      }
    } else {
      request->send(400, "text/plain", "Missing parameters");
    }
  });
    server.on("/tare", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("esp") && request->hasParam("type")) {
      int esp = request->getParam("esp")->value().toInt();
      String type = request->getParam("type")->value();
        if (esp == 1 && type == "drive") {
        driveaxile.tareNoDelay();
        request->send(200, "text/plain", "drive axle tared");
      } else {
        request->send(400, "text/plain", "Invalid parameters");
      }
    } else {
      request->send(400, "text/plain", "Missing parameters");
    }
  });
    // Start the server
    server.begin();}

// read the weight sensor and send the data to the flask app through mqtt client 
void send(){
  static boolean newDataReady1 = 0;
  const int serialPrintInterval = 200; // increase value to slow down serial print activity
  // Check for new data from the first sensor
  if (driveaxile.update()) newDataReady1 = true;
  // Get smoothed value from the first sensor dataset
  if (newDataReady1) {
    if (millis() > t + serialPrintInterval) {
      drive_weight = driveaxile.getData()/1000;
      if(drive_weight<0){
        temp_drive=drive_weight;
        drive_weight=0;
      } 
      }
      //compensate here
      if( drive_weight>0.25){
        drive_weight=drive_weight+temp_drive;
      }
  }
   
}
void setup() {
  Serial.begin(57600);
  //define the pins 
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  servoPINS.attach(servoPIN);
  setup_wifi();
  setupServer();
  client.setServer(mqtt_server, mqtt_port);
  driveaxile.begin();
  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  driveaxile.start(stabilizingtime, _tare);
  // Check for initialization errors
  if (driveaxile.getTareTimeoutFlag() || driveaxile.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  } else {
    preferences.begin("calibration", true);
    float driveCalValue = preferences.getFloat("driveCalValue", 332.66); // Default to 195.08 if not set
    preferences.end();
    driveaxile.setCalFactor(driveCalValue);
    Serial.println("Startup is complete");
  }
}
void loop() {
  if (!client.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000) {  // Try to reconnect every 5 seconds
      lastReconnectAttempt = now;
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    client.loop();
     send();

    String drive_data_str = String(drive_weight);

    client.publish(mqtt_topic_drive, drive_data_str.c_str());
    delay(300);
    client.publish(mqtt_topic_steer, "0.3");
    delay(300);
    if (driveaxile.getTareStatus() == true) {
      delay(500);}
  }
    if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') {
      driveaxile.tareNoDelay(); // Tare the first sensor
    }  
  }
 
}