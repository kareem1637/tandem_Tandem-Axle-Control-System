
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
  const char* ssid = "ESP32_AP";
  const char* password = "123456789";
  // Static IP address configuration
  IPAddress staticIP(192, 168, 4, 4); // Desired static IP address
  IPAddress gateway(192, 168, 4, 1);   // IP address of the main ESP32 AP
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns(192, 168, 4, 1);       // Optional: DNS server
  const char* mqtt_server = "192.168.4.2";
  const int mqtt_port = 1883;
  const char* mqtt_topic_rear = "outTopic/node3";
  const char* mqtt_topic = "inputTopic";
  WiFiClient espClient;
  PubSubClient client(espClient);
  AsyncWebServer server(80);
  Preferences preferences;
// weight sensors pins and Setup
  const int HX711_dout_1 = 15; // mcu > HX711 dout pin
  const int HX711_sck_1 = 2;  // mcu > HX711 sck pin
  HX711_ADC rearaxile(HX711_dout_1, HX711_sck_1);
  unsigned long t = 0;
  float rear_weight=0;
// Define pins for motor control
  #define ENCODER_A_PIN 33
  #define ENCODER_B_PIN 32
  #define Enable 25
  #define MOTOR_PIN_A 21
  #define MOTOR_PIN_B 19
  volatile long encoderValue = 0;
  volatile int lastEncoded = 0;
  #define STEPS_PER_CM 70 // Steps per centimeter based on calculations
// define variables
  int temp =0;
  //variables to compensate the -value of readings
  float temp_rear=0;
   int prevX = 0;
   String prevOrder = ""; // Variable to store previous order
   int currentPosition = 0; // Variable to store the current position
   int lastMovement = 0; // Variable to store the last movement direction
  unsigned long lastReconnectAttempt = 0;
// define servos 
  static const int servoPIN = 14;
  Servo servoPINS;



bool reconnect() {
  if (client.connect("ESP2")) {
    Serial.println("MQTT connected");
    client.subscribe(mqtt_topic);
    return true;
  } else {
    Serial.print("MQTT connection failed, rc=");
    Serial.println(client.state());
    return false;
  }
}
//function to callibrate the weight sensor send 'r' to recallibrate any time   **you should calibrate the sensor every time the tandem moves**
void calibrate(HX711_ADC &loadCell) {
  Serial.println("***");
  Serial.println("Start calibration:");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("Send 't' from serial monitor to set the tare offset.");

  boolean _resume = false;
  while (_resume == false) {
    loadCell.update();
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 't') loadCell.tareNoDelay();
    }
    if (loadCell.getTareStatus() == true) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

  Serial.println("Now, place your known mass on the loadcell.");
  Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    loadCell.update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass != 0) {
        Serial.print("Known mass is: ");
        Serial.println(known_mass);
        _resume = true;
      }
    }
  }

  loadCell.refreshDataSet(); // Refresh the dataset to be sure that the known mass is measured correctly
  float newCalibrationValue = loadCell.getNewCalibration(known_mass); // Get the new calibration value

  Serial.print("New calibration value has been set to: ");
  Serial.print(newCalibrationValue);
  
  Serial.println("End calibration");
  Serial.println("***");
  Serial.println("To re-calibrate, send 'r' from serial monitor.");

}
void IRAM_ATTR handleEncoder() {
  int MSB = digitalRead(ENCODER_A_PIN); // Most significant bit
  int LSB = digitalRead(ENCODER_B_PIN); // Least significant bit

  int encoded = (MSB << 1) | LSB; // Combine the two pin values
  int sum = (lastEncoded << 2) | encoded; // Combine the previous and current encoder values

  // Check the transitions to determine the direction
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderValue++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderValue--;
  }

  lastEncoded = encoded; // Store the current encoder value
}
void moveMotorDistance(float distance_cm, String direction) {
  long targetSteps = distance_cm * STEPS_PER_CM;
  long initialEncoderValue = encoderValue;

  if (direction == "forward") {
    digitalWrite(MOTOR_PIN_A, LOW);
    digitalWrite(MOTOR_PIN_B, HIGH);
    analogWrite(Enable, 255); // 50% duty cycle forward
  } else if (direction == "backward") {
    digitalWrite(MOTOR_PIN_A, HIGH);
    digitalWrite(MOTOR_PIN_B, LOW);
    analogWrite(Enable, 255); // 50% duty cycle backward
  }

  while (abs(encoderValue - initialEncoderValue) < targetSteps) {
    // Wait until target steps are reached
  }

  // Stop the motor
  analogWrite(Enable, 0); // Duty cycle 0 (motor off)
}

//pins
void lockpin() {
    servoPINS.write(130);
}

void unlockpin() {
    servoPINS.write(0);
}

void setup_wifi() {
  delay(10);
  // Set the static IP address
  if (!WiFi.config(staticIP, gateway, subnet, dns)) {
    Serial.println("STA Failed to configure");
  }

  // Connect to the AP
  WiFi.begin(ssid, password);

  // Check for the connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // Print the IP address
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

//this the function for get the new pins postion 
 void callback(char* topic, byte* payload, unsigned int length) {
  // Parse JSON payload
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, payload, length);
  
  // Access JSON data
  int x = doc["x"];
  String order = doc["order"];  
  // Check if incoming data is different from previous data
  if (order == prevOrder && x == prevX) {
    return; // Exit the function early if the order and x remain the same
  }
  unlockpin();
  
  int movement = 0; // Variable to store the movement adjustment
  int temp_movement=0;
  // Determine the movement adjustment based on the received order
  if (order.indexOf("backward") != -1) {
    movement = -1; // Move backward
    temp_movement=-1;
  } else if (order.indexOf("forward") != -1) {
    movement = 1; // Move forward
    temp_movement=1;
  } else {
    // Neither "backward" nor "forward" found in order string
    Serial.println("Invalid order received");
    return; // Exit the function early if the order is invalid
  }
  
   // Adjust the movement if it's in the same direction as the last movement
  if (movement == lastMovement) {
    if (x > prevX) {
     x -= prevX;} // Subtract the last movement to adjust for incremental movement
     else {
      // Check if movement is reversed
      if (movement == 1) { // Forward movement reversed to backward
        movement = -1; // Change movement direction to backward
        temp_movement=1;
    } else if (movement == -1) { // Backward movement reversed to forward
        movement = 1; // Change movement direction to forward
        temp_movement=-1;
    }
      x = prevX - x;} // Adjust x for backward movement
    
   
  }
    // Update the current position and last movement
    currentPosition += (movement * x); // Adjust current position based on movement direction
    
  
  
  // Perform the movement action
  if (movement == -1) {
    moveMotorDistance(x, "backward"); // Call backward function with currentPosition and backward as arguments
    Serial.print("Moving the tandem backward by ");
    Serial.print(x);
    Serial.println(" pins");
  } else if (movement == 1) {
    moveMotorDistance(x, "forward"); // Call forward function with currentPosition and forward as arguments
    Serial.print("Moving the tandem forward by ");
    Serial.print(x);
    Serial.println(" pins");
  }
  
  // Update previous order
  prevOrder = order;
  prevX = abs(currentPosition);
  movement=temp_movement ; 
  lastMovement = movement;
  
  lockpin();
}

// read the weight sensor and send the data to the flask app through mqtt client 
void send(){
  static boolean newDataReady1 = 0;
  const int serialPrintInterval = 200; // increase value to slow down serial print activity
  // Check for new data from the first sensor
  if (rearaxile.update()) newDataReady1 = true;
  // Get smoothed value from the first sensor dataset
  if (newDataReady1) {
    if (millis() > t + serialPrintInterval) {
      rear_weight = rearaxile.getData()/1000;
      if(rear_weight<0){
        temp_rear=rear_weight;
        rear_weight=0;}   
      }
      //compensate here
      if(rear_weight>0.25){
        rear_weight=rear_weight+temp_rear;
      }
  }
   
}
void setupServer() {
  // Set up routes for the web server
    server.on("/motor", HTTP_GET, [](AsyncWebServerRequest *request){
      if (request->hasParam("direction")) {
        String direction = request->getParam("direction")->value();
        moveMotorDistance(2, direction);
        request->send(200, "text/plain", "Motor moved " + direction);
      } else {
        request->send(400, "text/plain", "Bad Request");
      }
    });
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
    server.on("/tare", HTTP_GET, [](AsyncWebServerRequest *request){
      if (request->hasParam("esp") && request->hasParam("type")) {
        int esp = request->getParam("esp")->value().toInt();
        String type = request->getParam("type")->value();
          if (esp == 2 && type == "tandem") {
          rearaxile.tareNoDelay();
          request->send(200, "text/plain", "tandem axle tared");
        } else {
          request->send(400, "text/plain", "Invalid parameters");
        }
      } else {
        request->send(400, "text/plain", "Missing parameters");
      }
    });
  // Calibration route
    server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request){
      if (request->hasParam("esp") && request->hasParam("type") && request->hasParam("weight")) {
        int esp = request->getParam("esp")->value().toInt();
        String type = request->getParam("type")->value();
        float Known_weight = request->getParam("weight")->value().toFloat();
        if (esp == 2 && type == "tandem") {
          rearaxile.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
          float rearCalValue = rearaxile.getNewCalibration(Known_weight); // Get and set the new calibration value
          // Save the new calibration values to EEPROM
          preferences.begin("calibration", false);
          preferences.putFloat("rearCalValue", rearCalValue);
          preferences.end();
          request->send(200, "text/plain", "drive axle calibrated with factor: " + String(rearCalValue));
        } else {
          request->send(400, "text/plain", "Invalid parameters");
        }
      } else {
        request->send(400, "text/plain", "Missing parameters");
      }
    });
    // Start the server
    server.begin();
}

void setup() {
  Serial.begin(57600);
  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
  pinMode(Enable, OUTPUT);

  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), handleEncoder, CHANGE);
  // Stop the motor
  analogWrite(Enable, 0); // Duty cycle 0 (motor off)
  servoPINS.attach(servoPIN);

  /////////
  setup_wifi();
  setupServer();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  rearaxile.begin();
  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  rearaxile.start(stabilizingtime, _tare);
  // Check for initialization errors
  if (rearaxile.getTareTimeoutFlag() || rearaxile.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1)
      ;
  } else {
    preferences.begin("calibration", true);
    float rearCalValue = preferences.getFloat("rearCalValue", 259.42); // Default to 195.08 if not set
    preferences.end();
    rearaxile.setCalFactor(rearCalValue);
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
    String rear_data_str = String(rear_weight);
    // Publish sensor data to MQTT topics
    client.publish(mqtt_topic_rear, rear_data_str.c_str());
      delay(250);
    // Check if last tare operation is complete
    if (rearaxile.getTareStatus() == true) {
    delay(500); // Adjust delay as needed
    }
  }
   if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') {
      rearaxile.tareNoDelay(); // Tare the first sensor
    }  
  }
}
