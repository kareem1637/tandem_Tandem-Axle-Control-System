// MIT License
/*
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

//instructions

/*
* This code interfaces with a Flask app via MQTT to manage and adjust the position of a tandem axle based on weight sensor data.
* 
* Overview:
* 1. Calibration of weight sensors is performed initially.
* 2. Data Transmission: After calibration, weight data is sent to the Flask app, allowing for monitoring of the axle weights.
* 3. Dynamic Adjustment: If the Flask app detects an overweight condition on either the front or rear axle, it sends new pin positions via MQTT. The tandem axle is then adjusted accordingly.
* 
* Notes:
* - Ensure to replace the `ssid` and `password` with your WiFi credentials and update `mqtt_server` with the IP address of the device running the Flask app.
* 
* Libraries:
* - The code utilizes various open-source libraries for WiFi, MQTT, JSON parsing, weight sensor interfacing, and servo control. 
* - The respective copyrights for these libraries are reserved by their authors.
* - The main logic and structure of the code have been developed by us.
*/

// Include libraries
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HX711_ADC.h>
#include <ESP32Servo.h>
#include <cstdlib>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

// Setup the WiFi and MQTT
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

// Weight sensors pins and setup
const int HX711_dout_1 = 15; // mcu > HX711 dout pin
const int HX711_sck_1 = 2;  // mcu > HX711 sck pin
HX711_ADC rearaxile(HX711_dout_1, HX711_sck_1);
unsigned long t = 0;
float rear_weight = 0;

// Define pins for motor control
#define ENCODER_A_PIN 33
#define ENCODER_B_PIN 32
#define MOTOR_PIN_A 21
#define MOTOR_PIN_B 19
volatile long encoderValue = 0;
volatile int lastEncoded = 0;
#define STEPS_PER_CM 70 // Steps per centimeter based on calculations

// Define variables
int temp = 0;
// Variables to compensate the -value of readings
float temp_rear = 0;
int prevX = 0;
String prevOrder = ""; // Variable to store previous order
int currentPosition = 0; // Variable to store the current position
int lastMovement = 0; // Variable to store the last movement direction
unsigned long lastReconnectAttempt = 0;

// Define servos 
static const int servoPIN = 27;
static const int servoSTER = 14;
Servo servoPINS;
Servo servoSTEER;
int flage = 0;

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

// Function to move the motors forward
void moveMotorDistance(float distance_cm, String direction) {
  long targetSteps = distance_cm * STEPS_PER_CM;
  long initialEncoderValue = encoderValue;

  if (direction == "forward") {
    analogWrite(MOTOR_PIN_A, 128); // 50% duty cycle forward
    analogWrite(MOTOR_PIN_B, 0);   // Stop backward
  } else if (direction == "backward") {
    analogWrite(MOTOR_PIN_A, 0);   // Stop forward
    analogWrite(MOTOR_PIN_B, 128); // 50% duty cycle backward
  }

  while (abs(encoderValue - initialEncoderValue) < targetSteps) {
    // Wait until target steps are reached
  }

  // Stop the motor
  analogWrite(MOTOR_PIN_A, 0); // Duty cycle 0 (motor off)
  analogWrite(MOTOR_PIN_B, 0); // Duty cycle 0 (motor off)
}

// Steering
void turn_right() {
  servoSTEER.write(90);
  flage = 1;
}

void turn_left() {
  servoSTEER.write(160);
  flage = 0;
}

void centered() {
  if (flage == 1) {
    servoSTEER.write(130);
  } else if (flage == 0) {
    servoSTEER.write(120);
  }
}

// Pins
void lockpin() {
  servoPINS.write(180);
}

void unlockpin() {
  servoPINS.write(70);
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

// This is the function for get the new pins position 
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
  int temp_movement = 0;
  // Determine the movement adjustment based on the received order
  if (order.indexOf("backward") != -1) {
    movement = -1; // Move backward
    temp_movement = -1;
  } else if (order.indexOf("forward") != -1) {
    movement = 1; // Move forward
    temp_movement = 1;
  } else {
    // Neither "backward" nor "forward" found in order string
    Serial.println("Invalid order received");
    return; // Exit the function early if the order is invalid
  }

  // Adjust the movement based on the difference between current and previous values
  int difference = abs(currentPosition - x);
  int temp_difference = difference * 2;
  Serial.print("Movement: ");
  Serial.print(movement);
  Serial.print(", Distance: ");
  Serial.println(difference);

  if (difference > 0) {
    if (order.indexOf("forward") != -1) {
      moveMotorDistance(temp_difference, "forward");
      lastMovement = temp_movement;
    } else if (order.indexOf("backward") != -1) {
      moveMotorDistance(temp_difference, "backward");
      lastMovement = temp_movement;
    }
  }

  // Update the current position and previous values
  currentPosition = x;
  prevX = x;
  prevOrder = order;
  lockpin();
}

void setup() {
  Serial.begin(115200);
  // Call the function to connect to WiFi
  setup_wifi();
  // MQTT setup
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  // Connect to MQTT server
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe(mqtt_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }

  // Attach the servos
  servoPINS.attach(servoPIN);
  servoSTEER.attach(servoSTER);

  // Weight sensors setup
  rearaxile.begin();
  rearaxile.start(2000);
  rearaxile.setCalFactor(696.0);

  // Set initial encoder value
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), handleEncoder, CHANGE);
}

void loop() {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    client.loop();
  }

  // Calibration
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0;
  if (rearaxile.update()) newDataReady = 1;
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = rearaxile.getData();
      temp_rear = (i + 7.6);
      Serial.print("Rear Axle: ");
      Serial.println(temp_rear);
      t = millis();
      rear_weight = temp_rear;
      // Create a JSON document
      DynamicJsonDocument doc(256);
      doc["value"] = rear_weight;
      // Serialize JSON document to a string
      String json;
      serializeJson(doc, json);
      // Publish the JSON string via MQTT
      client.publish(mqtt_topic_rear, json.c_str());
    }
  }
  delay(10);
}
