#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HX711_ADC.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

// WiFi and MQTT setup
const char *ssid = "ESP32_AP";
const char *password = "123456789";
IPAddress local_ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
const char* mqtt_server = "192.168.4.2";
const int mqtt_port = 1883;
const char* mqtt_topic_drive = "outTopic/node2";
const char* mqtt_topic_steer = "outTopic/node1";
const char* mqtt_topic = "inputTopic";
WiFiClient espClient;
PubSubClient client(espClient);
AsyncWebServer server(80);
Preferences preferences;

// Weight sensors pins and setup
const int HX711_dout_1 = 15; // mcu > HX711 dout pin
const int HX711_sck_1 = 2;   // mcu > HX711 sck pin
const int HX711_dout_2 = 5;  // mcu > HX711 dout pin
const int HX711_sck_2 = 18;  // mcu > HX711 sck pin
HX711_ADC driveaxile(HX711_dout_2, HX711_sck_2);
HX711_ADC steeraxile(HX711_dout_1, HX711_sck_1);
unsigned long t = 0;
float drive_weight = 0;
float steer_weight = 0;

// Motor control pins
#define ENCODER_A_PIN 14
#define ENCODER_B_PIN 27
#define MOTOR_PIN_A 26
#define MOTOR_PIN_B 25
volatile long encoderValue = 0;
volatile int lastEncoded = 0;
#define STEPS_PER_CM 177.3 // Steps per centimeter based on calculations

// Ultrasonic sensor pins
int trigPin = 19;  // Arduino pin tied to trigger pin on the ultrasonic sensor.
int echoPin = 21;  // Arduino pin tied to echo pin on the ultrasonic sensor.

// Variables
long duration;
float temp_steer = 0;
float temp_drive = 0;
unsigned long lastReconnectAttempt = 0;

// Servo control
static const int servoPIN = 22;
static const int servoSTER = 23;
Servo servoPINS;
Servo servoSTEER;

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

// Steering functions
void turn_right() {
  servoSTEER.write(70);
}

void turn_left() {
  servoSTEER.write(150);
}

void centered() {
  servoSTEER.write(110);
}

// Pin control functions
void lockpin() {
  servoPINS.write(180);
}

void unlockpin() {
  servoPINS.write(70);
}

// Read the ultrasonic data
float getdistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2.0;
  return distance;
}

void setup_wifi() {
  // Setting up the AP
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
}

void send() {
  static boolean newDataReady1 = 0;
  const int serialPrintInterval = 200; // Increase value to slow down serial print activity

  if (driveaxile.update() && steeraxile.update()) newDataReady1 = true;

  if (newDataReady1) {
    if (millis() > t + serialPrintInterval) {
      drive_weight = driveaxile.getData() / 1000;
      if (drive_weight < 0) {
        temp_drive = drive_weight;
        drive_weight = 0;
      }

      steer_weight = steeraxile.getData() / 1000;
      if (steer_weight < 0) {
        temp_steer = steer_weight;
        steer_weight = 0;
      }

      t = millis();
      newDataReady1 = 0;
    }

    if (steer_weight > 0.25 || drive_weight > 0.25) {
      drive_weight = drive_weight + temp_drive;
      steer_weight = steer_weight + temp_steer;
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

  server.on("/servo", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("direction")) {
      String direction = request->getParam("direction")->value();
      if (direction == "left") {
        turn_left();
        request->send(200, "text/plain", "Servo moved left");
      } else if (direction == "right") {
        turn_right();
        request->send(200, "text/plain", "Servo moved right");  
      } else if (direction == "centered") {
        centered();
        request->send(200, "text/plain", "Servo centered");
      } else {
        request->send(400, "text/plain", "Invalid direction");
      }
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

  server.on("/tare", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("esp") && request->hasParam("type")) {
      int esp = request->getParam("esp")->value().toInt();
      String type = request->getParam("type")->value();
      if (esp == 1 && type == "steer") {
        steeraxile.tareNoDelay();
        request->send(200, "text/plain", "Steering axle tared");
      } else if (esp == 1 && type == "drive") {
        driveaxile.tareNoDelay();
        request->send(200, "text/plain", "drive axle tared");
      } else {
        request->send(400, "text/plain", "Invalid parameters");
      }
    } else {
      request->send(400, "text/plain", "Missing parameters");
    }
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
      if (esp == 1 && type == "steer") {
        steeraxile.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
        float steerCalValue = steeraxile.getNewCalibration(Known_weight); // Get and set the new calibration value
        // Save the new calibration values to EEPROM
        preferences.begin("calibration", false);
        preferences.putFloat("steerCalValue", steerCalValue);
        preferences.end();
        request->send(200, "text/plain", "Steering axle calibrated with factor: " + String(steerCalValue));
      } else if (esp == 1 && type == "drive") {
        driveaxile.refreshDataSet();//refresh the dataset to be sure that the known mass is measured correct 
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

  // Start the server
  server.begin();
}

void setup() {
  Serial.begin(57600);
  // Define the pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
  digitalWrite(MOTOR_PIN_A, LOW);
  digitalWrite(MOTOR_PIN_B, LOW);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), handleEncoder, CHANGE);
  
  servoPINS.attach(servoPIN);
  servoSTEER.attach(servoSTER);

  setup_wifi();
  setupServer();
  client.setServer(mqtt_server, mqtt_port);
  driveaxile.begin();
  steeraxile.begin();

  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  driveaxile.start(stabilizingtime, _tare);
  steeraxile.start(stabilizingtime, _tare);

  if (driveaxile.getTareTimeoutFlag() || driveaxile.getSignalTimeoutFlag() ||
      steeraxile.getTareTimeoutFlag() || steeraxile.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  } else {
    preferences.begin("calibration", true);
    float driveCalValue = preferences.getFloat("driveCalValue", 195.08); // Default to 195.08 if not set
    float steerCalValue = preferences.getFloat("steerCalValue", 241.14); // Default to 241.14 if not set
    preferences.end();
    driveaxile.setCalFactor(driveCalValue);
    steeraxile.setCalFactor(steerCalValue);
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
    String steer_data_str = String(steer_weight);

    client.publish(mqtt_topic_drive, drive_data_str.c_str());
    delay(300);
    client.publish(mqtt_topic_steer, steer_data_str.c_str());
    delay(300);
    if (driveaxile.getTareStatus() == true || steeraxile.getTareStatus() == true) {
      delay(500);
    }
  }
  delay(1000);
  lockpin();
  delay(1000);
  unlockpin();
}
