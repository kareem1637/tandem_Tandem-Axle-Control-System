# tandem_Tandem-Axle-Control-System
Overview
This repository contains the code for controlling a tandem axle system using an ESP32 microcontroller. The system interacts with a Flask application via MQTT to dynamically adjust the position of the tandem axle based on weight sensor data.

Features
Calibration of weight sensors for accurate readings.
Real-time transmission of weight data to a Flask app for monitoring.
Dynamic adjustment of axle position based on weight thresholds received via MQTT.
Libraries Used
WiFi: WiFi.h
MQTT: PubSubClient.h
JSON Parsing: ArduinoJson.h
HX711 ADC Library: HX711_ADC.h
Servo Control: ESP32Servo.h
Asynchronous Web Server: ESPAsyncWebServer.h
Preferences: Preferences.h
Setup Instructions
Hardware Requirements
ESP32 microcontroller board
HX711 load cell amplifier
Servo motors
WiFi network connection
Software Requirements
Arduino IDE (or compatible IDE)
Required libraries (listed above)
Installation Steps
Clone the Repository:

bash
Copy code
git clone https://github.com/kareem1637/tandem-axle-control.git
Open in Arduino IDE:

Launch Arduino IDE.
Open the tandem-axle-control.ino file from the cloned repository.
Install Libraries:

Navigate to Sketch > Include Library > Manage Libraries...
Search for each of the libraries listed in the "Libraries Used" section.
Click Install for each library.
Configure WiFi Credentials:

Replace ssid and password variables in the code with your WiFi credentials.
cpp
Copy code
const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";
Set MQTT Server IP:

Replace mqtt_server variable with the IP address of your MQTT broker (Flask app).
cpp
Copy code
const char* mqtt_server = "192.168.1.100";  // Replace with your MQTT broker IP
Adjust Pin Configurations (if needed):

Modify pin definitions in the code to match your hardware setup, especially if different pins are used for motor control, encoder, and servo.
Upload the Code:

Select your ESP32 board from Tools > Board menu.
Choose the correct port from Tools > Port menu.
Click Upload to upload the code to your ESP32.
Monitor Serial Output:

Open the Serial Monitor (Tools > Serial Monitor) to view debug messages and ensure everything is functioning correctly.
Usage
Calibration:
Upon startup, the system performs calibration of the weight sensors. Ensure stable conditions during calibration.
Operation:
The system continuously monitors weight data from the sensors.
When an MQTT message is received from the Flask app indicating an overweight condition on either axle, the system adjusts the axle position accordingly using servo and motor controls.
Troubleshooting
No WiFi Connection:

Ensure correct WiFi credentials are provided in the code.
Check if the ESP32 connects to your WiFi network (check Serial Monitor for debug messages).
MQTT Connection Issues:

Verify MQTT broker IP address (mqtt_server variable).
Check MQTT connection status in Serial Monitor (client.state()).
