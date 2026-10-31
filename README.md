OVERVIEW

This project implements a fall detection system. This code uses an Arduino UNO R4, MPU6050 accelerometer and gyroscope sensor, and Blynk IoT. The system detects sudden falls based on accelerometer and gyroscope data and sends notifications to a Blynk mobile app. The physical prototype and code will detect a fall due to rapid change of position in a short time and differentiates itself from normal daily activities. Once detected, the Arduino sends an alert (unless the push button is pressed).


HARDWARE REQUIREMENTS

- Arduino UNO R4

- MPU6050

- momentary push button 

- Blynk IoT 

- Breadboard

- 6x male wires

- USB-C for programming and power supply (ensure it supports data transfer, not just power supply)


SOFTWARE REQUIREMENTS

- Arduino IDE (including relevant libraries)

- Blynk mobile app

INSTALLATION

1. Attach the MPU6050 sensor to the breadboard as per the wiring diagram.

2. Install the necessary libraries in the Arduino IDE:
- Wire.h
- SoftwareSerial.h
- WiFiS3.h
- BlynkSimpleWifi.h
3. Open the provided Arduino sketch (sketch_oct29a_copy_20261031083621_copy_2026_copy_20261031143155) in Arduino IDE.
4. Replace the placeholder values in the sketch with your Blynk authentication token and Wi-Fi credentials.
5. Upload the sketch to your Arduino UNO R4 Wifi board.

USAGE

1. Open the Blynk mobile app on your smartphone.
2. Create a new project and obtain the Blynk authentication token (Devices).
3. Create an Event on app with event code, fall_alert, and enable notifications.
 - Also, go to developer zone > datastream > then make 2 seperate datastreams. 
4. One will have pin: V1 (integer) and other will be pin: V2 (integer) - this ensures data transmits to BLYNK app
5. Run the Arduino board with the uploaded sketch.
6. Monitor the Blynk app for fall detection alerts and status updates (in Serial Monitor).

Configuration

- Adjust the fall threshold and gyro threshold variables in the Arduino sketch to fine-tune the fall detection sensitivity.
- Customise the Blynk project notifications as per your preferences

TROUBLESHOOTING

- Ensure the MPU6050 sensor and push button is properly connected to the Arduino board.
- Verify that the Wi-Fi credentials and Blynk authentication token are correctly set in the sketch.
- Check the serial monitor for debug messages and sensor readings to diagnose any issues.

WIRING DIAGRAM
<img width="957" height="538" alt="Screenshot 2026-10-31 at 1 52 52 pm" src="https://github.com/user-attachments/assets/6125c6c3-50f7-4463-af51-5ab4e41dd01e" />
