# Smart Fall Detection Band

## Overview
The **Smart Fall Detection Band** is a real-time fall detection system built using the ESP32 microcontroller and various sensors, including an accelerometer and gyroscope (MPU6050), heart rate sensor (MAX30102), and GPS module (NEO-6M). The system detects falls using a custom sensor fusion algorithm and sends real-time notifications via MQTT for remote monitoring.

This project is designed to improve health monitoring for elderly individuals or people at risk of falling, allowing caretakers to be notified immediately if a fall occurs, along with the exact GPS location.

## Features
1. **Real-time Fall Detection**: Utilizes an accelerometer and gyroscope for detecting abnormal motion patterns that indicate a fall.
2. **Heart Rate Monitoring**: Continuously monitors heart rate using the MAX30102 sensor and incorporates it into the fall detection logic.
3. **GPS Location Tracking**: Captures the GPS coordinates of the user at the time of the fall using the NEO-6M GPS module.
4. **Remote Notifications**: Uses MQTT protocol to publish fall alerts, heart rate data, and GPS location to a cloud server for real-time monitoring via a mobile app or web interface.
5. **Compact Design**: A compact PCB integrates all components, powered by a Li-ion battery, with a buzzer for audible alerts.

## Components
- **ESP32**: Main microcontroller for managing sensors and communication.
- **MPU6050**: 6-axis accelerometer and gyroscope used for motion detection.
- **MAX30102**: Pulse oximeter sensor used for heart rate monitoring.
- **NEO-6M GPS Module**: GPS module for tracking the user’s location.
- **MQTT Broker**: Used for publishing heart rate and location data for remote monitoring.

## Project Structure
This repository contains the following:
- **Smart_Fall_Detection_Band.ino**: The main Arduino code for the ESP32. This file integrates all sensors, processes the data, detects falls, and sends data to the MQTT broker.

## Hardware Setup
1. **ESP32**: The core microcontroller, responsible for sensor data processing and communication with the MQTT broker over WiFi.
2. **MPU6050**: Mounted on the PCB to detect changes in acceleration and angular velocity, providing key data for fall detection.
3. **MAX30102**: Positioned to monitor heart rate, used as a secondary indicator in the event of a fall.
4. **NEO-6M GPS Module**: Captures the location of the user at the time of the fall and transmits it via MQTT.
5. **Li-ion Battery**: Powers the entire system, providing portability.
6. **Buzzer**: Used for audible alerts when a fall is detected.

## Software Dependencies
The following libraries are required:
- **MPU6050.h**: Library for the MPU6050 accelerometer and gyroscope sensor.
- **MAX30102_PulseOximeter.h**: Library for the MAX30102 heart rate sensor.
- **TinyGPS++.h**: Library for parsing GPS data from the NEO-6M module.
- **PubSubClient.h**: Library for MQTT communication.
- **ESP8266WiFi.h**: Library to connect the ESP32 to WiFi.
- **I2Cdev.h**: Library for I2C communication with the MPU6050 sensor.

## How It Works
1. **Fall Detection Algorithm**: 
   - The accelerometer and gyroscope data are fused to calculate a resultant acceleration and angular velocity.
   - If the resultant acceleration exceeds a threshold, it indicates the possibility of a fall. The system then monitors the angle of rotation and the heart rate to confirm the fall.
   - If the resultant angle exceeds a threshold and the heart rate is outside normal bounds, a fall is detected.

2. **Real-Time Monitoring**: 
   - Upon detecting a fall, the system sends the user’s GPS coordinates and heart rate data to a remote server via MQTT.
   - A mobile app or web interface can subscribe to the MQTT topics and receive real-time notifications, along with location data.

3. **Buzzer Alert**: 
   - In addition to sending remote notifications, the system activates a buzzer to alert nearby individuals in the event of a fall.

## MQTT Topics
The data is published to the following MQTT topics:
- `FallDetection_Data_HR`: Publishes heart rate data.
- `FallDetection_Data_GPT`: Publishes GPS latitude.
- `FallDetection_Data_GPG`: Publishes GPS longitude.

## Future Improvements
- Mobile App Development: Enhance the mobile app for notifications, locations and better vivualization.
- Improved Accuracy: Further refine the sensor fusion algorithm to reduce false positives/negatives.
- Power Optimization: Implement power-saving features to extend battery life.

## Acknowledgments
This project is inspired by the need for enhanced safety in elderly care, aiming to contribute to UN Sustainable Development Goal 3 (Good Health and Well-being), and was built as part of a course Introduction to IoTs.
