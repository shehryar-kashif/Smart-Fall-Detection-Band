#include <MPU6050.h>
#include <Wire.h>
#include <I2Cdev.h>
#include "MAX30102_PulseOximeter.h"                 //Library for Max30102 heart-rate sensor
#include <TinyGPS++.h>                              //Library for GPS sensor
#include <SoftwareSerial.h>                         //Library for software Serial communication through D5,D6
#include <PubSubClient.h>                           //Library for MQTT communication
#include <ESP8266WiFi.h>                            //Library for connecting to WiFi

#define REPORTING_PERIOD_MS     1000

const int Buzzer = 15;

TinyGPSPlus gps;                                    //Object to act as GPS Sensor
SoftwareSerial gpsSerial(D5,D6);                    //For communication through software serial

const char* ssid = "Shehryar's A32";                //WiFi credentials
const char* pswd = "HOT_SPOT!";

const char* mqtt_broker = "broker.hivemq.com";      //MQTT Broker details
const char* topicHR = "FallDetection_Data_HR";      //MQTT topic for heart rate
const char* topicGPSlat = "FallDetection_Data_GPT"; //MQTT topic for GPS Lat
const char* topicGPSlng = "FallDetection_Data_GPG"; //MQTT topic for GPS Long
const char* clientID = "NodeMCU_Cl1";               //Client ID
const int mqtt_port = 1883;                         //MQTT TCP Port

WiFiClient espClient;
PubSubClient client(espClient);

PulseOximeter pox;                                  //Object to act as Max30102 sensor
MPU6050 mpu;                                        //Object to act as MPU6050 sensor
unsigned long prevTime;                             //Varible to store previous time
unsigned long currentTime;                          //Variable to store current Time
float timeInterval;                                 //Variable for storing time Interval for measuring angle
bool fall;                                          //Variable that indicates fall has occured or not
int16_t ax, ay, az, gx, gy, gz, totalAngle;
float heartRate;                                    //Variable to store heart Rate
float latitude = 33.622841;                         //Variables to store location
float longitude = 72.957703;

void setup(){
  Serial.begin(115200);
  mpu.initialize();
  pox.begin();
  Wire.begin();
  gpsSerial.begin(115200);
  WiFi.begin(ssid, pswd);
  pinMode(9, OUTPUT);

  //Block to check MPU6050 connection
  if (!mpu.testConnection()){
    Serial.println("MPU connection failed!");
  }
  else{
    Serial.println("MPU Connected successfully!");
  }

  //Block to check Max30102 connection
  if (!pox.begin()){
    Serial.println("Pulse Oximeter connection failed");
  }
  else{
    Serial.println("Pulse Oximeter connected successfully!");
  }

  //Loop that waits 10 seconds to connect with WiFi
  Serial.print("Connecting to WiFi");
  for (int i = 0; i < 10; i++){
    delay(1000);
    Serial.print(".");
    if (WiFi.status() == WL_CONNECTED){
      break;
    }
  }

  if (WiFi.status() != WL_CONNECTED){
    Serial.print("WiFi not connected");
  }
  else{
    Serial.print("WiFi is connected!");
  }

  client.setServer(mqtt_broker, mqtt_port);
  client.connect(clientID);

  if (!client.connected()){
    Serial.print("MQTT not connected");
  }
  else{
    Serial.print("MQTT is connected!");
  }

  prevTime = 0;
  fall = 0;

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);        //Setting Accelerometer range to +/- 8g
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);        //Setting gyroscope range to 500 Deg/sec
  pox.setIRLedCurrent(MAX30102_LED_CURR_7_6MA);          //Setting Max30102 current to 6 mA instead of default 50mA

}

void loop(){
  //Get acceleration from accelerometer and normalizing it (Range of values is set from 0 to 8)
  mpu.getAcceleration(&ax, &ay, &az);
  ax = ax / 4096;
  ay = ay / 4096;
  az = az / 4096;
  float resAcc = pow((pow(ax,2) + pow(ay,2) + pow(az,2)),0.5);
  Serial.print("ResAcc = ");
  Serial.println(resAcc);

  heartRate = pox.getHeartRate();
  Serial.println(heartRate);
  //client.publish(topicHR, String(resAcc).c_str());

  //Calculating time for time Interval
  prevTime = millis();

  //Block that checks if Acceleration has exceeded threshold
  if (resAcc > 1){
    //Gets change in rotation (omega) and normalizes it
    mpu.getRotation(&gx, &gy, &gz);
    gx = gx / 131;
    gy = gy / 131;
    gz = gz / 131;

    //Calulation of time Interval for calculating angle
    currentTime = millis();
    timeInterval = currentTime - prevTime;
    gx = timeInterval * gx;
    gy = timeInterval * gy;
    gz = timeInterval * gz;
    totalAngle = pow((pow(abs(gx),2) + pow(abs(gy),2) + pow (abs(gz),2)),0.5);
    Serial.print("TotalAngle = ");
    Serial.print(totalAngle);

    //Block that checks if angle is greater than threshold or not
    if (totalAngle > 100){
      //Calculates acceleration after fall to detect whether person is stationary or not (for 10 seconds)
      resAcc = 0;
      heartRate = 0;
      for (int time = 0; time < 10; time++){
        mpu.getAcceleration(&ax, &ay, &az);
        ax = ax / 4096;
        ay = ay / 4096;
        az = az / 4096;
        resAcc = resAcc + pow((pow(ax,2) + pow(ay,2) + pow(az,2)),0.5);
        heartRate += pox.getHeartRate();
        delay(1000);
      }
      resAcc = resAcc / 10.0;
      Serial.print("ResAcc (2) = ");
      Serial.println(resAcc);
      heartRate = heartRate / 10.0;
      Serial.print("Heart Rate = ");
      Serial.println(heartRate);

      //Block that checks if fall occured or not
      if (resAcc < 2 && (heartRate > 90 || heartRate < 15) ){
        fall = 1;
        Serial.println(fall);
        client.publish(topicGPSlat, String(latitude, 6).c_str());
        client.publish(topicGPSlng, String(longitude, 6).c_str());

        for (int i = 0; i < 20; i++){
          analogWrite(13, 100);
          delay(500);
          analogWrite(13, 0);
          delay(500);
        }
        delay(5000);
      }
    }
  }

  if (gpsSerial.available()) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        Serial.print("Latitude: ");
        Serial.println(latitude, 6); // Latitude in degrees (float)
        Serial.print("Longitude: ");
        Serial.println(longitude, 6); // Longitude in degrees (float)
      } 
      else {
        Serial.println("Waiting for GPS signal...");
      }
    }
  }

  delay(1000);
}