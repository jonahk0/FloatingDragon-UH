/**
Helper functions for calculating FlashIAP block device limits
**/

// Ensures that this file is only included once
#pragma once 

#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "FloatingDragon.h"


#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

// #include <Adafruit_BME688.h>
//#define LAUNCH_ALTITUDE -63.83
//#define GROUND_ALTITUDE -89.53
float LAUNCH_ALTITUDE = 30;
float altitude = LAUNCH_ALTITUDE;
float GROUND_ALTITUDE = 0;


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);


void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0xEC, 0x94, 0xCB, 0x6D, 0x02, 0x00};
//PILOT: EC:94:CB:6D:D5:34
//GROUND: EC:94:CB:6D:02:00

// Label Local Device
String device_name = "PILOT";

// Create a struct_message to hold outgoing message



// Create a struct_message to hold incoming message
struct_message incoming_message;

// Create a struct_message to hold outgoing message
struct_message outgoing_message;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    //success = "Delivery Success :)";
  }
  else{
    //success = "Delivery Fail :(";
  }
}




// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming_message, incomingData, sizeof(incoming_message));
  Serial.print("Bytes received: ");
  Serial.println(len);

}

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme;

unsigned long prevMillis = 0;
unsigned long currentMillis = millis();

struct_timer startTimer(int timerType){
  struct_timer newTimer;


  if(timerType == ONESECTIMER){
    newTimer.start_time = millis();
    newTimer.end_time = newTimer.start_time + 1000;
    newTimer.timeout = 0;
    altitude = altitude - 1;
  }
  else if (timerType == PARACHUTETIMER){
    newTimer.start_time = millis();
    newTimer.end_time = newTimer.start_time + 5000;
    newTimer.timeout = 0;
  }
  else if (timerType == MOTORTIMER){
    newTimer.start_time = millis();
    newTimer.end_time = newTimer.start_time + 3000;
    newTimer.timeout = 0;
  }

  return newTimer;
}

bool checkTimer(struct_timer runningTimer){
  if(runningTimer.end_time <= millis()){
    runningTimer.timeout = 1;
  }
  return runningTimer.timeout;
}
struct_timer motortimer = startTimer(MOTORTIMER);
struct_timer onesectimer = startTimer(ONESECTIMER);
struct_timer parachutetimer = startTimer(PARACHUTETIMER);



// Motor DRIVE B
#define MOTOR1_IN1 18
#define MOTOR1_IN2 5
// Motor DRIVE A
#define MOTOR2_IN1 23
#define MOTOR2_IN2 19

void startSensors(){
  if (!bme.begin()) {
   Serial.println("Could not find a valid BME680 sensor, check wiring!");
     while (1);
   }
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms  
}


// Forward function to control both motors at the same speed
void forward(){//int speed) {
  digitalWrite(MOTOR1_IN1, LOW);
  analogWrite(MOTOR1_IN2, 128);
  // digitalWrite(MOTOR2_IN1, LOW);
  // analogWrite(MOTOR2_IN2, 128);//half speed
}

// Left function to control the left motor faster than the right motor
void left(){//int speed) {
  // analogWrite(motorPin1, speed * 1.2);  // Increase left motor speed by 20%
  // analogWrite(motorPin2, speed);
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, HIGH);
  digitalWrite(MOTOR2_IN1, LOW);
  analogWrite(MOTOR2_IN2, 255 * 0.8);
}

// Right function to control the right motor faster than the left motor
void right(){//int speed) {
  // analogWrite(motorPin1, speed);
  // analogWrite(motorPin2, speed * 1.2);  // Increase right motor speed by 20%
  digitalWrite(MOTOR1_IN1, LOW);
  analogWrite(MOTOR1_IN2, 255 * 0.8);
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, HIGH);
}

void backward(){//int speed) {
  // analogWrite(motorPin1, speed);
  // analogWrite(motorPin2, speed);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR2_IN2, LOW);
  digitalWrite(MOTOR2_IN1, HIGH);
}

void stopMotors(){
  pinMode(MOTOR1_IN1, INPUT);
  pinMode(MOTOR1_IN2, INPUT);
  pinMode(MOTOR2_IN1, INPUT);
  pinMode(MOTOR2_IN2, INPUT);
}

void configureMotors(){
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);

  //stopMotors();
}

enum static_paths {
  FORWARDPATH,
  LEFTSLANTPATH,
  RIGHTSLANTPATH
};

void calculate_trajectory(){
  //nothing
}

void start_monitoring(){
    WiFi.mode(WIFI_STA);
    // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

int ideal_trajectory = FORWARDPATH;


void setup() {
  Serial.begin(115200);
  while (!Serial);
  // stopMotors();
  // startSensors();
  // calculate_trajectory();
  // start_monitoring();
  altitude = LAUNCH_ALTITUDE;

}
// Define states as constants
const int STATE_STARTUP = 0;
const int STATE_WAIT_FOR_LAUNCH = 1;
const int STATE_LAUNCH = 2;
const int STATE_DEPLOY_PARACHUTE = 3;
const int STATE_FLIGHT = 4;
const int STATE_LANDING = 5;
const int STATE_DONE = 6;

// Define initial state
int state = STATE_STARTUP;

String Flight_STATUS = "Start Up";


void loop() {
  // Execute code for current state
  delay(100);
  switch (state) {
    case STATE_STARTUP:
      // Perform actions for startup state
      // Transition to next state when startup is complete
      if(checkTimer(onesectimer)){
        //Send Data
        sendStatus();

        onesectimer = startTimer(ONESECTIMER);
      }
      Flight_STATUS = "Start Up";
      state = STATE_WAIT_FOR_LAUNCH;
      Serial.println(Flight_STATUS);
      break;
      
    case STATE_WAIT_FOR_LAUNCH:
      // Perform actions for waiting for launch state
      // Transition to next state when launch is detected
      if(checkTimer(onesectimer)){
        //Send Data
        sendStatus();

        onesectimer = startTimer(ONESECTIMER);
      }
      Flight_STATUS = "Waiting For Launch";
      if (launchDetected()) {
        state = STATE_LAUNCH;
      }
      break;
      
    case STATE_LAUNCH:
      // Perform actions for launch state
      // 
      if(checkTimer(onesectimer)){
        //Send Data
        sendStatus();

        onesectimer = startTimer(ONESECTIMER);
      }
      Flight_STATUS = "BEING LAUNCHED";
      state = STATE_DEPLOY_PARACHUTE;
      Serial.println(Flight_STATUS);
      break;
      
    case STATE_DEPLOY_PARACHUTE:
      // Perform actions for parachute deployment state
      // Transition to next state when landing is detected
      if(checkTimer(onesectimer)){
        //Send Data
        sendStatus();

        onesectimer = startTimer(ONESECTIMER);
      }
      
      Flight_STATUS = "DEPLOYING PARACHUTE";
      if (parachuteDeployed()) {
        //configureMotors();
        //forward();
        motortimer = startTimer(MOTORTIMER);
        state = STATE_FLIGHT;
        
        
      }
      Serial.println(Flight_STATUS);
      break;
      
    case STATE_FLIGHT:
      // Perform actions for flight state
      // Transition to next state when landing is detected
      if(checkTimer(onesectimer)){
        //Send Data
        sendSensorData();

        onesectimer = startTimer(ONESECTIMER);
      }
      
      if(checkTimer(motortimer)){
        //PID CONTROL
        //stopMotors();
        Serial.println(Flight_STATUS);

        motortimer = startTimer(MOTORTIMER);
      }
      Flight_STATUS = "IN FLIGHT";
      if (landingDetected()) {
        state = STATE_LANDING;
      }
      break;
      
    case STATE_LANDING:
      // Perform actions for landing state
      // Transition to next state when landing is complete
      if(checkTimer(onesectimer)){
        //Send Data
        sendStatus();

        onesectimer = startTimer(ONESECTIMER);
      }
      //configureMotors();
      //backward();

      Flight_STATUS = "BRACING FOR LANDING";
      state = STATE_DONE;
      Serial.println(Flight_STATUS);
      break;
      
    case STATE_DONE:
      // Perform actions for done state
      if(checkTimer(onesectimer)){
        //Send Data
        sendStatus();

        onesectimer = startTimer(ONESECTIMER);
      }

      //stopMotors();
      Flight_STATUS = "DONE";
      // Stay in this state indefinitely
      break;
  }
}

// Helper functions for detecting events

bool launchDetected() {
  // Code to detect launch event
  //float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  if(altitude <= (LAUNCH_ALTITUDE - 5)){
    return true;
  }
  else{ 
    return false;
    }
  // Returns true when launch is detected, false otherwise
}

bool parachuteDeployed() {
  // Code to detect parachute deployment event
  // Returns true when parachute is deployed, false otherwise
  return true; 
}

bool landingDetected() {
  // Code to detect landing event
  // Returns true when landing is detected, false otherwise

  //float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  if(altitude <= (GROUND_ALTITUDE + 10)){
    return true;
  }
  else{ 
    return false;
    }
}


void sendStatus(){
  outgoing_message = {device_name, STATUSMESSAGE, Flight_STATUS, 0, 0, 0, 0, 0, 0, 0}; 
  updateSerialMonitor();
  // // Send message via ESP-NOW
  // esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing_message, sizeof(outgoing_message));
   
  // if (result == ESP_OK) {
  //   Serial.print(device_name);
  //   Serial.println(": Message sent with success");
  //   //printMessageStruct("OUTGOING", outgoing_message);
  // }
  // else {
  //   Serial.print(device_name);
  //   Serial.println(": Error sending the message");
  // }

}

void sendSensorData(){
  // float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA); // in m

  //sensors_event_t event;
  //bno.getEvent(&event);
  // event.orientation.x
  // event.orientation.y
  // event.orientation.z

  // event.acceleration.x
  // event.acceleration.y
  // event.acceleration.z

  // event.gyro.x
  // event.gyro.y
  // event.gyro.z

  // event.magnetic.x
  // event.magnetic.y
  // event.magnetic.z

  //outgoing_message = {device_name, DATAMESSAGE, "DATA", bme.readAltitude(SEALEVELPRESSURE_HPA), event.acceleration.x, event.acceleration.y, event.acceleration.z, event.gyro.x, event.gyro.y, event.gyro.z}; 
    outgoing_message = {device_name, DATAMESSAGE, "DATA", altitude, random(-100, 100), random(-100, 100), random(-100, 100), random(0, 360), random(0, 360), random(0, 360)}; 
    updateSerialMonitor();
  // Send message via ESP-NOW
  // esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing_message, sizeof(outgoing_message));
   
  // if (result == ESP_OK) {
  //   Serial.print(device_name);
  //   Serial.println(": Message sent with success");
  //   //printMessageStruct("OUTGOING", outgoing_message);
  // }
  // else {
  //   Serial.print(device_name);
  //   Serial.println(": Error sending the message");
  // }
}

void updateSerialMonitor(){
  
  if(outgoing_message.message_type == STATUSMESSAGE){
    Serial.print(outgoing_message.sender_value);
    Serial.print(":");
    Serial.println(outgoing_message.status_str);
    }
  
  else if(outgoing_message.message_type == DATAMESSAGE){
    Serial.print(outgoing_message.sender_value);
    Serial.println(":");

  Serial.print("Altitude: ");
  Serial.println(outgoing_message.pilot_altitude, 4);

  Serial.print("Acceleration X: ");
  Serial.print(outgoing_message.pilot_acceleration_x, 4);
  Serial.print("\tAcceraltion Y: ");
  Serial.print(outgoing_message.pilot_acceleration_y, 4);
  Serial.print("\tAcceleration Z: ");
  Serial.println(outgoing_message.pilot_acceleration_z, 4);

  Serial.print("Gyro X: ");
  Serial.print(outgoing_message.pilot_gyro_x, 4);
  Serial.print("\t Gyro Y: ");
  Serial.print(outgoing_message.pilot_gyro_y, 4);
  Serial.print("\t Gyro Z: ");
  Serial.println(outgoing_message.pilot_gyro_z, 4);
    
    }
  else {
    Serial.print(outgoing_message.sender_value);
    Serial.print(": Hello, ");
    Serial.println(device_name);
   }
  
}