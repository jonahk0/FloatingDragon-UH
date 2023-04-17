#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0xEC, 0x94, 0xCB, 0x6D, 0xD5, 0x34};
//PILOT: EC:94:CB:6D:D5:34
//GROUND: EC:94:CB:6D:02:00


// Label Local Device
String device_name = "GROUND";

// Create a struct_message to hold outgoing message

enum broadcast_message_types{
  DATAMESSAGE,
  STATUSMESSAGE
};

//Must match the receiver structure
typedef struct struct_message {
    String sender_value;

    int message_type;

    String status_str;

    float pilot_altitude;

    float pilot_acceleration_x;
    float pilot_acceleration_y;
    float pilot_acceleration_z;

    float pilot_gyro_x;
    float pilot_gyro_y;
    float pilot_gyro_z;

    //unsigned long program_time;

} struct_message;

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


int message_type;
String status_str, sender_name;
float pilot_altitude, pilot_accel_x, pilot_gyro_x, pilot_accel_y, pilot_gyro_y, pilot_accel_z, pilot_gyro_z;


// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming_message, incomingData, sizeof(incoming_message));
  Serial.print("Bytes received: ");
  Serial.println(len);
  
  sender_name = incoming_message.sender_value;
  message_type = incoming_message.message_type;

  status_str = incoming_message.status_str;

  pilot_altitude = incoming_message.pilot_altitude;

  pilot_accel_x = incoming_message.pilot_acceleration_x;
  pilot_accel_y = incoming_message.pilot_acceleration_y;
  pilot_accel_z = incoming_message.pilot_acceleration_z;

  pilot_gyro_x = incoming_message.pilot_gyro_x;
  pilot_gyro_y = incoming_message.pilot_gyro_y;
  pilot_gyro_z = incoming_message.pilot_gyro_z;

  //printMessageStruct("INCOMING", incoming_message);

  updateSerialMonitor();

}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
    
  // Set device as a Wi-Fi Station
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
 
void loop() {

}

void updateSerialMonitor(){
  
  if(message_type == STATUSMESSAGE){
    Serial.print(sender_name);
    Serial.print(":");
    Serial.println(status_str);
    }
  
  else if(message_type == DATAMESSAGE){
    Serial.print(sender_name);
    Serial.println(":");

  Serial.print("Altitude: ");
  Serial.println(pilot_altitude, 4);

  Serial.print("Acceleration X: ");
  Serial.print(pilot_accel_x, 4);
  Serial.print("\tAcceraltion Y: ");
  Serial.print(pilot_accel_y, 4);
  Serial.print("\tAcceleration Z: ");
  Serial.println(pilot_accel_z, 4);

  Serial.print("Gyro X: ");
  Serial.print(pilot_gyro_x, 4);
  Serial.print("\t Gyro Y: ");
  Serial.print(pilot_gyro_y, 4);
  Serial.print("\t Gyro Z: ");
  Serial.println(pilot_gyro_z, 4);
    
    }
  else {
    Serial.print(sender_name);
    Serial.print(": Hello, ");
    Serial.println(device_name);
   }
  
}



