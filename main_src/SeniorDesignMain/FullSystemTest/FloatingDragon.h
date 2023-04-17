#ifndef FLOATINGDRAGON
#define FLOATINGDRAGON


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



typedef struct struct_timer {
    unsigned long start_time; //time that the timer started
    unsigned long end_time;  //time that the timer should timeout
    bool timeout; //indicates whether the timer is out of time or not
} struct_timer;


enum timerTypes{
  ONESECTIMER,
  PARACHUTETIMER,
  MOTORTIMER
};

#endif