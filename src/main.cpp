#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include "Thegill.h"

//SDA 21
//SCL 22
//Motor values are a signed byte 

// byte pico_frame[13]={0,0,0,0,0,0,0,0,0,0,0,0,0}; //dummy data

struct control_frame
{
  public:
    uint8_t flag_set=0x11;

    int left_top_motor_target=0;
    int left_bot_motor_target=0;
    int right_top_motor_target=0;
    int right_bot_motor_target=0;
    float ease_value=0; //thinking of chaging the name of the interpolation factor/coeff to just "ease value"

    int arm_rotation_speed=0;
    uint8_t arm_servo_pose=0;
    uint8_t elbow_servo_pose=0;
    int arm_extension_speed=0;
    uint8_t pitch_servo_pose=0;
    uint8_t yaw_servo_pose=0;
    uint8_t grip_servo_pose=0;
};
control_frame pico_frame;

unsigned char transmission_frame[sizeof(pico_frame)];

void picoPush(/*flags,LTmot,LBmot,RTmot,RBmot,Intrp,ArmRotation,ArmPose,ElbowPose,Extension,Pitch,Yaw,Grip*/) //program is literally "byte" banging the i2c coms, could use some optimizations by increasing buffer size or putting half buffers here ig
{
  int buffer_partition_integral_count = 0;
  memcpy(&transmission_frame,&pico_frame,sizeof(pico_frame));
  for(int i=0;i<sizeof(transmission_frame);i++)
  {
    buffer_partition_integral_count++;
    if(buffer_partition_integral_count<=1)
    {
      Wire.beginTransmission(0x17);
    }
    Wire.write(transmission_frame[i]);
    if(buffer_partition_integral_count>30)
    {buffer_partition_integral_count=0;Wire.endTransmission();}
  }
  // Wire.beginTransmission(0x17);
  // Wire.write((uint8_t*)&transmission_frame,sizeof(transmission_frame));
  Wire.endTransmission();
}



void setup() {
  setCpuFrequencyMhz(240);
  //Init i2c
  Wire.setBufferSize(sizeof(pico_frame));
  Wire.setClock(400*1000);
  Wire.begin();
  
  WiFi.begin("eAse","Akila10101947##@");

  //Init UART (Debug)
  Serial.begin(9600);
  Serial.println("\n Bonjour :)");
  picoPush();
  pico_frame.flag_set=0xFF;
  pico_frame.left_top_motor_target=15;
  pico_frame.left_bot_motor_target=-15;
  pico_frame.right_top_motor_target=20;
  pico_frame.right_bot_motor_target=-11112;
  pico_frame.ease_value=-0.55;
  pico_frame.arm_rotation_speed=55;
  pico_frame.arm_servo_pose=90;
  pico_frame.elbow_servo_pose=45;
  pico_frame.arm_extension_speed=-55;
  pico_frame.pitch_servo_pose=120;
  pico_frame.yaw_servo_pose=5;
  pico_frame.grip_servo_pose=180;
  delay(4000);
} 

void loop() 
{
    Serial.println("Pushing packet");
    pico_frame.left_bot_motor_target++;
    picoPush(); 
}