#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include "Thegill.h"

//SDA 21
//SCL 22
//Motor values are a signed byte 

// byte pico_frame[13]={0,0,0,0,0,0,0,0,0,0,0,0,0}; //dummy data

IPAddress local_IP(4,4,4,100);
IPAddress gateway(4,4,4,100);
IPAddress subnet(255,255,255,0);
const char* ssid     = "Thegill Soul";
const char* password = "ASCE321#";

void startUDPServer();
void receiveUDPPackets();
void testMotors(uint8_t clientID);
void sendResponseToApp(IPAddress deviceIP);

float joystick1X = 0.0, joystick1Y = 0.0;
float joystick2X = 0.0, joystick2Y = 0.0;
float sliderValue = 0.0;
float temperature = 25.0;
float batteryVoltage = 3.7;
int s1Value = 0, s2Value = 0, s3Value = 0, s4Value = 0;  
int m1Value = 0, m2Value = 0;  // For storing values (Arm Motors)

unsigned long lastUDPPacketTime = 0;
unsigned long udpTimeout = 5000;  
const unsigned int udpPort = 4210;  

WiFiUDP udp;

char incomingPacket[255];
String receivedMessage;

struct control_frame
{
  public:
    uint8_t flag_set=0x00;

    int left_top_motor_target=0; //Theoretically the control frame should only consist of the right and the left speeds no ? but still, I will be sending the entire control frame and if this gets problematic it'll have to be sorted
    int left_bot_motor_target=0;
    int right_top_motor_target=0;
    int right_bot_motor_target=0;
    float ease_value=0; //thinking of chaging the name of the interpolation factor/coeff to just "ease value"

    int arm_rotation_speed=0;
    uint8_t arm_servo_pose=0;
    int elbow_servo_pose=0;
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
    if(buffer_partition_integral_count>31)
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
  
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid,password,2,0,4);
  
  startUDPServer();

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

    receiveUDPPackets();

    // Only call sendResponseToApp if a UDP packet was received
    if (lastUDPPacketTime > 0) 
    { // Check if any UDP packet was received
    IPAddress senderIP = udp.remoteIP(); // Get the IP address of the sender
    sendResponseToApp(senderIP); // Send response to the app
    }

    // Send temperature and battery status every 5 seconds if no UDP packet is received
    if (millis() - lastUDPPacketTime > udpTimeout) {
    //sendTemperatureAndBattery();
    }

    picoPush(); 
}