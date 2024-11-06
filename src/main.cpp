#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include "Thegill.h"

//SDA 21
//SCL 22
//Motor values are a signed byte 

// byte pico_frame[13]={0,0,0,0,0,0,0,0,0,0,0,0,0}; //dummy data

//J1 = forwardBackwards (J1:f) (-1.0+1.0)
//J2 = left right (J2:f) (-1.0+1.0)
//Slider1/2/3:f
//Button1/2 Pressed
//S0->s6:f(s1+s6 -1.0+1.0) (rest = 0 180); 

IPAddress local_IP(4,4,4,100);
IPAddress gateway(4,4,4,100);
IPAddress subnet(255,255,255,0);
const char* ssid     = "Thegill Soul";
const char* password = "ASCE321#";

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

struct control_frame
{
  public:
    uint8_t flag_set=0;

    float motor_bias=0;
    float motor_power=0;

    float arm_rotation_speed=0;
    uint8_t arm_servo_pose=90;
    uint8_t elbow_servo_pose=90;
    float arm_extension_speed=0;
    uint8_t pitch_servo_pose=90;
    uint8_t yaw_servo_pose=90;
    uint8_t grip_servo_pose=0;

};
control_frame pico_frame;

char incomingPacket[255];
String receivedMessage;

WiFiUDP udp;
String callback;
void startUDPServer() {
  udp.begin(udpPort);
  Serial.printf("UDP server started at IP: %s, port: %d\n", WiFi.localIP().toString().c_str(), udpPort);
}
bool i2c_reset_flag;

unsigned char transmission_frame[sizeof(pico_frame)];

void picoPush() //program is literally "byte" banging the i2c coms, could use some optimizations by increasing buffer size or putting half buffers here ig
{
  if(sizeof(pico_frame)>32){
  memcpy(&transmission_frame,&pico_frame,sizeof(pico_frame));
  int buffer_partition_integral_count = 0;
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
  }else
  {
    Wire.beginTransmission(0x17);
    Wire.write((uint8_t*)&pico_frame,sizeof(pico_frame));
    Wire.endTransmission();
  }
}


void receiveUDPPackets() {
   int packetSize = udp.parsePacket();
  if (packetSize) {
    lastUDPPacketTime = millis();  // Reset the timeout
     int len = udp.read(incomingPacket, 255);
    if (len > 0) incomingPacket[len] = 0;
    receivedMessage = String(incomingPacket);
    
    switch (receivedMessage.charAt(0))
    {
    case 'J':
      switch (receivedMessage.charAt(1))
      {
      case '1':
        pico_frame.motor_power=receivedMessage.substring(3).toFloat();
        break;
      case '2':
        pico_frame.motor_bias=receivedMessage.substring(3).toFloat();
        break;
      }
      break;
    
    case 'S':
      switch (receivedMessage.charAt(1))
      { //S0->s6:f(s1+s6 -1.0+1.0) (rest = 0 180); 
      case '0':
        pico_frame.arm_servo_pose=receivedMessage.substring(3).toInt();
        break;
      case '1':
        pico_frame.arm_rotation_speed=receivedMessage.substring(3).toFloat();
        break;
      case '2':
        pico_frame.elbow_servo_pose=receivedMessage.substring(3).toInt();
        break;
      case '3':
        pico_frame.pitch_servo_pose=receivedMessage.substring(3).toInt();
        break;
      case '4':
        pico_frame.yaw_servo_pose=receivedMessage.substring(3).toInt();
        break;
      case '5':
        pico_frame.grip_servo_pose=receivedMessage.substring(3).toInt();
        break;
      case '6':
        pico_frame.arm_extension_speed=receivedMessage.substring(3).toFloat();
        break;
      }
    break;
    
    case 'B':
      switch (receivedMessage.charAt(7))
      {
        case '1': 
        if(pico_frame.flag_set==B00000000)
        {
         pico_frame.flag_set=B00000001; 
        }else
        {
          pico_frame.flag_set=B00000000;
        }
        break;
        case '2': 
        Serial.println("Requesting Reset");
        Wire.requestFrom(0x17,sizeof("Flushed!"));
        do
        {
          callback.concat((char)Wire.read());
        }while (!callback.equals("Flushed!"));
        Serial.println(callback);
        callback="";
        break;
      }
    break;
    }
    //Serial.printf("Received UDP packet: %s\n", incomingPacket);
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
    //IPAddress senderIP = udp.remoteIP();
    //Serial.printf("Received UDP packet from IP: %s\n", senderIP.toString().c_str());
    picoPush();
    //handleJoystickOrSlider(receivedMessage);
  }
}
void sendResponseToApp(IPAddress deviceIP) {
  udp.beginPacket(deviceIP, udpPort); // Use the IP from the sender
  
  // Convert the message to a char array
  String message = "Temperature: " + String(temperature) + ", Battery: " + String(batteryVoltage);
  udp.write((const uint8_t*)message.c_str(), message.length()); // Explicitly cast to uint8_t*
  
  udp.endPacket();
}



  bool blinker;

void setup() {
  setCpuFrequencyMhz(240);
  //Init i2c
  Wire.setClock(400*1000);
  Wire.begin();

  //Wifi Init: 

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    blinker=!blinker;
    digitalWrite(LED_BUILTIN,blinker);
    delay(500);
    Serial.println("Connecting to WiFi...");
  }

  // WiFi.softAPConfig(local_IP, gateway, subnet);
  // WiFi.softAP(ssid,password,2,0,4);
  
  startUDPServer();

  //Init Serial (Debug)
  pinMode(LED_BUILTIN,OUTPUT);
  Serial.begin(9600);
  Serial.println("\n Bonjour :)");
} 

unsigned long millisdiff;

void loop() 
{
    if(WiFi.status()!= WL_CONNECTED)
    { 
      pico_frame.arm_extension_speed=0;
      pico_frame.arm_rotation_speed=0;
      pico_frame.motor_power=0;
      pico_frame.motor_bias=0;
      picoPush();
      WiFi.disconnect();
      Serial.println("Disconnected, attempting reconnect, probing:");
      do{
      WiFi.begin(ssid, password);
      blinker=!blinker;
      digitalWrite(LED_BUILTIN,blinker);
      delay(500);
      Serial.println("Connecting to WiFi...");
      } while (WiFi.status() != WL_CONNECTED);
    }
    receiveUDPPackets();

    // Only call sendResponseToApp if a UDP packet was received
    if (millis()-lastUDPPacketTime > 600) 
    { // Check if any UDP packet was received
    //static IPAddress senderIP = udp.remoteIP(); // Get the IP address of the sender
    //sendResponseToApp(senderIP); // Send response to the app
    pico_frame.arm_extension_speed=0;
    pico_frame.arm_rotation_speed=0;
    pico_frame.motor_power=0;
    pico_frame.motor_bias=0;
    picoPush();
    }//so we don't lose control over the robot yk

    if(millis()-millisdiff>=500){
    digitalWrite(LED_BUILTIN,0);
    // Serial.printf("The packets fetched:\nflags:%x\nMotor Power:%f\nMotor Bias:%f\nRot Speed:%f\nArm pose:%i\nElbow pose:%i\nExtns Speed:%f\nPitch pose:%i\nYaw pose:%i\nGrip pose:%i\n\n\n",
    // pico_frame.flag_set,pico_frame.motor_power,pico_frame.motor_bias,pico_frame.arm_rotation_speed,pico_frame.arm_servo_pose,pico_frame.elbow_servo_pose,
    // pico_frame.arm_extension_speed,pico_frame.pitch_servo_pose,pico_frame.yaw_servo_pose,pico_frame.grip_servo_pose);
    millisdiff=millis();
    picoPush();
    }
    //picoPush(); 
}