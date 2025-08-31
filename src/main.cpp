#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "Thegill.h"
#include <ThegillComms.h>

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
const char* password = "AASCE321#";

float joystick1X = 0.0, joystick1Y = 0.0;
float joystick2X = 0.0, joystick2Y = 0.0;
float sliderValue = 0.0;
float temperature = 25.0;
float batteryVoltage = 3.7;
int s1Value = 0, s2Value = 0, s3Value = 0, s4Value = 0;
int m1Value = 0, m2Value = 0;  // For storing values (Arm Motors)

Servo basketservo;
bool app_status;
String handShakeCode;
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
  Serial.print("Connected to:");
  Serial.println(WiFi.gatewayIP());
  Serial.print("As:");
  Serial.println(WiFi.localIP());
  // WiFi.softAPConfig(local_IP, gateway, subnet);
  // WiFi.softAP(ssid,password,2,0,4);
  
  startUDPServer();


  basketservo.attach(18);
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
      app_status=0;
      udp.stop();
      udp.begin(udpPort);
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
      Serial.print("Connected to:");
      Serial.println(WiFi.gatewayIP());
      Serial.print("As:");
      Serial.println(WiFi.localIP());
      Serial.println("Now attempting to connect to the app...");
      // do
      // {
      //   Serial.println("Sending UDP IP...");
      //   udp.beginPacket(WiFi.gatewayIP(),4210);
      //   handShakeCode=String("ESP:")+WiFi.localIP();
      //   udp.write((const uint8_t*)handShakeCode.c_str(),handShakeCode.length());
      //   udp.endPacket();
      //   delay(100);
      //   receiveUDPPackets();
      //   if(WiFi.status()!=WL_CONNECTED)
      //   {
      //     break;
      //   }
      // }while (app_status==0);
      Serial.println("Connection Re-Established!");
    }
    receiveUDPPackets();
    basketservo.write(map(basketpose,0,100,0,180));
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