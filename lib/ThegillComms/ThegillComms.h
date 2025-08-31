#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>

struct control_frame {
    uint8_t flag_set = 0;
    float motor_bias = 0;
    float motor_power = 0;
    float arm_rotation_speed = 0;
    uint8_t arm_servo_pose = 90;
    uint8_t elbow_servo_pose = 90;
    float arm_extension_speed = 0;
    uint8_t pitch_servo_pose = 90;
    uint8_t yaw_servo_pose = 90;
    uint8_t grip_servo_pose = 0;
};

extern control_frame pico_frame;
extern unsigned long lastUDPPacketTime;
extern const unsigned int udpPort;
extern int basketpose;
extern WiFiUDP udp;
extern float temperature;
extern float batteryVoltage;

void startUDPServer();
void picoPush();
void receiveUDPPackets();
void sendResponseToApp(IPAddress deviceIP);
