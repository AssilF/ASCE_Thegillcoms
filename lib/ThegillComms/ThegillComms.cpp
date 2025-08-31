#include "ThegillComms.h"

control_frame pico_frame;
unsigned long lastUDPPacketTime = 0;
const unsigned int udpPort = 4210;
int basketpose;
WiFiUDP udp;

static char incomingPacket[255];
static String receivedMessage;
static String callback;

void startUDPServer() {
  udp.begin(udpPort);
  Serial.printf("UDP server started at IP: %s, port: %d\n", WiFi.localIP().toString().c_str(), udpPort);
}

void picoPush() {
  unsigned char transmission_frame[sizeof(pico_frame)];
  if (sizeof(pico_frame) > 32) {
    memcpy(&transmission_frame, &pico_frame, sizeof(pico_frame));
    int buffer_partition_integral_count = 0;
    for (int i = 0; i < sizeof(transmission_frame); i++) {
      buffer_partition_integral_count++;
      if (buffer_partition_integral_count <= 1) {
        Wire.beginTransmission(0x17);
      }
      Wire.write(transmission_frame[i]);
      if (buffer_partition_integral_count > 31) {
        buffer_partition_integral_count = 0;
        Wire.endTransmission();
      }
    }
    Wire.endTransmission();
  } else {
    Wire.beginTransmission(0x17);
    Wire.write((uint8_t*)&pico_frame, sizeof(pico_frame));
    Wire.endTransmission();
  }
}

void receiveUDPPackets() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    lastUDPPacketTime = millis();
    int len = udp.read(incomingPacket, 255);
    if (len > 0) incomingPacket[len] = 0;
    receivedMessage = String(incomingPacket);
    switch (receivedMessage.charAt(0)) {
      case 'J':
        switch (receivedMessage.charAt(1)) {
          case '1':
            pico_frame.motor_power = receivedMessage.substring(3).toFloat();
            break;
          case '2':
            pico_frame.motor_bias = receivedMessage.substring(3).toFloat();
            break;
        }
        break;
      case 'S':
        switch (receivedMessage.charAt(1)) {
          case '0':
            pico_frame.arm_servo_pose = receivedMessage.substring(3).toInt();
            break;
          case '1':
            pico_frame.arm_extension_speed = receivedMessage.substring(3).toFloat();
            break;
          case '2':
            pico_frame.pitch_servo_pose = receivedMessage.substring(3).toInt();
            break;
          case '3':
            pico_frame.elbow_servo_pose = receivedMessage.substring(3).toInt();
            break;
          case '4':
            pico_frame.grip_servo_pose = receivedMessage.substring(3).toInt();
            break;
          case '5':
            pico_frame.yaw_servo_pose = receivedMessage.substring(3).toInt();
            break;
          case '6':
            pico_frame.arm_rotation_speed = receivedMessage.substring(3).toFloat();
            break;
          case 'l':
            switch (receivedMessage.charAt(6)) {
              case '1':
                basketpose = receivedMessage.substring(8).toFloat();
                break;
            }
            break;
        }
        break;
      case 'B':
        switch (receivedMessage.charAt(7)) {
          case '1':
            if (pico_frame.flag_set == B00000000) {
              pico_frame.flag_set = B00000001;
            } else {
              pico_frame.flag_set = B00000000;
            }
            break;
          case '2':
            Serial.println("Requesting Reset");
            Wire.requestFrom(0x17, sizeof("Flushed!"));
            do {
              callback.concat((char)Wire.read());
            } while (!callback.equals("Flushed!"));
            Serial.println(callback);
            callback = "";
            break;
        }
        break;
    }
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    picoPush();
  }
}

void sendResponseToApp(IPAddress deviceIP) {
  udp.beginPacket(deviceIP, udpPort);
  String message = "Temperature: " + String(temperature) + ", Battery: " + String(batteryVoltage);
  udp.write((const uint8_t*)message.c_str(), message.length());
  udp.endPacket();
}

