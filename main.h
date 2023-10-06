#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ardupilotmega/mavlink.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>

// #define   STATIC_IP   
// #define   RX  16
// #define   TX  17

/* taskz handlez */
TaskHandle_t TaskUART;
TaskHandle_t TaskUDP;

/* global network info definitions */
const char* ssid = "tidepool";
const char* password =  "gone fishing";
IPAddress gateway(10, 1, 0, 1); 
IPAddress subnet;
IPAddress localhost;

/* host connection information */
WiFiUDP udp;
WiFiClient client;
const uint16_t remoteport = 14550;
IPAddress gshost(10, 1, 0, 69); ///TODO: try to dynamically look for host IP ???


/* definition of mavlink serial port (Serial1 on ESP32 feather) */
// SoftwareSerial SerialMAV(RX, TX);
HardwareSerial SerialMAV = Serial1;

/* used in main loop to track first message received */
bool firstReceived = false;

/* other globals */
uint8_t fcSysID = 1;
uint8_t missionCount = 0;
bool receivedCount = false;
const uint16_t sensorMsgFreqHz = 5; 
Adafruit_NeoPixel pixels(3, 25, NEO_GRB + NEO_KHZ800);
int subocts = 0;
int hostocts = 0;
mavlink_message_t msg;
mavlink_status_t status;
uint8_t c;
uint16_t len;

void incrementIPAddress(IPAddress &ip, IPAddress subnet, int fullSubOcts);
void TaskUDPFun(void * pvParameters);
void TaskUARTFun(void * pvParameters);