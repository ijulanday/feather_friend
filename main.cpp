#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ardupilotmega/mavlink.h>

/* taskz handlez */
TaskHandle_t TaskUART;
TaskHandle_t TaskUDP;

/* global network info definitions */
const char* ssid = "Hydro_Corp";
const char* password =  "warpedo1";
IPAddress local_IP(10, 1, 4, 20); ///TODO: use dynamic IP in production
IPAddress gateway(10, 1, 0, 1); ///TODO: change gateway & subnet info to match production network
IPAddress subnet(255, 255, 240, 0);

/* host connection information */
WiFiUDP udp;
const uint16_t remoteport = 14550;
const uint16_t localport = 14551;
IPAddress gshost(10, 1, 5, 123); ///TODO: try to dynamically look for host IP ???


/* definition of mavlink serial port (Serial1 on ESP32 feather) */
HardwareSerial SerialMAV = Serial1;

/* used in main loop to track first message received */
bool firstReceived = false;

/* other globals */
uint8_t fcSysID = 1;
WiFiClient client;
uint8_t missionCount = 0;
bool receivedCount = false;
const uint16_t sensorMsgFreqHz = 5; 

/* send mavlink stuff to GS if received from FC */
void TaskUARTFun(void * pvParameters) {
  Serial.print("Started uart activity on core "); Serial.println(xPortGetCoreID());
  mavlink_message_t msg;
  mavlink_status_t status;
  uint8_t c;
  uint16_t len;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  for(;;)
  {
    udp.beginPacket(gshost, remoteport);
    while (SerialMAV.available())
    {
      c = SerialMAV.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
      {
        switch (msg.msgid)
        {
          /* add cases here for special cases / debugging (as below) */
          case MAVLINK_MSG_ID_MISSION_ACK:
          {
            /* resume status messages when mission upload is complete */
            mavlink_message_t startmsg;
            mavlink_msg_request_data_stream_pack(255, MAV_COMP_ID_ALL, &startmsg, fcSysID, MAV_COMP_ID_ALL, MAV_DATA_STREAM_ALL, sensorMsgFreqHz, 1);
            uint16_t startlen = mavlink_msg_to_send_buffer(buf, &startmsg);
            SerialMAV.write(buf, startlen);
          } 
          default:
          {
            len = mavlink_msg_to_send_buffer(buf, &msg);
            udp.write(buf, len);
          }
        }
        break;
      }
    }
    udp.endPacket();
    vTaskDelay(1);
  }
}

/* receive udp traffic from GS and send to FC */
void TaskUDPFun(void * pvParameters) {
  Serial.print("Started udp activity on core "); Serial.println(xPortGetCoreID());
  mavlink_message_t msg;
  mavlink_status_t status;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint8_t c;
  for(;;)
  {
    int packetSize = udp.parsePacket();
    if (packetSize) 
    {
      while (udp.available())
      {
        c = udp.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
        {
          switch (msg.msgid) 
          {
            /* add cases here for special cases / debugging (as below) */
            case MAVLINK_MSG_ID_MISSION_COUNT:
            {
              /* stop all the status updates!! (so that the exchange happens better) */
              mavlink_message_t stopmsg;
              mavlink_msg_request_data_stream_pack(255, MAV_COMP_ID_ALL, &stopmsg, fcSysID, MAV_COMP_ID_ALL, MAV_DATA_STREAM_ALL, sensorMsgFreqHz, 0);
              uint16_t stoplen = mavlink_msg_to_send_buffer(buf, &stopmsg);
              SerialMAV.write(buf, stoplen);
            } /* no break cuz we still wanna write the original message to the FC */
            default:
            {
              uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
              SerialMAV.write(buf, len);
            }
          }
          break;
        }
      }
    }
    vTaskDelay(1);
  }
}

void setup() {

  /* debug serial */
  Serial.begin(9600);
  while (!Serial) {}

  /* mavlink serial */
  SerialMAV.begin(115200);
  while (!SerialMAV) {}
  firstReceived = false;

  /* send req for data only after first message (heartbeat usually) is received */
  mavlink_message_t msg;
  mavlink_status_t status;
  bool firstReceived = false;
  while (firstReceived == false)
  {
    Serial.print("waiting for heartbeat from FC...\r");
    while (SerialMAV.available())
    {
      uint8_t c = SerialMAV.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
      {
        fcSysID = msg.sysid;
        Serial.print("\n got fc heartbeat: sysID "); Serial.println(fcSysID);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_request_data_stream_pack(255, MAV_COMP_ID_ALL, &msg, fcSysID, MAV_COMP_ID_ALL, MAV_DATA_STREAM_ALL, sensorMsgFreqHz, 1);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        SerialMAV.write(buf, len);
        firstReceived = true; 
        break;
      }
    }
  }
  
  /* configure wifi connection with a static ip */
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA failed to configure (whatever that means!)");
  }

  /* connect to wifi network provided ssid and password */
  WiFi.begin(ssid, password);
  Serial.print("Connecting to wifi");
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\n wifi connected! \n");

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    TaskUARTFun,   /* Task function. */
                    "TaskUART",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TaskUART,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(250); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    TaskUDPFun,   /* Task function. */
                    "TaskUDP",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TaskUDP,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
  delay(250); 
}

void loop() {
  vTaskDelete(NULL);
}