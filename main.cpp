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
IPAddress gshost(10, 1, 5, 123); ///TODO: try to dynamically look for host IP

/* definition of mavlink serial port (Serial1 on ESP32 feather) */
HardwareSerial SerialMAV = Serial1;

/* used in main loop to track first message received */
bool firstReceived = false;

/* other globals */
uint8_t fcSysID = 1;
WiFiClient client;
uint8_t missionCount = 0;
bool receivedCount = false;

/**
 * receives mavlink messages to a buffer from serial. updates args passed as pointers
 * 
 * @param   msg     pointer to mavlink message
 * @param   status  pointer to mavlink status
 * @returns         boolean for message receive (1 if successful, 0 if not)
 */
bool MavlinkReceiveUART(mavlink_message_t* msg, mavlink_status_t* status) {
  while (SerialMAV.available())
  {
    uint8_t c = SerialMAV.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, msg, status))
    {
      /* ... */
      return true;
    }
  }

  return false;
}

/**
 * receives mavlink messages to a buffer from udp. updates args passed as pointers
 * 
 * @param   msg     pointer to mavlink message
 * @param   status  pointer to mavlink status
 * @returns         boolean for message receive (1 if successful, 0 if not)
 */
bool MavlinkReceiveUDP(mavlink_message_t* msg, mavlink_status_t* status) {
  while (SerialMAV.available())
  {
    uint8_t c = SerialMAV.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, msg, status))
    {
      /* ... */
      return true;
    }
  }

  return false;
}

/* send mavlink stuff to GS if received from FC */
void TaskUARTFun(void * pvParameters) {
  Serial.print("Started uart activity on core "); Serial.println(xPortGetCoreID());
  mavlink_message_t msg;
  mavlink_status_t status;
  uint8_t c;
  uint16_t len;
  for(;;)
  {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    udp.beginPacket(gshost, remoteport);
    while (SerialMAV.available())
    {
      c = SerialMAV.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
      {
        switch (msg.msgid)
        {
          case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
          {
            len = mavlink_msg_to_send_buffer(buf, &msg);
            Serial.println("also getting mission request int for some reason");
          } break;
          case MAVLINK_MSG_ID_MISSION_REQUEST: /* this is deprecated -- need to transform into MISSION_REQUEST_INT */
          {
            udp.flush();
            mavlink_mission_request_t req;  /* old, stinky format */
            mavlink_msg_mission_request_decode(&msg, &req);
            // mavlink_msg_mission_request_int_pack(fcSysID, MAV_COMP_ID_ALL, &msg, req.target_system, MAV_COMP_ID_ALL, req.seq, req.mission_type); /* new, epic format for cool ppl */
            len = mavlink_msg_to_send_buffer(buf, &msg);

            Serial.println("mission request on uart");
            Serial.print("- targsys: "); Serial.println(req.target_system);
            Serial.print("- seq: "); Serial.println(req.seq);
            Serial.print("- type: "); Serial.println(req.mission_type);

            /* resume udp and suspend here */
            // vTaskResume(TaskUDP);
            // vTaskSuspend(NULL);
          } break;
          case MAVLINK_MSG_ID_MISSION_ACK:
          {
            len = mavlink_msg_to_send_buffer(buf, &msg);

            /* start streaming data again */
            mavlink_message_t reqDataMsg;
            uint8_t reqDataBuf[MAVLINK_MAX_PACKET_LEN];
            mavlink_msg_request_data_stream_pack(255, MAV_COMP_ID_ALL, &reqDataMsg, fcSysID, MAV_COMP_ID_ALL, MAV_DATA_STREAM_ALL, 5, 1);
            uint16_t reqDataLen = mavlink_msg_to_send_buffer(reqDataBuf, &reqDataMsg);
            SerialMAV.write(reqDataBuf, reqDataLen);

            Serial.println("mission ack sent (omg?!) ");

            /* resume UDP task */
            // vTaskResume(TaskUDP);
            // receivedCount = false;
          } break;
          default:
          {
            len = mavlink_msg_to_send_buffer(buf, &msg);
          } break;
        }
        udp.write(buf, len);
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
  for(;;)
  {
    int packetSize = udp.parsePacket();
    if (packetSize) 
    {
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      while (udp.available())
      {
        uint8_t c = udp.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
        {
          udp.flush();
          switch (msg.msgid) 
          {
            case MAVLINK_MSG_ID_MISSION_COUNT:
            {
              if (receivedCount == true) break;
              uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
              SerialMAV.write(buf, len);
              mavlink_mission_count_t count;
              mavlink_msg_mission_count_decode(&msg, &count);

              /* stop with the data flood while trying to do the mission bs */
              mavlink_msg_request_data_stream_pack(255, MAV_COMP_ID_ALL, &msg, fcSysID, MAV_COMP_ID_ALL, MAV_DATA_STREAM_ALL, 5, 0);
              len = mavlink_msg_to_send_buffer(buf, &msg);
              SerialMAV.write(buf, len);

              Serial.println("mission count on udp: ");
              Serial.print("- type: "); Serial.println(count.mission_type);
              Serial.print("- count: "); Serial.println(count.count);

              /* suspend & disallow coming here again */
              // udp.flush();
              // vTaskSuspend(NULL);
              // receivedCount = true;
            } break;
            case MAVLINK_MSG_ID_MISSION_ITEM_INT:
            {
              uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
              SerialMAV.write(buf, len);
              mavlink_mission_item_int_t item;
              mavlink_msg_mission_item_int_decode(&msg, &item);

              Serial.println("received mission item (int) on udp");
              Serial.print("- seq no: "); Serial.println(item.seq);

              /* resume UART and suspend */
              // vTaskResume(TaskUART);
              // vTaskSuspend(NULL);
            } break;
            case MAVLINK_MSG_ID_MISSION_ITEM:
            {
              uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
              SerialMAV.write(buf, len);
              mavlink_mission_item_t item;
              mavlink_msg_mission_item_decode(&msg, &item);

              Serial.println("received mission item on udp");
              Serial.print("- seq no: "); Serial.println(item.seq);

              /* resume UART and suspend */
              // vTaskResume(TaskUART);
              // vTaskSuspend(NULL);
            } break;
            default:
            {
              uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
              SerialMAV.write(buf, len);
            } break;
          }

          break; /* get OUT of dis loop if u done reading */
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
  while (firstReceived == false)
  {
    Serial.print("waiting for heartbeat from FC...\r");
    if (MavlinkReceiveUART(&msg, &status))
    {
      fcSysID = msg.sysid;
      Serial.print("got fc heartbeat: sysID "); Serial.println(fcSysID);
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      mavlink_msg_request_data_stream_pack(255, MAV_COMP_ID_ALL, &msg, fcSysID, MAV_COMP_ID_ALL, MAV_DATA_STREAM_ALL, 5, 1);
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      SerialMAV.write(buf, len);
      firstReceived = true; 
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