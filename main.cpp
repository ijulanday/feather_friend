#include "main.h"


void setup() 
{
  /* pixel stuff */
  // pixels.begin();
  // pixels.clear();
  // pixels.setPixelColor(2, 255, 255, 255);
  // pixels.setBrightness(200);
  // pixels.show();

  /* debug serial */
  Serial.begin(9600);
  delay(1000);

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
    Serial.print("waiting for heartbeat from FC...\n");
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
  
  /* connect to wifi network provided ssid and password */
  WiFi.begin(ssid, password);
  Serial.print("Connecting to wifi");
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  subnet = WiFi.subnetMask();
  gateway = WiFi.gatewayIP();

  Serial.print("\n wifi connected! \n");
  Serial.print("subnet: "); Serial.println(subnet);
  Serial.print("gateway: "); Serial.println(gateway);
  Serial.print("target GS IP: "); Serial.println(gshost);

  #ifdef STATIC_IP
    /* configure a static IP for the drone */
    if (!WiFi.config(local_IP, gateway, subnet)) {
      Serial.println("STA failed to configure");
    }
  #endif
}

void loop() 
{
  udp.beginPacket(gshost, remoteport);
  while (SerialMAV.available())
  {
    c = SerialMAV.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];

      switch (msg.msgid)
      {
        /* add cases here for special cases / debugging (as below) */
        ///TODO: add case statement (and other logic) for resuming the data stream after final param sent to GS
        case MAVLINK_MSG_ID_MISSION_ACK:
        {
          /* resume status messages when mission upload is complete */
          mavlink_message_t startmsg;
          mavlink_msg_request_data_stream_pack(255, MAV_COMP_ID_ALL, &startmsg, fcSysID, MAV_COMP_ID_ALL, MAV_DATA_STREAM_ALL, sensorMsgFreqHz, 1);
          uint16_t startlen = mavlink_msg_to_send_buffer(buf, &startmsg);
          SerialMAV.write(buf, startlen);
        } break;
      }
      
      len = mavlink_msg_to_send_buffer(buf, &msg);
      udp.write(buf, len);
      
      break;
    }
  }
  udp.endPacket();

  int packetSize = udp.parsePacket();
  if (packetSize) 
  {
    while (udp.available())
    {
      c = udp.read();
      if (mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status))
      {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];

        switch (msg.msgid) 
        {
          /* add cases here for special cases / debugging (as below) */
          case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
          {
            /* stop all the status updates!! (so that the exchange happens better) */
            mavlink_message_t stopmsg;
            mavlink_msg_request_data_stream_pack(255, MAV_COMP_ID_ALL, &stopmsg, fcSysID, MAV_COMP_ID_ALL, MAV_DATA_STREAM_ALL, sensorMsgFreqHz, 0);
            uint16_t stoplen = mavlink_msg_to_send_buffer(buf, &stopmsg);
            SerialMAV.write(buf, stoplen);
          } break;
          case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
          {
            Serial.println("requested single param!");
          } break;
          case MAVLINK_MSG_ID_MISSION_COUNT:
          {
            /* stop all the status updates!! (so that the exchange happens better) */
            mavlink_message_t stopmsg;
            mavlink_msg_request_data_stream_pack(255, MAV_COMP_ID_ALL, &stopmsg, fcSysID, MAV_COMP_ID_ALL, MAV_DATA_STREAM_ALL, sensorMsgFreqHz, 0);
            uint16_t stoplen = mavlink_msg_to_send_buffer(buf, &stopmsg);
            SerialMAV.write(buf, stoplen);
          } break;
        }

        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        SerialMAV.write(buf, len);
        break;
      }
    }
  }
}

/* cool function for incrementing IP addresses */
void incrementIPAddress(IPAddress &ip, IPAddress subnet, int fullSubOcts)
{
  int i = 3;  /* start at lowest octet */
  Serial.print("testing ip: "); Serial.println(ip);
  while (i >= fullSubOcts) /* octet index should never go below zero */
  {
    Serial.print("i = "); Serial.println(i);
    uint8_t invnet = ~subnet[i];
    if ((ip[i] + 1) > invnet) /* if octet value exceeds subnet... */
    {
      Serial.print("octet "); Serial.print(i); Serial.print(" exceeded subnet max of "); Serial.println(invnet);
      ip[i] = 0;  /* reset value */
      i--; /* move to increment next higher octet */
      continue;
    } 

    else 
    {
      ip[i]++;  /* increment octet value */
      Serial.print("new ip: "); Serial.println(ip);
      return; 
    }
  }

  return;
}