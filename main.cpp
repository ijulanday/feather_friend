#include "main.h"


void setup() 
{
  /* debug serial */
  Serial.begin(9600);
  delay(1000);

  /* mavlink serial */
  SerialMAV.begin(57600);
  while (!SerialMAV) {}
  firstReceived = false;
  
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
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        SerialMAV.write(buf, len);
        break;
      }
    }
  }
}

