#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

const int spiCSPin = 10;

MCP_CAN CAN(spiCSPin);

void setup()
{
  Serial.begin(9600);

  while (CAN_OK != CAN.begin(CAN_500KBPS))
  {
    Serial.println("CAN BUS Init Failed");
    delay(100);
  }
  Serial.println("CAN BUS  Init OK!");
}


void loop()
{
  unsigned char len = 0;
  unsigned char buf[8];

  if(CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf);

    unsigned long canId = CAN.getCanId();

    Serial.println("-----------------------------");
    Serial.print("Data from ID: 0x");
    Serial.println(canId, HEX);

    for(int i = 0; i<len; i++)
    {
      Serial.print(buf[i]);
      Serial.print(" | ");
    }
    Serial.println();

    /*
    if (canId == 21) // ÖN ULTRASONIK
    {
    unsigned int number[4];
    number[0] = buf[0] | buf[1] << 8;
    number[1] = buf[2] | buf[3] << 8;
    number[2] = buf[4] | buf[5] << 8;
    number[3] = buf[6] | buf[7] << 8;
    Serial.print(number[0]);
    Serial.print(" | ");
    Serial.print(number[1]);
    Serial.print(" | ");
    Serial.print(number[2]);
    Serial.print(" | ");
    Serial.print(number[3]);
    Serial.println();
    }
    */

    //MOTOR KONTROL
    if (canId == 25)
    {
      
    }
  }
}