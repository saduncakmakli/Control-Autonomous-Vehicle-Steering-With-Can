#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

//UNO İCİN
//MOSI 11 İLE KISA DEVRE!
//MISO 12 İLE KISA DEVRE!
//SCK 13 İLE KISA DEVRE!
//ISCP ÜZERİNDEN SPI KULLANIRKEN BU PINLERI KULLANMA!
//SMD OLMAYAN DIP SOKETLİ ARDUİNOLARDA 2.ICSP PINLARI 11,12,13DEN BAĞIMSIZ KULLANILABİLİR.
//UNO PWM PINLER 3,5,6,9,10
//UNO INTERRUPT PINLER 2,3

//MEGA İCİN
//MOSI 51
//MISO 50
//SCK 52
//ISCP ÜZERİNDEN SPI KULLANIRKEN BU PINLERI KULLANMA!
//MEGA PWM PINLER 2,3,4,5,6,7,8,9,10,11,12,13
//13 ON BOARD LED
//MEGA INTERRUPT PINLER INT0->2, INT1->3, INT2->21, INT3->20, INT4->19, INT5->18

//SERIAL DEBUG ICIN 0,1 KULLANILIYOR BU PINLERI KULLANMA!

const int spiCSPin = 10; //50 MISO, 51 MOSI, 52 SCK

const int LIMIT_SOL_INTERRUPT_PIN = 21; //INT 2
const int LIMIT_SAG_INTERRUPT_PIN = 20; //INT 3

const int ENCODER_A_INTERRUPT_PIN = 19; //INT 4
const int ENCODER_B_INTERRUPT_PIN = 18; //INT 5
//const int ENCODER_Z_PIN = NULL;

//LIMIT SWITLER BİR UCU SOL->20, SAG->21 DIGER UCU GND

MCP_CAN CAN(spiCSPin);

volatile long temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder

void EncoderAInterrupt()
{
  // ai0 is activated if DigitalPin nr ENCODER_A_INTERRUPT_PIN is going from LOW to HIGH
  // Check pin ENCODER_B_INTERRUPT_PIN to determine the direction
  if (digitalRead(ENCODER_B_INTERRUPT_PIN) == LOW)
  {
    counter++;
  }
  else
  {
    counter--;
  }
}

void EncoderBInterrupt()
{
  // ai0 is activated if DigitalPin nr ENCODER_B_INTERRUPT_PIN is going from LOW to HIGH
  // Check with pin ENCODER_A_INTERRUPT_PIN to determine the direction
  if (digitalRead(ENCODER_A_INTERRUPT_PIN) == LOW)
  {
    counter--;
  }
  else
  {
    counter++;
  }
}

void kalibrasyon()
{

}

void sag_limit()
{
  
}

void sol_limit()
{

}

void setup()
{
  Serial.begin(9600);
  pinMode(ENCODER_A_INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_INTERRUPT_PIN, INPUT_PULLUP);

  //Setting up interrupt
  //A rising pulse from encodenren activated EncoderAInterrupt(). AttachInterrupt ENCODER_A_INTERRUPT_PIN.
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_INTERRUPT_PIN), EncoderAInterrupt, RISING);

  //B rising pulse from encodenren activated EncoderBInterrupt(). AttachInterrupt ENCODER_B_INTERRUPT_PIN.
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_INTERRUPT_PIN), EncoderBInterrupt, RISING);

  pinMode(LIMIT_SOL_INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SAG_INTERRUPT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LIMIT_SOL_INTERRUPT_PIN), sol_limit, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SAG_INTERRUPT_PIN), sag_limit, FALLING);

  Serial.begin(9600);

  while (CAN_OK != CAN.begin(CAN_500KBPS))
  {
    Serial.println("CAN BUS Init Failed");
    delay(100);
  }
  Serial.println("CAN BUS  Init OK!");

  kalibrasyon();
}

void loop()
{
  // Send the value of counter
  if (counter != temp)
  {
    Serial.println(counter);
    temp = counter;
  }

  unsigned char len = 0;
  unsigned char buf[8];

  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf);

    unsigned long canId = CAN.getCanId();

    Serial.println("-----------------------------");
    Serial.print("Data from ID: 0x");
    Serial.println(canId, HEX);

    for (int i = 0; i < len; i++)
    {
      Serial.print(buf[i]);
      Serial.print(" | ");
    }
    Serial.println();

    //MOTOR KONTROL
    if (canId == 250) //Direksiyon Açı bilgisi
    {
    }
  }
}