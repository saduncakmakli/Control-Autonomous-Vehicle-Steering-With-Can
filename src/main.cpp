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
//LIMIT SWITLER BİR UCU SOL->20, SAG->21 DIGER UCU 5V

//ENCODER 360 DERECE = 2048 PULSE
const int ENCODER_A_INTERRUPT_PIN = 19; //INT 4
const int ENCODER_B_INTERRUPT_PIN = 18; //INT 5
//const int ENCODER_Z_PIN = NULL;

//BTS7960B 20 Amper Motor Sürücü Kartı Bağlantısı
//VCC : +5 VDC
//GND : Toprak Bağlantısı
//L_IS, R_IS : Akım Test Pinleri //Current alarm output
//L_EN, R_EN : PWM girişleri
//L_IN, R_IN : Motor yön kontrol pinleri 

const int MOTORDRIVER_LEFT_ENABLE_PIN = 22;
const int MOTORDRIVER_RIGHT_ENABLE_PIN = 23;
const int MOTORDRIVER_LEFT_PWM_PIN = 9; //PWM PIN
const int MOTORDRIVER_RIGHT_PWM_PIN = 10; //PWM PIN

MCP_CAN CAN(spiCSPin);

volatile short temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder

volatile short max_left_to_right = 0; //Kalibrasyon sonrası tam orta nokta bu sayının yarısı olmalı.

enum calibration {left_calibration, right_calibration, running};

volatile calibration calibration_state = left_calibration;

volatile short target = 0;

void EncoderAInterrupt();
void EncoderBInterrupt();
void sag_limit();
void sol_limit();
void sagdon();
void soldon();
void sagdon(int pwm);
void soldon(int pwm);
void gucukes();

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

void sag_limit()
{
  gucukes();
  if (calibration_state != left_calibration)
  {
    max_left_to_right = counter;

    if (calibration_state == right_calibration) //Sağ kalibrasyonun tamamlanması.
    {
      calibration_state = running;
      target = max_left_to_right/2;
    }
  }

}

void sol_limit()
{
  gucukes();
  counter = 0;
  if (calibration_state == left_calibration) //Sol kalibrasyonun tamamlanması.
  {
    calibration_state = right_calibration;
    sagdon();
  }
}

void sagdon()
{
  digitalWrite(MOTORDRIVER_RIGHT_ENABLE_PIN, HIGH);
  digitalWrite(MOTORDRIVER_LEFT_ENABLE_PIN, HIGH);
  digitalWrite(MOTORDRIVER_LEFT_PWM_PIN, LOW);
  analogWrite(MOTORDRIVER_RIGHT_PWM_PIN, 255);
}

void sagdon(int pwm)
{
  digitalWrite(MOTORDRIVER_RIGHT_ENABLE_PIN, HIGH);
  digitalWrite(MOTORDRIVER_LEFT_ENABLE_PIN, HIGH);
  digitalWrite(MOTORDRIVER_LEFT_PWM_PIN, LOW);
  analogWrite(MOTORDRIVER_RIGHT_PWM_PIN, pwm);
}

void soldon()
{
  digitalWrite(MOTORDRIVER_LEFT_ENABLE_PIN, HIGH);
  digitalWrite(MOTORDRIVER_RIGHT_ENABLE_PIN, HIGH);
  digitalWrite(MOTORDRIVER_RIGHT_PWM_PIN, LOW);
  analogWrite(MOTORDRIVER_LEFT_PWM_PIN, 255);
}

void soldon(int pwm)
{
  digitalWrite(MOTORDRIVER_LEFT_ENABLE_PIN, HIGH);
  digitalWrite(MOTORDRIVER_RIGHT_ENABLE_PIN, HIGH);
  digitalWrite(MOTORDRIVER_RIGHT_PWM_PIN, LOW);
  analogWrite(MOTORDRIVER_LEFT_PWM_PIN, pwm);
}

void gucukes()
{
  digitalWrite(MOTORDRIVER_LEFT_ENABLE_PIN, LOW);
  digitalWrite(MOTORDRIVER_RIGHT_ENABLE_PIN, LOW);
  digitalWrite(MOTORDRIVER_LEFT_PWM_PIN, LOW);
  digitalWrite(MOTORDRIVER_RIGHT_PWM_PIN, LOW);
}

void setup()
{
  Serial.begin(9600);

  //ENCODER SETUP
  pinMode(ENCODER_A_INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_INTERRUPT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_INTERRUPT_PIN), EncoderAInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_INTERRUPT_PIN), EncoderBInterrupt, RISING);

  //LIMIT SWITCH SETUP
  pinMode(LIMIT_SOL_INTERRUPT_PIN, INPUT);
  pinMode(LIMIT_SAG_INTERRUPT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(LIMIT_SOL_INTERRUPT_PIN), sol_limit, LOW);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SAG_INTERRUPT_PIN), sag_limit, LOW);

  //MOTOR DRIVER SETUP
  pinMode(MOTORDRIVER_LEFT_ENABLE_PIN,OUTPUT);
  pinMode(MOTORDRIVER_RIGHT_ENABLE_PIN, OUTPUT);
  pinMode(MOTORDRIVER_LEFT_PWM_PIN, OUTPUT);
  pinMode(MOTORDRIVER_RIGHT_PWM_PIN, OUTPUT);

  digitalWrite(MOTORDRIVER_LEFT_ENABLE_PIN, LOW);
  digitalWrite(MOTORDRIVER_RIGHT_ENABLE_PIN, LOW);
  digitalWrite(MOTORDRIVER_LEFT_PWM_PIN, LOW);
  digitalWrite(MOTORDRIVER_RIGHT_PWM_PIN, LOW);

  //SERIAL BEGIN
  Serial.begin(9600);

  //CAN CONNETION BEGIN
  while (CAN_OK != CAN.begin(CAN_500KBPS))
  {
    Serial.println("CAN BUS Init Failed");
    delay(100);
  }
  Serial.println("CAN BUS  Init OK!");

  soldon();
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

    //GELEN ACIYA GORE MOTOR KONTROL
    if (canId == 250) //Direksiyon Açı bilgisi
    {
      target = map(buf[1], 0, 255, 0, max_left_to_right);
      if ()
      //pıd entegre edilebilir.
    }
  }
}