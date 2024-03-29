#define BLYNK_TEMPLATE_ID "TMPL6g3pjkHqp"
#define BLYNK_TEMPLATE_NAME "TempHum"
#define BLYNK_AUTH_TOKEN "74O-g_0xMdV_5Aly2IUbKDoSXIm3AKtn"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>


#include "Arduino.h"
#include "PCF8574.h"  // https://github.com/xreef/PCF8574_library
#define I2C_Address 0x20
#define I2C_SDA_Pin 21
#define I2C_SCL_Pin 22
// Instantiate Wire for generic use at 100kHz
TwoWire I2Ctwo = TwoWire(1);
// Set i2c address
PCF8574 pcf8574(&I2Ctwo, I2C_Address, I2C_SDA_Pin, I2C_SCL_Pin);


// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "EM Anu";
char pass[] = "anuwat11";


/*******************************************************************************
 * ET-ESP32(WROVER) RS485 V2 
 * Tools->Board:"ESP32 Wrover Module"
 *******************************************************************************
 * I2C Interface & I2C Bus
 * -> IO22                = I2C_SCL
 * -> IO21                = I2C_SDA
 * -> I2C RTC:DS3231      = I2C Address : 0x68:1100100(x)
 * -> I2C EEPROM 24LC16   = I2C Address : 0x50:1010000(x)
 * -> I2C ADC MCP3423     = I2C Address : 0x6D:1100101(x)
 * -> I2C Sensor:BME280   = I2C Address : 0x76:1110110(x)
 * -> I2C Sebsor:SHT31    = I2C Address : 0x44:1000100(x)/0x45:1010101(x)
 * SPI Interface SD Card
 * -> SD_CS               = IO4
 * -> SPI_MISO            = IO19S
 * -> SPI_MOSI            = IO23
 * -> SPI_SCK             = IO18
 * UART2 RS485 Half Duplex Auto Direction
 * -> IO26                = RX2
 * -> IO27                = TX2
 * User Switch
 * -> IO36                = USER_SW
 * RTC Interrupt
 * -> IO39                = RTC_INT#
 *******************************************************************************/

//=================================================================================================
#include <Wire.h>
//=================================================================================================

//=================================================================================================
// Start of Default Hardware : ET-ESP32(WROVER) RS485 V2
//=================================================================================================
// Remap Pin USART -> C:\Users\Admin\Documents\Arduino\hardware\espressif\esp32\cores\esp32\HardwareSerial.cpp
//                    C:\Users\Admin\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.0\cores\esp32\HardwareSerial.cpp
//=================================================================================================
#include <HardwareSerial.h>
//=================================================================================================
#define SerialDebug Serial  // USB Serial(Serial0)
//=================================================================================================
#define SerialRS485_RX_PIN 26
#define SerialRS485_TX_PIN 27
#define SerialRS485 Serial2  // Serial2(IO27=TXD,IO26=RXD)
//=================================================================================================
#define SerialLora_RX_PIN 14
#define SerialLora_TX_PIN 13
#define SerialLora Serial1  // Serial1(IO13=TXD,IO14=RXD)
//=================================================================================================
#define LORA_RES_PIN 33  // ESP32-WROVER :IO33(LoRa-RESET)
#define LORA_RES_PRESS LOW
#define LORA_RES_RELEASE HIGH
//=================================================================================================
#define I2C_SCL_PIN 22  // ESP32-WROVER : IO22(SCL1)
#define I2C_SDA_PIN 21  // ESP32-WROVER : IO21(SDA1)
//=================================================================================================
#define LED_PIN 2  // ESP-WROVER  : IO2
#define LedON 1
#define LedOFF 0
//=================================================================================================
#define USER_SW_PIN 36  // ESP32-WROVER :IO36
#define SW_PRESS LOW
#define SW_RELEASE HIGH
//=================================================================================================
#define RTC_INT_PIN 39  // ESP32-WROVER :IO39
#define RTC_INT_ACTIVE LOW
#define RTC_INT_DEACTIVE HIGH
//=================================================================================================
// End of Default Hardware : ET-ESP32(WROVER) RS485 V2
//=================================================================================================

//=================================================================================================
// Demo RS485 Modbus RTU Interface Soil Moisture Sensor(SOIL MOISTURE-H MODBUS RTU)
// Red    = +5V or 24V(3.6-30VDC)
// Black  = GND
// White  = RS485(B)
// Yellow = RS485(A)
// Green  = NC
//=================================================================================================
// InputRegister[1] = Soil Moisture INT16 Value
// HoldingRegister[512] = Soil Moisture Sensor Slave ID
//=================================================================================================
#include "ModbusMaster.h"  // https://github.com/4-20ma/ModbusMaster
//=================================================================================================
ModbusMaster node;  // instantiate ModbusMaster object
//=================================================================================================

//=================================================================================================
uint8_t result;
//=================================================================================================
float soil_moisture_float_value;
//=================================================================================================

void setup() {
  //===============================================================================================
  // Start of Initial Default Hardware : ET-ESP32(WROVER) RS485 V2
  //===============================================================================================
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LedOFF);
  //===============================================================================================
  pinMode(USER_SW_PIN, INPUT_PULLUP);
  pinMode(RTC_INT_PIN, INPUT_PULLUP);
  //===============================================================================================
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  //===============================================================================================
  SerialDebug.begin(115200);
  while (!SerialDebug)
    ;
  //===============================================================================================
  // End of Initial Default Hardware : ET-ESP32(WROVER) RS485 V2
  //===============================================================================================

  //===============================================================================================
  SerialDebug.println();
  SerialDebug.println("ET-ESP32(WROVER)RS485 V2.....Ready");
  //===============================================================================================

  //===============================================================================================
  SerialDebug.println();
  SerialDebug.println("ET-ESP32(WROVER)RS485 V2...Demo RS485 Modbus Master Library");
  SerialDebug.println("Interface...Soil Moisture-H Modbus RTU");
  //===============================================================================================

  //===============================================================================================
  SerialRS485.begin(9600, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);
  while (!SerialRS485)
    ;
  //===============================================================================================
  node.begin(1, SerialRS485);  // Soil Moisture = Modbus slave ID 1
  //===============================================================================================

  pcf8574.pinMode(0, OUTPUT);
  pcf8574.pinMode(1, OUTPUT);
  pcf8574.pinMode(2, OUTPUT);
  pcf8574.pinMode(3, OUTPUT);
  pcf8574.pinMode(4, INPUT_PULLUP);
  pcf8574.pinMode(5, INPUT_PULLUP);
  pcf8574.pinMode(6, INPUT_PULLUP);
  pcf8574.pinMode(7, INPUT_PULLUP);
  pcf8574.begin();

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

}
int Counter = 0;
int Counter2 = 0;

void loop() {
  uint8_t result;
  uint16_t data[2];
  Blynk.run();

  Serial.println("get data");
  result = node.readInputRegisters(1, 2);
  if (result == node.ku8MBSuccess) {
    Serial.print("Temp: ");
    Serial.println(node.getResponseBuffer(0) / 10.0f);
    Serial.print("Humi: ");
    Serial.println(node.getResponseBuffer(1) / 10.0f);
    Serial.println();
    Blynk.virtualWrite(V1, node.getResponseBuffer(0) / 10.0f);
    Blynk.virtualWrite(V2, node.getResponseBuffer(1) / 10.0f);
  }

  if (pcf8574.digitalRead(P4) == LOW) {
    delay(20);
    while (pcf8574.digitalRead(P4) == LOW)
      delay(50);
    Counter++;
    delay(10);
    Serial.println(Counter);
    pcf8574.digitalWrite(P1, Counter % 2);
  }
  if (pcf8574.digitalRead(P5) == LOW) {
    delay(20);
    while (pcf8574.digitalRead(P5) == LOW)
      delay(50);
    Counter2++;
    delay(10);
    Serial.println(Counter2);
    pcf8574.digitalWrite(P2, Counter2 % 2);
  }

  delay(250);
}

BLYNK_WRITE(V3)
{
  int pinValue = param.asInt();  // assigning incoming value from pin V1 to a variable
  pcf8574.digitalWrite(P0, !pinValue);
  if (pinValue == 1) {
    // do something when button is pressed;
    Serial.println("White Lamp is ON");
  } else if (pinValue == 0) {
    Serial.println("White Lamp is OFF");
  }
}

BLYNK_WRITE(V4)
{
  int pinValue = param.asInt();  // assigning incoming value from pin V1 to a variable
  pcf8574.digitalWrite(P1, !pinValue);
  if (pinValue == 1) {
    Counter = 0;
    Serial.println("Green Lamp is ON");
  } else if (pinValue == 0) {
    Counter = 1;
    Serial.println("Green Lamp is OFF");
  }
}

BLYNK_WRITE(V5)
{
  int pinValue = param.asInt();  // assigning incoming value from pin V1 to a variable
  pcf8574.digitalWrite(P2, !pinValue);
  if (pinValue == 1) {
    Counter2 = 0;
    Serial.println("Yellow Lamp is ON");
  } else if (pinValue == 0) {
    Counter2 = 1;
    Serial.println("Yellow Lamp is OFF");
  }
}
