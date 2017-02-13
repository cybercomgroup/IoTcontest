#include <Sodaq_RN2483.h>
#include <Sodaq_wdt.h>
#include <StringLiterals.h>
#include <Switchable_Device.h>
#include <Utils.h>

#include "env.h"

#define debugSerial SerialUSB
#define loraSerial Serial2

//const uint8_t devAddr[4] =
//{
//  0xED, 0x0D, 0xDA, 0xDB,
//};
//
//const uint8_t appSKey[16] =
//{
//  0x6B, 0x2C, 0x4B, 0xF0,
//  0x59, 0xFB, 0x2E, 0xCC,
//  0x63, 0x37, 0x63, 0xF0,
//  0x2A, 0xC9, 0xD3, 0x05,
//};
//
//const uint8_t nwkSKey[16] =
//{
//  0x46, 0x90, 0x9F, 0x22,
//  0x34, 0x38, 0xF8, 0xA9,
//  0x02, 0x25, 0x21, 0x1A,
//  0xD1, 0xDE, 0xF5, 0x36,
//};


const uint8_t testPayload[] =
{
  0x13, 0x37, 0x13, 0x37
};

void RED() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void GREEN() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void BLUE() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);
}

void ledOFF()
{
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void setup()
{

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  ledOFF();

  while ((!debugSerial) && (millis() < 10000));

  debugSerial.begin(57600);
  loraSerial.begin(LoRaBee.getDefaultBaudRate());

  LoRaSetup();
//
//  debugSerial.print("devAddr: ");
//  for (int i = 0; i < 4; i++)
//  {
//    debugSerial.print(devAddr[i], HEX);
//  }
//  debugSerial.println("");
//  debugSerial.print("appSKey: ");
//  for (int i = 0; i < 16; i++)
//  {
//    debugSerial.print(appSKey[i], HEX);
//  }
//  debugSerial.println("");
//  debugSerial.print("nwkSkey: ");
//  for (int i = 0; i < 16; i++)
//  {
//    debugSerial.print(nwkSKey[i], HEX);
//  }
//  debugSerial.println("");


}

void LoRaSetup()
{
  LoRaBee.setDiag(loraSerial); // optional
  bool init = LoRaBee.initABP(loraSerial, devAddr, appSKey, nwkSKey, false);
  if (init)
  {
    debugSerial.println("Connection to the network was successful.");

  }
  else
  {
    debugSerial.println("Connection to the network failed!");
  }
}

void loop()
{
   String reading = getTemperature();

    switch (LoRaBee.send(1, (uint8_t*)reading.c_str(), reading.length()))
    {
    case NoError:
      debugSerial.println("Successful transmission.");
      GREEN();
      break;
    case NoResponse:
      debugSerial.println("There was no response from the device.");
      break;
    case Timeout:
      debugSerial.println("Connection timed-out. Check your serial connection to the device! Sleeping for 20sec.");
      delay(20000);
      break;
    case PayloadSizeError:
      debugSerial.println("The size of the payload is greater than allowed. Transmission failed!");
      break;
    case InternalError:
      debugSerial.println("Oh No! This shouldn't happen. Something is really wrong! The program will reset the RN module.");
      LoRaSetup();
      break;
    case Busy:
      debugSerial.println("The device is busy. Sleeping for 10 extra seconds.");
      delay(10000);
      break;
    case NetworkFatalError:
      debugSerial.println("There is a non-recoverable error with the network connection. The program will reset the RN module.");
      LoRaSetup();
      break;
    case NotConnected:
      debugSerial.println("The device is not connected to the network. The program will reset the RN module.");
      LoRaSetup();
      break;
    case NoAcknowledgment:
      debugSerial.println("There was no acknowledgment sent back!");
      break;
    default:
      break;
    }
    // Delay between readings
    // 60 000 = 1 minute
    delay(5000); 
    ledOFF();
}

String getTemperature()
{
  //10mV per C, 0C is 500mV
  float mVolts = (float)analogRead(TEMP_SENSOR) * 3300.0 / 1023.0;
  float temp = (mVolts - 500.0) / 10.0;
  
  return String(temp);
}
