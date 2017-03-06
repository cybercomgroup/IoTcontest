#include <Sodaq_RN2483.h>
#include <Sodaq_wdt.h>
#include <StringLiterals.h>
#include <Switchable_Device.h>
#include <Utils.h>

#include "env.h"

//Serial ports
#define debugSerial SerialUSB
#define loraSerial Serial2

//Sensor IO
#define SMDigital 8 //D8
#define SMAnalog  0 //A0
#define WLAnalog 1 //A1


void RED()
{
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void GREEN()
{
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void BLUE()
{
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);
}

void WHITE()
{
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
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
  // Leds
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  // sensors
  pinMode(SMDigital, INPUT);

  ledOFF();
  
  while ((!debugSerial) && (millis() < 10000));

  debugSerial.begin(57600);
  loraSerial.begin(LoRaBee.getDefaultBaudRate());
  LoRaBee.setDiag(debugSerial);
  setupLoRa();

  uint8_t buf[8];
  uint8_t x = LoRaBee.getHWEUI(buf, sizeof(buf));
  debugSerial.println("HWEUI:");
  for(int i = 0; i < x; i++)
  {
    debugSerial.print(buf[i], HEX);
  }
  debugSerial.println();
}

void loop()
{
  
  String message = getMessage();
  debugSerial.println("Message(" + String(message.length())+" bytes): " + message);
  switch (LoRaBee.send(1, (uint8_t*)message.c_str(), message.length())) //switch (LoRaBee.send(1, testPayload, 2))
  {
  case NoError:
    debugSerial.println("Successful transmission.");
    GREEN();
    break;
  case NoResponse:
    debugSerial.println("There was no response from the device.");
    RED();
    setupLoRa();
    break;
  case Timeout:
    debugSerial.println("Connection timed-out. Check your serial connection to the device! Sleeping for 20sec.");
    RED();
    delay(20000);
    break;
  case PayloadSizeError:
    debugSerial.println("The size of the payload is greater than allowed. Transmission failed!");
    RED();
    break;
  case InternalError:
    debugSerial.println("Oh No! This shouldn't happen. Something is really wrong! The program will reset the RN module.");
    RED();
    setupLoRa();
    break;
  case Busy:
    debugSerial.println("The device is busy. Sleeping for 10 extra seconds.");
    RED();
    delay(10000);
    break;
  case NetworkFatalError:
    debugSerial.println("There is a non-recoverable error with the network connection. The program will reset the RN module.");
    RED();
    setupLoRa();
    break;
  case NotConnected:
    debugSerial.println("The device is not connected to the network. The program will reset the RN module.");
    RED();
    setupLoRa();
    break;
  case NoAcknowledgment:
    debugSerial.println("There was no acknowledgment sent back!");
    RED();
    break;
  default:
    break;
  }
  // Delay between readings
  // 60 000 = 1 minute
  delay(10000);

  receiveData();

  ledOFF();
}

/* Setup LoRa */
boolean setupLoRa()
{
  BLUE();
  
  // try first with OTA
  debugSerial.print("----------------\nTrying OTA configuration ");
  printOTAconfig(OTA_configuration);
  if (LoRaBee.initOTA(loraSerial, OTA_configuration.DevEUI, OTA_configuration.AppEUI, OTA_configuration.AppKey, true))
  {
    debugSerial.println("[Done]\n");
    WHITE();
    return true;
  }
  else
  {
    debugSerial.println("[Fail]\n");
  }

  // try ABP
  debugSerial.print("----------------\nTrying ABP configuration ");
  printABPconfig(ABP_configuration);
  if (LoRaBee.initABP(loraSerial, ABP_configuration.DevAddr, ABP_configuration.AppSKey, ABP_configuration.NwkSKey, true))
  {
    debugSerial.println("[Done]\n");
    WHITE();
    return true;
  }
  else
  {
    debugSerial.println("[Fail]\n");
  }

  return false;
}

/* Helpers */
void printOTAconfig(ota_config_t ota_config)
{
  debugSerial.print("DevEUI:");
  for(int i=0; i<8; i++)
  {
    debugSerial.print(ota_config.DevEUI[i], HEX);
  }
  debugSerial.print("\nAppEUI:");
  for(int i=0; i<8; i++)
  {
    debugSerial.print(ota_config.AppEUI[i], HEX);
  }
  debugSerial.print("\nAppKey:");
  for(int i=0; i<16; i++)
  {
    debugSerial.print(ota_config.AppKey[i], HEX);
  }
  debugSerial.print("\n");
}

void printABPconfig(abp_config_t abp_config)
{
  debugSerial.print("DevAddr:");
  for(int i=0; i<4; i++)
  {
    debugSerial.print(abp_config.DevAddr[i], HEX);
  }
  debugSerial.print("\nAppSKey:");
  for(int i=0; i<16; i++)
  {
    debugSerial.print(abp_config.AppSKey[i], HEX);
  }
  debugSerial.print("\nNwkSKey:");
  for(int i=0; i<16; i++)
  {
    debugSerial.print(abp_config.NwkSKey[i], HEX);
  }
  debugSerial.print("\n");
}

String getMessage()
{
  return String(1337);
}

void receiveData()
{
  // After we have send some data, we can receive some data
  // First we make a buffer
  uint8_t payload[64];
  // Now we fill the buffer and
  // len = the size of the data
  uint16_t len = LoRaBee.receive(payload, 64);
  String HEXPayload = "";

  // When there is no payload the lorabee.receive will return 0
  // I filter this out
  if (len > 0)
  {
    for (int i = 0; i < len; i++)
    {
      HEXPayload += String(payload[i], HEX);
    }
    debugSerial.println(HEXPayload);
  }
  else
  {
    debugSerial.println("no received transmission!");
  }
}
