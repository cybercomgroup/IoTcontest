#include <EddystoneBeacon.h>
#include <iBeacon.h>
#include <RN487x_BLE.h>
#include <RN487x_CONST.h>

#include <Sodaq_RN2483.h>
#include <Sodaq_wdt.h>
#include <StringLiterals.h>
#include <Switchable_Device.h>
#include <Utils.h>

#include "env.h"

//Serial ports
#define debugSerial SerialUSB
#define bleSerial Serial1
#define loraSerial Serial2

int time;
uint8_t countBtDevices;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  while ((!debugSerial) && (millis() < 10000));

  debugSerial.begin(57600);
  
  rn487xBle.setDiag(debugSerial); // To get debug information, uncomment #define DEBUG in .cpp in library
  rn487xBle.hwInit() ;
  
  loraSerial.begin(LoRaBee.getDefaultBaudRate());
  bleSerial.begin(rn487xBle.getDefaultBaudRate()) ;
  
  LoRaBee.setDiag(debugSerial); // To get debug information, uncomment #define DEBUG in .cpp in library
  
  
  while(!setupLoRa()) //Retry until true is returned
  {
    debugSerial.println("Failed to setup LoRaWAN, trying again");
  }
  setupBT();

}

void loop()
{
  countBtDevices = 0;
  rn487xBle.startScanning();
  time = millis();
  while(1)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    if(bleSerial.available())
    {
      char x = (char)bleSerial.read();
      debugSerial.print(x) ;
      if(x == '\n')
      {
        countBtDevices++;
      }
    }

    if(millis() >= time + 15000) //Scan for x milliseconds and then sends message
    {
      countBtDevices--;
      break;
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
  rn487xBle.stopScanning();
  
  debugSerial.println("Message(" + String(sizeof(countBtDevices)) + " bytes): " + countBtDevices);
  switch (LoRaBee.send(1, &countBtDevices, sizeof(countBtDevices)))
  {
  case NoError:
    debugSerial.println("Successful transmission.");
    break;
  case NoResponse:
    debugSerial.println("There was no response from the device.");
    setupLoRa();
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
    setupLoRa();
    break;
  case Busy:
    debugSerial.println("The device is busy. Sleeping for 10 extra seconds.");
    delay(10000);
    break;
  case NetworkFatalError:
    debugSerial.println("There is a non-recoverable error with the network connection. The program will reset the RN module.");
    setupLoRa();
    break;
  case NotConnected:
    debugSerial.println("The device is not connected to the network. The program will reset the RN module.");
    setupLoRa();
    break;
  case NoAcknowledgment:
    debugSerial.println("There was no acknowledgment sent back!");
    break;
  default:
    break;
  }
}

/* Setup LoRa */
boolean setupLoRa()
{  
  // try first with OTA
  debugSerial.print("----------------\nTrying OTA configuration ");
  printOTAconfig(OTA_configuration);
  if (LoRaBee.initOTA(loraSerial, OTA_configuration.DevEUI, OTA_configuration.AppEUI, OTA_configuration.AppKey, true))
  {
    debugSerial.println("Finished setting up LorRaWAN");
    return true;
  }

  return false;
}

void setupBT()
{
  rn487xBle.initBleStream(&bleSerial);
  rn487xBle.factoryReset();
  
  if (rn487xBle.swInit())
  {
    debugSerial.println("Init. procedure done!") ;
  }
  else
  {
    debugSerial.println("Init. procedure failed!") ;
    while(1) ;
  }

  // >> Configuring the BLE
  
  // First enter in command/configuration mode
  rn487xBle.enterCommandMode() ;
  
//  // Enable low power for longer runtime
//  if(rn487xBle.enableLowPower())
//  {
//    debugSerial.println("Low power enabled");
//  }
//  else
//  {
//    debugSerial.println("Low power could not be enabled");
//  }

  // Set advertisement power
  if(rn487xBle.setAdvPower(0))
  {
    debugSerial.println("Set advertisement power");
  }
  else
  {
    debugSerial.println("Could not set advertisement power");
  }

  // Set connection power
  if(rn487xBle.setConPower(0))
  {
    debugSerial.println("Set connection power");
  }
  else
  {
    debugSerial.println("Could not set connection power");
  }

  // Remove GATT services
  rn487xBle.setDefaultServices(NO_SERVICE) ;
  // Set passive scan and does not filter out duplicate scan results
  //rn487xBle.setSupportedFeatures(PASSIVE_SCAN_BMP) ;
  // Take into account the settings by issuing a reboot
  rn487xBle.reboot() ;
  rn487xBle.enterCommandMode() ;
  // Halt advertisement
  rn487xBle.stopAdvertising() ;

  debugSerial.println("Finished setting up Bluetooth");
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
