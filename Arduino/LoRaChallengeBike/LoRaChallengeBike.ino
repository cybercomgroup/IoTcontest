/*
  LoRa Challenge Bike Condition Monitoring
  @author: Pablo Puñal Pereira <pablo.punal@cybercom.com>
  @date: February 2017
*/
#include <Sodaq_RN2483.h>
#include <Sodaq_wdt.h>
#include <RTCZero.h>
#include "leds.h"
#include "gps.h"
#include "lora.h"
#include "env.h"

#define DEBUG SerialUSB
#define DEBUG_BAUDRATE 57600
#define GPS Serial
#define GPS_BAUDRATE 9600
#define LORA Serial2
#define LORA_BAUDRATE LoRaBee.getDefaultBaudRate()
#define SPIN_INTERRUPT  8
#define SPIN_LED        9
#define TEST_OUTPUT     10
#define GPS_DATA_PORT   100
#define SPINS_DATA_PORT 101
#define REGISTRATION_PORT 102

/* Global vars */
boolean spin_led_on = false;
String gps_message = "";
String gps_buffer = "";
int gps_buffer_size = 0;
long spin_counter = 0;
RTCZero rtc;
gps_data_t gps_data;
lora_connection_t lora_connection;
boolean task_send_spins = false;
boolean task_send_gps = false;


/* Setup Functions. */
void setup() {
  setupVars();
  setupIOs();
  LED_OFF;
  setupInterrupts();
  setupRTC();
  setupSerialPorts();
  DEBUG.println("[LoRa Challenge Bike Condition Monitoring]");
  DEBUG.println("setup.LoRa");

  switch(lora_connection=setupLoRa()) {
    case not_connected:
      DEBUG.println("LoRa not connected");
      break;
    case abp_connected:
      DEBUG.println("LoRa connected by ABP");
      break;
    case ota_connected:
      DEBUG.println("LoRa connected by OTA");
      break;
    default:
      DEBUG.println("Unexpected LoRa status");
      break;
  }

  sendRegistration();
}

void setupVars() {
  gps_message.reserve(200);
  gps_buffer.reserve(200);
}

void setupIOs() {
  pinMode(SPIN_LED, OUTPUT);
  pinMode(TEST_OUTPUT, OUTPUT);
  pinMode(SPIN_INTERRUPT, INPUT_PULLDOWN);
}

void setupInterrupts() {
  noInterrupts();
  attachInterrupt(SPIN_INTERRUPT, _INT_SPIN_WHEEL, RISING);
  interrupts();
}

void setupRTC() {
  rtc.begin();
  rtc.setTime(0,0,0);
  rtc.setDate(0,0,0);
  rtc.setAlarmSeconds(59);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(_INT_RTC);
}

void setupSerialPorts() {
  while((!DEBUG) && (millis() < 10000));
  DEBUG.begin(DEBUG_BAUDRATE);
  GPS.begin(GPS_BAUDRATE);
  LORA.begin(LORA_BAUDRATE);
}

lora_connection_t setupLoRa() {
  WHITE;
  if (LoRaBee.initOTA(LORA, OTA_configuration.DevEUI, OTA_configuration.AppEUI, OTA_configuration.AppKey, true))
    return ota_connected;
  if (LoRaBee.initABP(LORA, ABP_configuration.DevAddr, ABP_configuration.AppSKey, ABP_configuration.NwkSKey, true))
    return abp_connected;
  return not_connected;
}

/* Loop functions.  */
int loopCounter = 0;
void loop() {
  loopCounter++;

  if (!(loopCounter%50))
    digitalWrite(TEST_OUTPUT, HIGH);
  else
    digitalWrite(TEST_OUTPUT, LOW);
  delay(10);

  LED_OFF;
  if (lora_connection == not_connected) RED;

  /*if (!(loopCounter%100)) {
  checkIncomingMessages();
}*/

  /*if (task_send_gps) {
    sendGPS();
    task_send_gps = false;
  }*/

  if (task_send_spins) {
    sendSpins();
    task_send_spins = false;
  }
}

/* Helpers */
void analyzeNMEA(void) {
  String prefix = gps_message.substring(1,6);
  if (prefix == "GPRMC") {
    //DEBUG.println("GPRMC");
  } else if (prefix == "GPGGA") {
    //DEBUG.println("GPGGA");
    parseGPGGA();
  } else if (prefix == "GPGSA") {
    //DEBUG.println("GPGSA");
  } else {
    //DEBUG.println("Unexpected: "+prefix);
  }
}

void parseGPGGA() {
  DEBUG.print(gps_message);
  int comma=0, comma_position = 0, satellites, gpsTime, latG, latM, lonG, lonM;
  float latS, lonS;
  boolean north, east;
  String tmpString, tmpString2;

  for (int i=0; i<gps_message.length(); i++) {
    if (gps_message.charAt(i) == ',') {
      switch(comma++) {
        case 1:
          gpsTime = gps_message.substring(comma_position+1, i).toInt();
          break;
        case 2:
          tmpString = gps_message.substring(comma_position+1, i);
          tmpString2 = tmpString.substring(0, tmpString.indexOf('.'));
          latG = tmpString2.substring(0, tmpString2.length()-2).toInt();
          latM = tmpString2.substring(tmpString2.length()-2).toInt();
          tmpString2 = tmpString.substring(tmpString.indexOf('.')+1);
          latS = tmpString2.toFloat()/pow(10,tmpString2.length()-2);
          break;
        case 3:
          north = (gps_message.substring(comma_position+1, i).equals("N"));
          break;
        case 4:
          tmpString = gps_message.substring(comma_position+1, i);
          tmpString2 = tmpString.substring(0, tmpString.indexOf('.'));
          lonG = tmpString2.substring(0, tmpString2.length()-2).toInt();
          lonM = tmpString2.substring(tmpString2.length()-2).toInt();
          tmpString2 = tmpString.substring(tmpString.indexOf('.')+1);
          lonS = tmpString2.toFloat()/pow(10,tmpString2.length()-2);
          break;
        case 5:
          east = (gps_message.substring(comma_position+1, i).equals("E"));
          break;
        case 7:
          satellites = gps_message.substring(comma_position+1, i).toInt();
          break;
        default:
          break;
      }
      comma_position = i;
    }
  }


  if (satellites > 2) { // precision of at least 3 satellites is needed
    gps_data.satellites = satellites;
    gps_data.timeHH = (int) gpsTime/10000;
    gps_data.timeMM = (int)(gpsTime%10000 /100);
    gps_data.timeSS = gpsTime%100;
    gps_data.latitude = getGoogleCoords(north, latG, latM, latS);
    gps_data.longitude = getGoogleCoords(east, lonG, lonM, lonS);

  }

}

float getGoogleCoords(boolean emisphere, int g, int m, float s) {
  return (emisphere?1:-1)*(g+((s/60)+(float)m)/60);
}

boolean sendMessage(String message2send, int portNumber) {
  String message = message2send;
  DEBUG.print("Sending (");
  DEBUG.print(message.length());
  DEBUG.print(" bytes): ");
  DEBUG.println(message);
  if (lora_connection == not_connected)
    lora_connection = setupLoRa();
    else
  DEBUG.println("Already connected");

  boolean res = false;
  switch (LoRaBee.send(portNumber, (const uint8_t*)message.c_str(), message.length())) {
    case NoError:
      GREEN;
      DEBUG.println("Message sent");
      res = true;
      break;
    default:
      RED;
      DEBUG.println("Error -> reconnect next time");
      lora_connection = not_connected;
      res = false;
      break;
  }

  checkIncomingMessages();

  return res;
}

void sendSpins() {
  String spinsMsg = "{\"spins\":\""+String(spin_counter)+"\"}";
  if (sendMessage(spinsMsg, SPINS_DATA_PORT)) {
    spin_counter = 0;
  }
}

void sendRegistration() {
  String regMsg = "{\"bike\":1000}";
  sendMessage(regMsg, REGISTRATION_PORT);
}

void sendGPS() {
  String gpsMsg = "{\"la\":\""+String(gps_data.latitude,14)+"\",\"lo\":\""+String(gps_data.longitude,14)+"\"}";
  sendMessage(gpsMsg, GPS_DATA_PORT);
}

void checkIncomingMessages() {
  uint8_t payload[64];
  uint16_t len = LoRaBee.receive(payload, 64);
  String payloadText = "";
  if (len > 0) {
    payloadText = (char*)payload;
    DEBUG.println("Incoming Message: "+payloadText);
    //task_send_gps = true;
    sendGPS();
  }
  /*else {
    DEBUG.println("No Incoming Message");
  }*/
}

void printGPSdata() {
  DEBUG.print(" [GPS] time: ");
  DEBUG.print(gps_data.timeHH, 10);
  DEBUG.print(":");
  DEBUG.print(gps_data.timeMM, 10);
  DEBUG.print(":");
  DEBUG.print(gps_data.timeSS, 10);
  DEBUG.print(", satellites: ");
  DEBUG.print(gps_data.satellites, 10);
  DEBUG.print(", latitude: ");
  DEBUG.print(gps_data.latitude, 10);
  DEBUG.print(", longitude: ");
  DEBUG.print(gps_data.longitude, 10);
}

void print2digits(int number) {
  if (number < 10) {
    DEBUG.print("0");
  }
  DEBUG.print(number);
}

void printTime() {
  print2digits(rtc.getHours());
  DEBUG.print(":");
  print2digits(rtc.getMinutes());
  DEBUG.print(":");
  print2digits(rtc.getSeconds());
}

void printStatusReport() {
  printTime();
  DEBUG.print(" Spins: ");
  DEBUG.print(spin_counter, 10);
  printGPSdata();
  DEBUG.print("\n");
}

/* Interruptions */
void serialEventRun(void) {
  if (GPS.available()) _INT_GPS();
}

void _INT_GPS() {
  GREEN;
  while(GPS.available()) {
    char inChar = GPS.read();
    //DEBUG.print(inChar);
    if (inChar == 0x24 && gps_buffer_size > 1) { // $ means new message
      gps_message = gps_buffer;
      analyzeNMEA();
      gps_buffer_size = 0;
      gps_buffer = "";
    }
    gps_buffer_size++;
    gps_buffer += inChar;
  }
}

void _INT_SPIN_WHEEL() {
  BLUE;
  spin_counter++;
}

int rtc_counter = 0;

void _INT_RTC() {
  rtc_counter += 10;
  printStatusReport();
  //checkIncomingMessages();
  if (rtc_counter > 9) {
    rtc_counter = 0;
    task_send_spins = true;
  }
}
