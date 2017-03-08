/*
  LoRa Challenge Bike Condition Monitoring
  @author: Pablo Puñal Pereira <pablo.punal@cybercom.com>
  @date: February 2017
*/
#include <Sodaq_RN2483.h>
#include <Sodaq_wdt.h>
#include <RTCZero.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
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
#define EXT_LED_RED        7
#define EXT_LED_GREEN         6
#define GPS_DATA_PORT   100
#define SPINS_DATA_PORT 101
#define REGISTRATION_PORT 102
#define FALL_PORT 103

/* AccGyroMag Sensor */
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);
#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

/* Global vars */
boolean EXT_LED_RED_on = false;
boolean EXT_LED_GREEN_on = false;
String gps_message = "";
String gps_buffer = "";
int gps_buffer_size = 0;
long spin_counter = 0;
RTCZero rtc;
gps_data_t gps_data;
lora_connection_t lora_connection;
boolean task_send_spins = false;
boolean task_send_gps = false;
int fall_counter = 0;


/* Setup Functions. */
void setup() {
  setupVars();
  setupIOs();
  LED_OFF;
  setupInterrupts();
  setupRTC();
  setupSerialPorts();
  setupSensors();
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
  pinMode(EXT_LED_RED, OUTPUT);
  pinMode(EXT_LED_GREEN, OUTPUT);
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

void setupSensors() {
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    DEBUG.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    blink_EXT_LED_RED();
    while(1);
  }
  DEBUG.println(F("LSM9DS0 9DOF Connected!"));
  printSensorDetails();

  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
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
void loop() {

  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  /*DEBUG.print("Accel X: "); DEBUG.print(accel.acceleration.x); DEBUG.print(" ");
  DEBUG.print("  \tY: "); DEBUG.print(accel.acceleration.y);       DEBUG.print(" ");
  DEBUG.print("  \tZ: "); DEBUG.print(accel.acceleration.z);     DEBUG.println("  \tm/s^2");
*/
  // print out magnetometer data
  /*DEBUG.print("Magn. X: "); DEBUG.print(mag.magnetic.x); DEBUG.print(" ");
  DEBUG.print("  \tY: "); DEBUG.print(mag.magnetic.y);       DEBUG.print(" ");
  DEBUG.print("  \tZ: "); DEBUG.print(mag.magnetic.z);     DEBUG.println("  \tgauss");

  // print out gyroscopic data
  DEBUG.print("Gyro  X: "); DEBUG.print(gyro.gyro.x); DEBUG.print(" ");
  DEBUG.print("  \tY: "); DEBUG.print(gyro.gyro.y);       DEBUG.print(" ");
  DEBUG.print("  \tZ: "); DEBUG.print(gyro.gyro.z);     DEBUG.println("  \tdps");

  // print out temperature data
  DEBUG.print("Temp: "); DEBUG.print(temp.temperature); DEBUG.println(" *C");

  DEBUG.println("**********************\n");*/

  if (accel.acceleration.z < 5) {
    fall_counter++;
    if (fall_counter > 100) {
      fall_counter = 0;
      sendFall();
    }
    DEBUG.println("FALL detected!");
    DEBUG.println(fall_counter,10);
    blink_EXT_LED_RED();
  } else {
    fall_counter = 0;
    EXT_LED_RED_on = false;
    digitalWrite(EXT_LED_RED, LOW);
  }


  delay(100);
  LED_OFF;
  if (lora_connection == not_connected) RED;
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
  //DEBUG.print(gps_message);
  //blink_EXT_LED_GREEN();
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

void sendFall() {
  String fallMsg = "{\"fall\":true,\"la\":"+String(gps_data.latitude,14)+",\"lo\":"+String(gps_data.longitude,14)+"}";
  sendMessage(fallMsg, FALL_PORT);
}

void sendGPS() {
  String gpsMsg = "{\"la\":"+String(gps_data.latitude,14)+",\"lo\":"+String(gps_data.longitude,14)+"}";
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

void printSensorDetails(void)
{
  sensor_t accel, mag, gyro, temp;

  lsm.getSensor(&accel, &mag, &gyro, &temp);

  DEBUG.println(F("------------------------------------"));
  DEBUG.print  (F("Sensor:       ")); DEBUG.println(accel.name);
  DEBUG.print  (F("Driver Ver:   ")); DEBUG.println(accel.version);
  DEBUG.print  (F("Unique ID:    ")); DEBUG.println(accel.sensor_id);
  /*/DEBUG.print  (F("Max Value:    ")); DEBUG.print(accel.max_value); DEBUG.println(F(" m/s^2"));
  DEBUG.print  (F("Min Value:    ")); DEBUG.print(accel.min_value); DEBUG.println(F(" m/s^2"));
  DEBUG.print  (F("Resolution:   ")); DEBUG.print(accel.resolution); DEBUG.println(F(" m/s^2"));*/
  DEBUG.println(F("------------------------------------"));
  DEBUG.println(F(""));

  DEBUG.println(F("------------------------------------"));
  DEBUG.print  (F("Sensor:       ")); DEBUG.println(mag.name);
  DEBUG.print  (F("Driver Ver:   ")); DEBUG.println(mag.version);
  DEBUG.print  (F("Unique ID:    ")); DEBUG.println(mag.sensor_id);
  /*DEBUG.print  (F("Max Value:    ")); DEBUG.print(mag.max_value); DEBUG.println(F(" uT"));
  DEBUG.print  (F("Min Value:    ")); DEBUG.print(mag.min_value); DEBUG.println(F(" uT"));
  DEBUG.print  (F("Resolution:   ")); DEBUG.print(mag.resolution); DEBUG.println(F(" uT"));*/
  DEBUG.println(F("------------------------------------"));
  DEBUG.println(F(""));

  DEBUG.println(F("------------------------------------"));
  DEBUG.print  (F("Sensor:       ")); DEBUG.println(gyro.name);
  DEBUG.print  (F("Driver Ver:   ")); DEBUG.println(gyro.version);
  DEBUG.print  (F("Unique ID:    ")); DEBUG.println(gyro.sensor_id);
  /*DEBUG.print  (F("Max Value:    ")); DEBUG.print(gyro.max_value); DEBUG.println(F(" rad/s"));
  DEBUG.print  (F("Min Value:    ")); DEBUG.print(gyro.min_value); DEBUG.println(F(" rad/s"));
  DEBUG.print  (F("Resolution:   ")); DEBUG.print(gyro.resolution); DEBUG.println(F(" rad/s"));*/
  DEBUG.println(F("------------------------------------"));
  DEBUG.println(F(""));

  DEBUG.println(F("------------------------------------"));
  DEBUG.print  (F("Sensor:       ")); DEBUG.println(temp.name);
  DEBUG.print  (F("Driver Ver:   ")); DEBUG.println(temp.version);
  DEBUG.print  (F("Unique ID:    ")); DEBUG.println(temp.sensor_id);
  /*DEBUG.print  (F("Max Value:    ")); DEBUG.print(temp.max_value); DEBUG.println(F(" C"));
  DEBUG.print  (F("Min Value:    ")); DEBUG.print(temp.min_value); DEBUG.println(F(" C"));
  DEBUG.print  (F("Resolution:   ")); DEBUG.print(temp.resolution); DEBUG.println(F(" C"));*/
  DEBUG.println(F("------------------------------------"));
  DEBUG.println(F(""));

  delay(500);
}

void blink_EXT_LED_RED() {
  EXT_LED_RED_on = !EXT_LED_RED_on;
  if (EXT_LED_RED_on) digitalWrite(EXT_LED_RED, HIGH);
  else digitalWrite(EXT_LED_RED, LOW);
}


void blink_EXT_LED_GREEN() {
  EXT_LED_GREEN_on = !EXT_LED_GREEN_on;
  if (EXT_LED_GREEN_on) digitalWrite(EXT_LED_GREEN, HIGH);
  else digitalWrite(EXT_LED_GREEN, LOW);
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
  blink_EXT_LED_GREEN();
  spin_counter++;
}

int rtc_counter = 0;

void _INT_RTC() {
  rtc_counter ++;
  printStatusReport();
  //checkIncomingMessages();
  if (rtc_counter > 9) {
    rtc_counter = 0;
    task_send_spins = true;
  }
}
