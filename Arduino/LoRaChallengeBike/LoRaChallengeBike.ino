/*
  LoRa Challenge Bike Condition Monitoring
  @author: Pablo Puñal Pereira <pablo.punal@cybercom.com>
  @date: February 2017
*/
#include <Sodaq_RN2483.h>
#include <Sodaq_wdt.h>
#include "leds.h"

#define DEBUG SerialUSB
#define DEBUG_BAUDRATE 57600
#define GPS Serial
#define GPS_BAUDRATE 4800
#define LORA Serial2
#define LORA_BAUDRATE LoRaBee.getDefaultBaudRate()
#define SPIN_INTERRUPT  8
#define SPIN_LED        9
#define TEST_OUTPUT     10

/* Global vars */
boolean spin_led_on = false;
String gps_message = "";
String gps_buffer = "";
int gps_buffer_size = 0;
long spin_counter = 0;

/* Setup Functions. */
void setup() {
  setupVars();
  setupIOs();
  RED;
  setupInterrupts();
  setupSerialPorts();
  DEBUG.println("[LoRa Challenge Bike Condition Monitoring]");
  DEBUG.println("setup.LoRa");
  if (setupLoRa())
    DEBUG.println(" Done");
  else
    DEBUG.println(" Fails");
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

void setupSerialPorts() {
  while((!DEBUG) && (millis() < 10000));
  DEBUG.begin(DEBUG_BAUDRATE);
  GPS.begin(GPS_BAUDRATE);
  LORA.begin(LORA_BAUDRATE);
}

boolean setupLoRa() {
  return true;
}

/* Loop functions.  */
int loopCounter = 0;
void loop() {

  if (loopCounter%5)
    digitalWrite(TEST_OUTPUT, HIGH);
  else
    digitalWrite(TEST_OUTPUT, LOW);

  if ((loopCounter++)%10 == 0) {
    DEBUG.print("Spins: ");
    DEBUG.println(spin_counter, 10);
  }
  delay(100);
  LED_OFF;
}

/* Helpers */
void analyzeNMEA(void) {
  String prefix = gps_message.substring(1,6);
  if (prefix == "GPRMC") {
    DEBUG.println("GPRMC");
  } else if (prefix == "GPGGA") {
    DEBUG.println("GPGGA");
  } else if (prefix == "GPGSA") {
    DEBUG.println("GPGSA");
  } else {
    DEBUG.println("Unexpected: "+prefix);
  }
}

/* Interruptions */
void serialEventRun(void) {
  if (GPS.available()) _INT_GPS();
}

void _INT_GPS() {
  GREEN;
  while(GPS.available()) {
    char inChar = GPS.read();
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
  RED;
  spin_counter++;
}