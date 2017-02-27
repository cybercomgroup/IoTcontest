#ifndef _leds_h_
#define _leds_h_

#define RED \
do { \
  digitalWrite(LED_RED, LOW);\
  digitalWrite(LED_GREEN, HIGH);\
  digitalWrite(LED_BLUE, HIGH); \
} while (0)

#define BLUE \
do { \
  digitalWrite(LED_RED, HIGH);\
  digitalWrite(LED_GREEN, HIGH);\
  digitalWrite(LED_BLUE, LOW); \
} while (0)

#define GREEN \
do { \
  digitalWrite(LED_RED, HIGH);\
  digitalWrite(LED_GREEN, LOW);\
  digitalWrite(LED_BLUE, HIGH); \
} while (0)

#define WHITE \
do { \
  digitalWrite(LED_RED, LOW);\
  digitalWrite(LED_GREEN, LOW);\
  digitalWrite(LED_BLUE, LOW); \
} while (0)

#define LED_OFF \
do { \
  digitalWrite(LED_RED, HIGH);\
  digitalWrite(LED_GREEN, HIGH);\
  digitalWrite(LED_BLUE, HIGH); \
} while (0)

#endif _leds_h_
