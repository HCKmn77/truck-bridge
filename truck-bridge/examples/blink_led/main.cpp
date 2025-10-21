#include <Arduino.h>

// Blink built-in LED on Arduino Uno (pin 13)

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                         // wait for 500 milliseconds
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(500);                         // wait for 500 milliseconds
}
