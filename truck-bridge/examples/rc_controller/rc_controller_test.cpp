#include <Arduino.h>

// ======== Hardware config ========

  #define DEBUG_SERIAL Serial2
  #define RC_CH1_PIN 23
  #define RC_CH2_PIN 22
  #define RC_CH3_PIN 21
  #define RC_CH4_PIN 19
  #define RC_CH5_PIN 18
  #define RC_CH6_PIN 5
  #define RC_BATT_PIN 4

#define DEBUG_BAUD 115200

// RC control globals
volatile uint16_t rc_channels[7] = {0, 0, 0, 0, 0, 0, 0};
bool use_rc_control = false;
unsigned long last_rc_update = 0;
const unsigned long RC_TIMEOUT = 500;

// PWM reading helper
uint16_t read_pwm_channel(int pin) {
  return pulseIn(pin, HIGH, 25000);
}

void update_rc_channels() {
  static unsigned long last_read = 0;
  unsigned long now = millis();
  
  // Read RC channels every 20ms (50Hz)
  if (now - last_read >= 20) {
    last_read = now;
    
    // Read all 6 channels
    rc_channels[0] = read_pwm_channel(RC_CH1_PIN);
    rc_channels[1] = read_pwm_channel(RC_CH2_PIN);
    rc_channels[2] = read_pwm_channel(RC_CH3_PIN);
    rc_channels[3] = read_pwm_channel(RC_CH4_PIN);
    rc_channels[4] = read_pwm_channel(RC_CH5_PIN);
    rc_channels[5] = read_pwm_channel(RC_CH6_PIN);
    rc_channels[6] = read_pwm_channel(RC_BATT_PIN);
    
    last_rc_update = now;
  }
}

bool rc_signal_lost() {
  return (millis() - last_rc_update) > RC_TIMEOUT;
}

void print_rc_status() {
  DEBUG_SERIAL.print("CH1:");
  DEBUG_SERIAL.print(rc_channels[0]);
  DEBUG_SERIAL.print("µs | CH2:");
  DEBUG_SERIAL.print(rc_channels[1]);
  DEBUG_SERIAL.print("µs | CH3:");
  DEBUG_SERIAL.print(rc_channels[2]);
  DEBUG_SERIAL.print("µs | CH4:");
  DEBUG_SERIAL.print(rc_channels[3]);
  DEBUG_SERIAL.print("µs | CH5:");
  DEBUG_SERIAL.print(rc_channels[4]);
  DEBUG_SERIAL.print("µs | CH6:");
  DEBUG_SERIAL.print(rc_channels[5]);
  DEBUG_SERIAL.print("µs | BATT:");
  DEBUG_SERIAL.println(rc_channels[6]);
}

void setup() {
  DEBUG_SERIAL.begin(DEBUG_BAUD);
  delay(2000);  // Wait for serial to stabilize
  
  DEBUG_SERIAL.println("\n\n========== RC CHANNEL TEST ==========");
  DEBUG_SERIAL.println("Initializing RC receiver pins...");
  
  // Configure RC input pins
  pinMode(RC_CH1_PIN, INPUT);
  pinMode(RC_CH2_PIN, INPUT);
  pinMode(RC_CH3_PIN, INPUT);
  pinMode(RC_CH4_PIN, INPUT);
  pinMode(RC_CH5_PIN, INPUT);
  pinMode(RC_CH6_PIN, INPUT);
  pinMode(RC_BATT_PIN, INPUT);
  
  DEBUG_SERIAL.println("RC receiver ready!");
}

void loop() {
  // Update RC channels
  update_rc_channels();
  
  // Print status every 500ms
  static unsigned long last_print = 0;
  if (millis() - last_print >= 500) {
    last_print = millis();
    print_rc_status();
  }
  
  delay(10);
}