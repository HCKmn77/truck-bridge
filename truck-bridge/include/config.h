#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

/**
 * Project-wide configuration settings
 * 
 */


#include <Arduino.h>


// ======== Hardware specific config ========
#if defined(TARGET_ESP32)

// Logging configuration
#define DEBUG_SERIAL Serial2
#define DEBUG_BAUD 115200

  // RC receiver
  #define RC_CH1_PIN 23   // Servo control
  #define RC_CH2_PIN 22   // Motor control
  #define RC_CH3_PIN 21 
  #define RC_CH4_PIN 19
  #define RC_CH5_PIN 18
  #define RC_CH6_PIN 5    // Mode switch (<1500 = RC, >1500 = AUTO)
  #define RC_BATT_PIN 4   // Battery voltage monitoring

  // actuators
  #define MOTOR_PIN 12
  #define SERVO_PIN 13
  
  // Status LEDs
  #define LED_BUILDIN 2     // Built-in LED on ESP32 board
  #define LED_MODE_PIN 12   // Indicates RC/ROS mode
  #define LED_RC_PIN 14     // Indicates RC signal status
  #define LED_ROS_PIN 27    // Indicates ROS connection status

  // sensors
  #define GYRO_X_PIN 34  
  #define GYRO_Y_PIN 35
  #define GYRO_Z_PIN 36
  #define WHEEL_RPM_PIN 39
#endif

// Timing intervals & constants
#define DEBUG_BAUD 115200
#define GYRO_PUBLISH_INTERVAL 100
#define RC_UPDATE_INTERVAL 20    // 50Hz
#define OUTPUT_UPDATE_INTERVAL 20  // 50Hz

// TODO: Use these timeouts to check connection status
#define RC_CONNECTION_TIMEOUT 500
#define ROS_CONNECTION_TIMEOUT 500
#define ROS_RECONNECT_INTERVAL 2000

#endif // PROJECT_CONFIG_H
