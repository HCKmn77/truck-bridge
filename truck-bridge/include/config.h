#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

/**
 * Project-wide configuration settings
 * This file defines hardware pin assignments, timing intervals, 
 * and other constants used across the project.
 */


// ======== Hardware specific config ========
// Check official NodeMCU   ESP32 pinout for reference: 
// https://joy-it.net/files/files/Produkte/SBC-NodeMCU-ESP32-C/SBC-NodeMCU-ESP32-C_Manual-EN_2025-01-17.pdf

#if defined(TARGET_ESP32)

// Logging configuration
#define DEBUG_SERIAL Serial2
#define DEBUG_BAUD 115200

  // RC receiver
  #define RC_CH1_PIN 23   // Servo control
  #define RC_CH2_PIN 22   // Motor control
  #define RC_CH3_PIN 0    // Not connected
  #define RC_CH4_PIN 0    // Not connected
  #define RC_CH5_PIN 0    // Not connected
  #define RC_CH6_PIN 5    // Mode switch (<1500 = RC, >1500 = AUTO)
  #define RC_BATT_PIN 0   // Not connected

  // actuators
  #define MOTOR_PIN 12
  #define SERVO_PIN 15
  
  // Status LEDs
  #define LED_MODE_PIN 18   // Indicates RC/ROS mode
  #define LED_RC_PIN 19     // Indicates RC signal status
  #define LED_ROS_PIN 21    // Indicates ROS connection status

  // sensors
  #define BMI270_SDA_PIN 2
  #define BMI270_SCL_PIN 4

  #define WHEEL_RPM_PIN 39
#endif

// Timing intervals & constants
#define DEBUG_BAUD 115200
#define GYRO_PUBLISH_INTERVAL 100
#define SERVO_FEEDBACK_PUBLISH_INTERVAL 100
#define RC_UPDATE_INTERVAL 20    // 50Hz
#define OUTPUT_UPDATE_INTERVAL 20  // 50Hz

// TODO: Use these timeouts to check connection status
#define RC_CONNECTION_TIMEOUT 500
#define ROS_CONNECTION_TIMEOUT 500
#define ROS_RECONNECT_INTERVAL 2000

#endif // PROJECT_CONFIG_H
