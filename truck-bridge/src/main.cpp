// External includes
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Servo.h>

// Project includes
#include "config.h"
#include "logger.h"
#include "output_control.h"
#include "shared_state.h"
#include "rc_input.h"
#include "ros_interface.h"
#include "sensor.h"

/********************************************************************************************************************
*   Studieprojekt: Truck Bridge | Hochschule Esslingen (2025)
*   Autor: Jeremia Haackmann (TIB)
* ------------------------------------------------------------------------------------------------------------------
*   Project description:
*     This firmware enables an ESP32-based Truck to be controlled via a RC controller or ROS commands.
*    
*   Modes:
*      - MANUAL (RC): Direct control via RC transmitter
*      - AUTO (ROS):  Control via ROS messages from a connected ROS2 agent running on a NVIDAA Jetson Nano
*
*   Hardware components:
*     Actuators:
*       - Servo:      Steering control
*       - DC motor:   Throttle control
*     Sensors:
*       - Gyroscope
*       - rpm_sensor: Wheel speed measurement
*     PWN Inputs from RC reciever:
*       - CH1:        Steering
*       - CH2:        Throttle
*       - CH3:        -
*       - CH4:        -
*       - CH5:        -
*       - CH6:        Mode switch (Manual/Auto)
*       - CH_BATT:   Battery voltage monitoring (not implemented)
*     LED indicator:
*       - MODE:       Indicates manual (RC) or automatic (ROS) mode
*       - ROS Status: Indicates connection status to ROS agent
*       - RC Status:  Indicates connection status from RC receiver
*     Communication:
*       - Serial1:    micro-ROS communication with ROS2 agent
*       - Seroal2:    Debug logging
*       - WiFi:       optional micro-ROS transport interface
*
*
* ------------------------------------------------------------------------------------------------------------------
*
*     This is the main entry point for the Truck Bridge firmware.
*     
*     This file initializes hardware, shared state, micro-ROS and RC communication 
*     and distributes the tasks across the ESP32's two cores.
*   
********************************************************************************************************************/

Servo servo;

// ======== Setup ========
void setup() {
  
  // --- Initialize logger ---
  DEBUG_SERIAL.begin(DEBUG_BAUD);
  delay(200);
  
  logger_init(LOG_LEVEL);
  xTaskCreatePinnedToCore(logger_task, "Logger", 2048, NULL, 2, NULL, 0); // Start logger task on core 0
  delay(200);
  LOG_INFO("MAIN-SETUP", "=== System Setup Starting ===");

  // --- Hardware Setup ---
  pinMode(LED_BUILDIN, OUTPUT);
  pinMode(LED_MODE_PIN, OUTPUT);
  pinMode(LED_ROS_PIN, OUTPUT);
  pinMode(LED_RC_PIN, OUTPUT);

  digitalWrite(LED_BUILDIN, LOW);
  digitalWrite(LED_MODE_PIN, LOW);
  digitalWrite(LED_ROS_PIN, LOW);
  digitalWrite(LED_RC_PIN, LOW);
  
  servo.attach(SERVO_PIN);

  // --- RC Receiver Setup ---
  pinMode(RC_CH1_PIN, INPUT);
  pinMode(RC_CH2_PIN, INPUT);
  pinMode(RC_CH3_PIN, INPUT);
  pinMode(RC_CH4_PIN, INPUT);
  pinMode(RC_CH5_PIN, INPUT);
  pinMode(RC_CH6_PIN, INPUT);
  
  // --- Create & init shared state (mutex inside) ---
  if (!shared_state_init()) {
    LOG_ERROR("MAIN-SETUP", "Failed to create shared state mutex!");
    return;
  }
  
  // --- Setup & Initialize micro-ROS (subscriptions, publishers, executor) ---

  ros_setup_transport();
  ros_setup_init();



/**
* ======= Creation of FreeRTOS Tasks ======================================================================================================= 
*    Task priorities: higher number -> higher priority
*    Notation: xTaskCreatePinnedToCore(task function, "name", stack size, parameters, priority, task handle, core id);
* ----------------------------------------------------------------------------------------------------------------------------------------
*    Core 0:
*        - Logger (started at beginning)
*        - ROS Communication
*        - Sensors
*    Core 1:
*        - RC Input
*        - Outputs Control
*==========================================================================================================================================

*/

xTaskCreatePinnedToCore(rc_input_task, "RC_Input", 3072, NULL, 3, NULL, 1);
xTaskCreatePinnedToCore(ros_comm_task, "ROS_Comm", 4096, NULL, 1, NULL, 0);
xTaskCreatePinnedToCore(sensor_task, "Sensors", 3072, NULL, 1, NULL, 0);
xTaskCreatePinnedToCore(output_control_task, "Outputs", 3072, NULL, 2, NULL, 1);

LOG_INFO("MAIN-SETUP", "All tasks created!");
LOG_INFO("MAIN-SETUP", "=== Setup Complete - Running Multi-Core ===");
}

void loop() {
// All work is outsourced to FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}


