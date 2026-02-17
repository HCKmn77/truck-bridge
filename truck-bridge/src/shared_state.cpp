/*
 * Shared state management for multicore tasks 
 * 
 * This module provides functions to safely access and modify shared state
 * variables across multiple tasks running on different cores. 
 * It uses a mutex to ensure that only one task can access the shared state at a time, preventing race conditions and ensuring data integrity.
 */

#include <string.h>
#include "config.h"
#include "logger.h"
#include "shared_state.h"

SemaphoreHandle_t state_mutex = NULL;
ControlState control_state = {
  // Defining the default states of the system 
  .rc_channels = {0,0,0,0,0,0},
  .use_rc_control = false,
  .desired_servo_angle = 90,
  .desired_motor_speed = 0,
  .led_state = false,
  .last_rc_update = 0,
  .last_ros_command = 0
};


bool shared_state_init(void) {
  state_mutex = xSemaphoreCreateMutex();
  return (state_mutex != NULL);
}
