/**
 * 
 * 
 * 
 */

#include <string.h>

#include "config.h"
#include "logger.h"
#include "shared_state.h"

ControlState control_state = {
  .rc_channels = {0,0,0,0,0,0},
  .use_rc_control = false,
  .desired_servo_angle = 90,
  .desired_motor_speed = 0,
  .led_state = false,
  .last_rc_update = 0,
  .last_ros_command = 0
};

SemaphoreHandle_t state_mutex = NULL;

bool shared_state_init(void) {
  state_mutex = xSemaphoreCreateMutex();
  return (state_mutex != NULL);
}

bool shared_rc_signal_lost(void) {
  if (state_mutex == NULL) {
    // pessimistic: assume lost
    LOG_WARN("SHARED-STATE", "state_mutex==NULL, assume RC signal LOST!");
    return true;
  }
  bool lost = false;
  if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(400)) == pdTRUE) {
    LOG_DEBUG("SHARED-STATE", "Checking RC signal loss. Last update: %lu ms ago",
      millis() - control_state.last_rc_update);
    unsigned long now = millis();
    lost = (now - control_state.last_rc_update) > 500;
    if(lost){
      LOG_WARN("SHARED-STATE", "RC signal LOST!");
    }
    xSemaphoreGive(state_mutex);
  }
  else
    {
        // Timeout reached - failed to take mutex
        TaskHandle_t ownerHandle = xSemaphoreGetMutexHolder(state_mutex);
        if (ownerHandle != NULL) {
            // Get the name of the task holding the lock
            const char* ownerName = pcTaskGetName(ownerHandle);
            LOG_ERROR("SHARED-TASK", "Timeout of 400ms reached. Mutex held by task: %s", ownerName);
        } 
        else {
            LOG_ERROR("SHARED-TASK", "Mutex not held, but something went wrong taking it.");
        }
    }
  return lost;
}
