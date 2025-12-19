#include <Arduino.h>
#include <string.h>
#include "shared_state.h"
#include "config.h"
#include "logger.h"
#include "rc_input.h"

// Helper to read PWM on a pin
static uint16_t read_pwm_channel(int pin) {
  return pulseIn(pin, HIGH, 25000);  // timeout 25ms
}

void rc_input_task(void *pvParameters) {
  LOG_INFO("RC-TASK", "Starting RC-TASK...");
  
  unsigned long last_debug = 0;
  
  while (1) {
    unsigned long now = millis();
    // Read RC channels at RC_UPDATE_INTERVAL
    uint16_t temp_channels[6];
    temp_channels[0] = read_pwm_channel(RC_CH1_PIN);
    temp_channels[1] = read_pwm_channel(RC_CH2_PIN);
    temp_channels[2] = read_pwm_channel(RC_CH3_PIN);
    temp_channels[3] = read_pwm_channel(RC_CH4_PIN);
    temp_channels[4] = read_pwm_channel(RC_CH5_PIN);
    temp_channels[5] = read_pwm_channel(RC_CH6_PIN);
    
    LOG_DEBUG("RC-TASK", "RC Channels Read: CH1:%u CH2:%u CH3:%u CH4:%u CH5:%u CH6:%u",
      temp_channels[0], temp_channels[1], temp_channels[2],
      temp_channels[3], temp_channels[4], temp_channels[5]);

    // Update shared state
    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      memcpy(control_state.rc_channels, temp_channels, sizeof(temp_channels));
      control_state.use_rc_control = (temp_channels[5] < 1500);
      control_state.last_rc_update = now;
      xSemaphoreGive(state_mutex);    
    }
    else
    {
        // Timeout reached - failed to take mutex
        TaskHandle_t ownerHandle = xSemaphoreGetMutexHolder(state_mutex);
        if (ownerHandle != NULL) {
            // Get the name of the task holding the lock
            const char* ownerName = pcTaskGetName(ownerHandle);
            LOG_ERROR("RC-TASK", "Mutex held by task: %s", ownerName);
        } 
        else {
            LOG_ERROR("RC-TASK", "Mutex not held, but something went wrong taking it.");
        }
    }
       
    // Debug output every 500ms
    // only log when log level is debug
    if ((LOG_LEVEL <= 0) && (now - last_debug >= 1000)) {
      last_debug = now;
      if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        LOG_DEBUG("RC-TASK", "Status: RC:%s | Mode:%s | CH1:%u CH2:%u CH6:%u",
          shared_rc_signal_lost() ? "LOST" : "OK",
          control_state.use_rc_control ? "RC" : "AUTO",
          control_state.rc_channels[0],
          control_state.rc_channels[1],
          control_state.rc_channels[5]);
        xSemaphoreGive(state_mutex);
      } 
      else {
        LOG_ERROR("RC-TASK", "Failed to acquire state_mutex within 200ms to print Status update!");
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(RC_UPDATE_INTERVAL));
  }
}
