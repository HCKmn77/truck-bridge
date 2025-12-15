#include <Arduino.h>

#include "ros_interface.h"
#include "logger.h"
#include "config.h"

void ros_comm_task(void *pvParameters) {
  LOG_INFO("ROS-COMM-TASK", "Starting ROS-COM-TASK ...");
  unsigned long last_reconnect = 0;
  
  while (1) {
    // Try to reconnect if disconnected
    if (!ros_is_connected()) {
      if (millis() - last_reconnect >= ROS_RECONNECT_INTERVAL) {
        LOG_INFO("ROS-COMM-TASK", "Attempting to reconnect...");
        // attempt a quick spin which will succeed only if underlying rcl context is OK
        ros_spin_some(10);
        last_reconnect = millis();
      }
    } else {
      // Spin executor to process ROS messages
      ros_spin_some(100);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
