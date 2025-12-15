#include <Arduino.h>
#include "config.h"
#include "ros_interface.h"
#include "logger.h"
#include "sensor.h"

void sensor_task(void *pvParameters) {
  LOG_INFO("SENSOR-TASK", "Starting SENSOR-TASK...");
  
  unsigned long last_publish = 0;
  
  while (1) {
    unsigned long now = millis();
    
    // Publish gyro data at GYRO_PUBLISH_INTERVAL
    if (now - last_publish >= GYRO_PUBLISH_INTERVAL && ros_is_connected()) {
      // Read analog values and convert to appropriate range
      float x = (analogRead(GYRO_X_PIN) - 512) * (500.0f / 1023.0f);
      float y = (analogRead(GYRO_Y_PIN) - 512) * (500.0f / 1023.0f);
      float z = (analogRead(GYRO_Z_PIN) - 512) * (500.0f / 1023.0f);
      LOG_DEBUG("SENSOR-TASK", "Gyro Readings - X: %.2f | Y: %.2f | Z: %.2f", x, y, z);
      
      ros_publish_gyro(x, y, z);
      last_publish = now;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
