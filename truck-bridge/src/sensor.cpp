#include <Arduino.h>
#include "config.h"
#include "ros_interface.h"
#include "logger.h"
#include "sensor.h"

void read_gyro(float &x, float &y, float &z) {
  // Read analog values and convert to appropriate range
  x = (analogRead(GYRO_X_PIN) - 512) * (500.0f / 1023.0f);
  y = (analogRead(GYRO_Y_PIN) - 512) * (500.0f / 1023.0f);
  z = (analogRead(GYRO_Z_PIN) - 512) * (500.0f / 1023.0f); 
}

void sensor_task(void *pvParameters) {
/* 
  * Sensor task
  * Reads sensor data and publishes to ROS
*/

  LOG_INFO("SENSOR-TASK", "Starting SENSOR-TASK...");
  
  unsigned long last_publish = 0;
  
  while (1) {
    unsigned long now = millis();
    
    // Gyro sensor reading
    float x, y, z;    
    if (ros_is_connected() && (now - last_publish >= GYRO_PUBLISH_INTERVAL)) {
      read_gyro(x, y, z);
      LOG_DEBUG("SENSOR-TASK", "Gyro Readings - X: %.2f | Y: %.2f | Z: %.2f", x, y, z);
     
      // Gyro publish readings to ROS topic
      ros_publish_gyro(x, y, z);
     
      last_publish = now;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
