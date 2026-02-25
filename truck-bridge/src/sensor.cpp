#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include "config.h"
#include "ros_interface.h"
#include "logger.h"
#include "sensor.h"

// BMI270 sensor instance
BMI270 imu;
bool sensor_initialized = false;

bool imu_init(void) {
  LOG_INFO("SENSOR-INIT", "Initializing BMI270 IMU on I2C (SDA=%d, SCL=%d)", BMI270_SDA_PIN, BMI270_SCL_PIN);
  
  // Initialize I2C on custom pins
  Wire.begin(BMI270_SDA_PIN, BMI270_SCL_PIN);
  
  // Check if sensor is connected and initialize
  while(imu.beginI2C(BMI2_I2C_PRIM_ADDR) != BMI2_OK) {
    LOG_ERROR("SENSOR-INIT", "BMI270 not detected at address 0x69");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  
  LOG_INFO("SENSOR-INIT", "BMI270 connected!");
  sensor_initialized = true;
  return true;
}

void read_gyro(float &x, float &y, float &z) {
  // Read gyroscope data from BMI270 (degrees/second)
  x = imu.data.gyroX;
  y = imu.data.gyroY;
  z = imu.data.gyroZ;
}

void read_accel(float &x, float &y, float &z) {
  // Read accelerometer data from BMI270 (in g's)
  x = imu.data.accelX;
  y = imu.data.accelY;
  z = imu.data.accelZ;
}

// Sensor task -----------------------------------------------------------------------------------

void sensor_task(void *pvParameters) {
/* 
  * Sensor task
  * Reads sensor data and publishes to ROS
*/

  LOG_INFO("SENSOR-TASK", "Starting SENSOR-TASK...");
  
  // Try to initialize sensor
  if (!imu_init()) {
    LOG_ERROR("SENSOR-TASK", "Failed to initialize BMI270 - sensor task terminating");
    vTaskDelete(NULL);
    return;
  }
  
  unsigned long last_publish = 0;
  uint32_t publish_failures = 0;
  const uint32_t max_failures = 10;
  
  while (1) {
    unsigned long now = millis();
    
    // Read and publish sensor data at configured interval
    if (ros_is_connected() && (now - last_publish >= GYRO_PUBLISH_INTERVAL)) {
      float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
      float accel_x = 0.0f, accel_y = 0.0f, accel_z = 0.0f;
      
      // Get latest sensor data
      imu.getSensorData();

      read_gyro(gyro_x, gyro_y, gyro_z);
      read_accel(accel_x, accel_y, accel_z);

      // Convert units to ROS standard
      // Gyro: deg/s -> rad/s, Accel: g -> m/s^2
      const float kGToMps2 = 9.80665f;
      gyro_x *= DEG_TO_RAD;
      gyro_y *= DEG_TO_RAD;
      gyro_z *= DEG_TO_RAD;
      accel_x *= kGToMps2;
      accel_y *= kGToMps2;
      accel_z *= kGToMps2;
      
      // Log data
      LOG_DEBUG("SENSOR-TASK", "Gyro[rad/s] - X: %.2f | Y: %.2f | Z: %.2f", gyro_x, gyro_y, gyro_z);
      LOG_DEBUG("SENSOR-TASK", "Accel[m/s²] - X: %.2f | Y: %.2f | Z: %.2f", accel_x, accel_y, accel_z);
      
      // Publish to ROS
      ros_publish_imu(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);
      
      last_publish = now;
      publish_failures = 0;
    } else if (!ros_is_connected()) {
      // Log ROS connection state (only once per interval for verbosity)
      static unsigned long last_warning = 0;
      if (now - last_warning >= 5000) {
        LOG_WARN("SENSOR-TASK", "Waiting for ROS connection...");
        last_warning = now;
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
