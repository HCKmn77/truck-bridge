#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>

// ESP32 specific includes for WiFi/BT control
#ifdef TARGET_ESP32
  #include <WiFi.h>
  #include "esp_bt.h"
#endif

#include "ros_interface.h"
#include "shared_state.h"
#include "logger.h"
#include "secrets.h"

// Internal micro-ROS objects (translation unit scope)
static rcl_subscription_t led_sub;
static rcl_subscription_t servo_sub;
static rcl_node_t node;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rclc_executor_t executor;
static std_msgs__msg__Bool led_msg;
static std_msgs__msg__Int32 servo_msg;
static rcl_publisher_t imu_pub;
static sensor_msgs__msg__Imu imu_msg;
static rcl_publisher_t servo_feedback_pub;
static std_msgs__msg__Int32 servo_feedback_msg;

static volatile bool ros_connected = false;

// ROS Callbacks and helper functions -----------------------------------

void led_callback(const void * msgin)
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  
  if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
    control_state.led_state = msg->data;
    control_state.last_ros_command = millis();
    xSemaphoreGive(state_mutex);
  }
  LOG_INFO("ROS-INIT-TASK", "LED set to: %s", msg->data ? "ON" : "OFF");
}

void servo_callback(const void * msgin) 
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int angle = constrain(msg->data, 0, 180);
  
  if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
    if (!control_state.use_rc_control) {
      control_state.desired_servo_angle = angle;
      control_state.last_ros_command = millis();
      LOG_INFO("ROS-INIT-TASK", "Servo angle (ROS): %d", angle);
    } else {
      LOG_WARN("ROS-INIT-TASK", "Servo: RC control active, ignoring ROS command");
    }
    xSemaphoreGive(state_mutex);
  }
}

// Publish IMU data to ROS topic
void ros_publish_imu(float gyro_x, float gyro_y, float gyro_z, float accel_x, float accel_y, float accel_z) {
  // Set angular velocity (gyroscope)
  imu_msg.angular_velocity.x = gyro_x;
  imu_msg.angular_velocity.y = gyro_y;
  imu_msg.angular_velocity.z = gyro_z;
  
  // Set linear acceleration
  imu_msg.linear_acceleration.x = accel_x;
  imu_msg.linear_acceleration.y = accel_y;
  imu_msg.linear_acceleration.z = accel_z;
  
  // Set timestamp
  imu_msg.header.stamp.sec = millis() / 1000;
  imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
  
  // Note: Orientation not computed on ESP32 (will be computed by ROS node)
  // Set covariance to -1 to indicate unknown covariance
  imu_msg.orientation_covariance[0] = -1.0;
  imu_msg.angular_velocity_covariance[0] = -1.0;
  imu_msg.linear_acceleration_covariance[0] = -1.0;
  
  rcl_publish(&imu_pub, &imu_msg, NULL);
}

// Publish servo angle feedback to ROS topic
void ros_publish_servo_feedback(int16_t angle) {
  servo_feedback_msg.data = angle;
  rcl_publish(&servo_feedback_pub, &servo_feedback_msg, NULL);
}

void ros_spin_some(uint32_t ms) {
  rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ms));
  if (ret != RCL_RET_OK) {LOG_ERROR("ROS-INIT-TASK", "Executor spin error: %d", ret);}
}

void set_ros_connected(bool connected) {
  ros_connected = connected;
}

bool ros_is_connected(void) {
    return ros_connected;
}

// ROS Setup functions ---------------------------------------------

void ros_setup_transport(void) {
  // Setup transport based on build-flag defined in platformio.ini
  #if defined(USE_SERIAL_TRANSPORT)
    LOG_INFO("ROS-INIT-TASK", "Using Serial transport");
    
    // Disable WiFi and Bluetooth to save power (not needed for serial)
    #ifdef TARGET_ESP32
      WiFi.mode(WIFI_OFF);
      btStop();
      LOG_INFO("ROS-INIT-TASK", "WiFi and Bluetooth disabled");
    #endif
    
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

  #elif defined(USE_WIFI_TRANSPORT)
    LOG_INFO("ROS-INIT-TASK", "Using WiFi transport");
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
  
  #elif defined(USE_ETHERNET_TRANSPORT)
    LOG_INFO("ROS-INIT-TASK", "Using Ethernet transport");
    set_microros_ethernet_transports(ETHERNET_MAC, AGENT_IP, AGENT_PORT);
  #endif
  // wait to establish connection
  delay(2000);
}

void ros_setup_init(void) {
  LOG_INFO("ROS-INIT-TASK", "Initializing micro-ROS...");
  allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) { LOG_ERROR("ROS-INIT-TASK", "Support init failed"); return;}


  // Node init ----------------------------------------------------

  ret = rclc_node_init_default(&node, "servo_node", "", &support);
  if (ret != RCL_RET_OK) { LOG_ERROR("ROS-INIT-TASK", "Node creation failed"); return; }


  // Subsciber init -----------------------------------------------

  // LED subscriber setup
  ret = rclc_subscription_init_default(
      &led_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "led_control");
  if (ret != RCL_RET_OK) { LOG_ERROR("ROS-INIT-TASK", "LED subscriber failed"); return; }

  // Servo subscriber setup
  ret = rclc_subscription_init_default(
      &servo_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "servo_angle/cmd");
  if (ret != RCL_RET_OK) { LOG_ERROR("ROS-INIT-TASK", "Servo subscriber failed"); return; }

  
  // Executor init ----------------------------------------------

  // Executor coordinates callbacks and handles incoming messages
  ret = rclc_executor_init(&executor, &support.context, 2, &allocator);
  if (ret != RCL_RET_OK) { LOG_ERROR("ROS-INIT-TASK", "Executor init failed"); return; }
  rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servo_callback, ON_NEW_DATA);
  LOG_INFO("ROS-INIT-TASK", "micro-ROS subscribers ready!");

  // Publisher init ----------------------------------------------
  
  // IMU publisher setup
  ret = rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data"
  );
  if (ret != RCL_RET_OK) { 
    LOG_ERROR("ROS-INIT-TASK", "IMU publisher failed");
    return; 
  }
  LOG_INFO("ROS-INIT-TASK", "IMU publisher ready!");


  // Servo state publisher setup
  ret = rclc_publisher_init_default(
    &servo_feedback_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "servo_angle/state"
  );
  if (ret != RCL_RET_OK) { 
    LOG_ERROR("ROS-INIT-TASK", "Servo state publisher failed");
    return; 
  }
  LOG_INFO("ROS-INIT-TASK", "Servo state publisher ready!");
}
