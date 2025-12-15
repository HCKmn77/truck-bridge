#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/vector3.h>

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
static rcl_publisher_t gyro_pub;
static geometry_msgs__msg__Vector3 gyro_msg;

static volatile bool gyro_ros_connected = false;

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

void ros_setup_transport(void) {
  //  Setup transport based on build-flag
  #if defined(USE_SERIAL_TRANSPORT)
    LOG_INFO("ROS-INIT-TASK", "Using Serial transport");
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

  #elif defined(USE_WIFI_TRANSPORT)
    LOG_INFO("ROS-INIT-TASK", "Using WiFi transport");
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
  
  #elif defined(USE_ETHERNET_TRANSPORT)
    LOG_INFO("ROS-INIT-TASK", "Using Ethernet transport");
    set_microros_ethernet_transports(ETHERNET_MAC, AGENT_IP, AGENT_PORT);
  #endif
  delay(2000);
}

void ros_setup_init(void) {
  LOG_INFO("ROS-INIT-TASK", "Initializing micro-ROS...");
  allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) { LOG_ERROR("ROS-INIT-TASK", "Support init failed"); return;}

  ret = rclc_node_init_default(&node, "servo_led_node", "", &support);
  if (ret != RCL_RET_OK) { LOG_ERROR("ROS-INIT-TASK", "Node creation failed"); return; }

  ret = rclc_subscription_init_default(
      &led_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "led_control");
  if (ret != RCL_RET_OK) { LOG_ERROR("ROS-INIT-TASK", "LED subscriber failed"); return; }

  ret = rclc_subscription_init_default(
      &servo_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "servo_angle");
  if (ret != RCL_RET_OK) { LOG_ERROR("ROS-INIT-TASK", "Servo subscriber failed"); return; }

  ret = rclc_executor_init(&executor, &support.context, 2, &allocator);
  if (ret != RCL_RET_OK) { LOG_ERROR("ROS-INIT-TASK", "Executor init failed"); return; }

  rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servo_callback, ON_NEW_DATA);
  LOG_INFO("ROS-INIT-TASK", "micro-ROS subscribers ready!");

  // Gyro publisher setup
  ret = rclc_publisher_init_default(
    &gyro_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "gyro_data"
  );
  if (ret != RCL_RET_OK) { 
    LOG_ERROR("ROS-INIT-TASK", "Gyro publisher failed");
    return; 
  }
  LOG_INFO("ROS-INIT-TASK", "Gyro publisher ready!");
  gyro_ros_connected = true;
}

void ros_publish_gyro(float x, float y, float z) {
  gyro_msg.x = x;
  gyro_msg.y = y;
  gyro_msg.z = z;
  rcl_publish(&gyro_pub, &gyro_msg, NULL);
}

void ros_spin_some(uint32_t ms) {
  if (!gyro_ros_connected) return;
  rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ms));
  if (ret != RCL_RET_OK) {
    LOG_ERROR("ROS-INIT-TASK", "Executor spin error: %d", ret);
    gyro_ros_connected = false;
  }
}

bool ros_is_connected(void) {
  return gyro_ros_connected;
}
