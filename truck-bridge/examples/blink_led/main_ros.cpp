#include <Arduino.h>
#include <WiFiNINA.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include "../include/secrets.h"

rcl_subscription_t subscriber;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
std_msgs__msg__Bool led_msg;

// LED pin for Nano RP2040 Connect
const int LED_PIN = LED_BUILTIN;

// Callback for ROS message
void led_callback(const void * msgin)
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  digitalWrite(LED_PIN, msg->data ? HIGH : LOW);
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial1.begin(115200);
  delay(2000);
  Serial1.println("Connecting to Wi-Fi...");

  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED)
  {
    status = WiFi.begin(SECRET_SSID, SECRET_PASS);
    delay(1000);
    Serial1.print(".");
  }
  Serial1.println("\nWi-Fi connected!");
  Serial1.print("Local IP: ");
  Serial1.println(WiFi.localIP());

  Serial1.println("Setting micro-ROS Wi-Fi transport...");
  set_microros_wifi_transports(SECRET_SSID, SECRET_PASS, AGENT_IP, AGENT_PORT);
  delay(2000);

  allocator = rcl_get_default_allocator();
  rcl_ret_t ret;

  ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) {
    Serial1.println("Failed to init micro-ROS support");
    return;
  }

  ret = rclc_node_init_default(&node, "rp2040_led_node", "", &support);
  if (ret != RCL_RET_OK) {
    Serial1.println("Failed to create node");
    return;
  }

  ret = rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "led_control");
  if (ret != RCL_RET_OK) {
    Serial1.println("Failed to create subscriber");
    return;
  }

  ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
  if (ret != RCL_RET_OK) {
    Serial1.println("Failed to init executor");
    return;
  }

  ret = rclc_executor_add_subscription(&executor, &subscriber, &led_msg, &led_callback, ON_NEW_DATA);
  if (ret != RCL_RET_OK) {
    Serial1.println("Failed to add subscription");
    return;
  }

  Serial1.println("micro-ROS LED subscriber ready!");
}

void loop()
{
  rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  if (ret != RCL_RET_OK)
  {
    Serial1.print("Executor error: ");
    Serial1.println(ret);
  }
  delay(10);
}
