#include <Arduino.h>
#include <WiFiNINA.h>
#include <Servo.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include "../include/secrets.h"

// ======== Hardware Pins ========
#define LED_PIN   LED_BUILTIN
#define SERVO_PIN 3

// ======== Debug Macros (Serial1) ========
#define DEBUG_BAUD     115200
#define DEBUG_PRINT(x)   { Serial1.print("[DBG] "); Serial1.print(x); }
#define DEBUG_PRINTLN(x) { Serial1.print("[DBG] "); Serial1.println(x); }

// ======== micro-ROS Globals ========
rcl_subscription_t led_sub;
rcl_subscription_t servo_sub;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
std_msgs__msg__Bool led_msg;
std_msgs__msg__Int32 servo_msg;

// ======== Servo Object ========
Servo servo;

// ======== Callbacks ========
void led_callback(const void * msgin)
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  digitalWrite(LED_PIN, msg->data ? HIGH : LOW);
  DEBUG_PRINT("LED set to: "); DEBUG_PRINTLN(msg->data ? "ON" : "OFF");
}

void servo_callback(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int angle = constrain(msg->data, 0, 180);
  servo.write(angle);
  DEBUG_PRINT("Servo angle: "); DEBUG_PRINTLN(angle);
}

// ======== Setup ========
void setup()
{
  // --- Debug UART ---
  Serial1.begin(DEBUG_BAUD);
  delay(1000);
  DEBUG_PRINTLN("Booting micro-ROS Wi-Fi node...");

  // --- Hardware ---
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  servo.attach(SERVO_PIN);

  // --- Wi-Fi Connection ---
  DEBUG_PRINTLN("Connecting to Wi-Fi...");
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    status = WiFi.begin(SECRET_SSID, SECRET_PASS);
    delay(1000);
    DEBUG_PRINT(".");
  }
  DEBUG_PRINTLN("\nWi-Fi connected!");
  DEBUG_PRINT("Local IP: "); DEBUG_PRINTLN(WiFi.localIP());

  // --- micro-ROS Transport ---
  DEBUG_PRINTLN("Setting up micro-ROS Wi-Fi transport...");
  set_microros_wifi_transports(SECRET_SSID, SECRET_PASS, AGENT_IP, AGENT_PORT);
  delay(2000); // allow sockets to settle

  // --- micro-ROS Init ---
  allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) { DEBUG_PRINTLN("Support init failed"); return; }

  ret = rclc_node_init_default(&node, "rp2040_servo_led_node", "", &support);
  if (ret != RCL_RET_OK) { DEBUG_PRINTLN("Node creation failed"); return; }

  ret = rclc_subscription_init_default(
      &led_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "led_control");
  if (ret != RCL_RET_OK) { DEBUG_PRINTLN("LED subscriber failed"); return; }

  ret = rclc_subscription_init_default(
      &servo_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "servo_angle");
  if (ret != RCL_RET_OK) { DEBUG_PRINTLN("Servo subscriber failed"); return; }

  ret = rclc_executor_init(&executor, &support.context, 2, &allocator);
  if (ret != RCL_RET_OK) { DEBUG_PRINTLN("Executor init failed"); return; }

  rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servo_callback, ON_NEW_DATA);

  DEBUG_PRINTLN("micro-ROS subscribers ready!");
}

// ======== Main Loop ========
void loop()
{
  rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  if (ret != RCL_RET_OK) {
    DEBUG_PRINT("Executor error: "); DEBUG_PRINTLN(ret);
  }
  delay(10);
}
