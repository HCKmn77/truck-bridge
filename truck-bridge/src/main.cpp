#include <Arduino.h>
#include <Servo.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include "../include/secrets.h"

// ======== Debug Macros ========

#define DEBUG_PRINT(x)   { DEBUG_SERIAL.print("[DEBUG] "); DEBUG_SERIAL.print(x); }
#define DEBUG_PRINTLN(x) { DEBUG_SERIAL.print("[DEBUG] "); DEBUG_SERIAL.println(x); }

// ======== Hardware specific config ========
#if defined(TARGET_ESP32)
  #define LED_PIN   2
  #define DEBUG_SERIAL Serial2
  #define SERVO_PIN 13

#elif defined(TARGET_RP2040)
  #define LED_PIN LED_BUILDIN
  #define DEBUG_SERIAL Serial1
  #define SERVO_PIN 20
#endif

#define DEBUG_BAUD 115200



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
  DEBUG_PRINTLN("LED callback triggered");
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  digitalWrite(LED_PIN, msg->data ? HIGH : LOW);
  DEBUG_PRINT("LED set to: "); DEBUG_PRINTLN(msg->data ? "ON" : "OFF");
}

void servo_callback(const void * msgin)
{
  DEBUG_PRINTLN("Servo callback triggered");
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int angle = constrain(msg->data, 0, 180);
  servo.write(angle);
  DEBUG_PRINT("Servo angle: "); DEBUG_PRINTLN(angle);
}

// ======== Setup ========
void setup()
{
  DEBUG_SERIAL.begin(DEBUG_BAUD);
  delay(1000);
  DEBUG_PRINTLN("Booting micro-ROS node...");

  // --- Hardware Setup ---
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  servo.attach(SERVO_PIN);

  // --- micro-ROS Transport mode ---
  
  #if defined(USE_SERIAL_TRANSPORT)
    DEBUG_PRINTLN("Using Serial transport");
    DEBUG_PRINTLN("Setting up micro-ROS Serial transport...");
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
  #elif defined(USE_WIFI_TRANSPORT)
    DEBUG_PRINTLN("Using WiFi transport");
    DEBUG_PRINTLN("Setting up micro-ROS WiFi transport...");
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
  #elif defined(USE_ETHERNET_TRANSPORT)
    DEBUG_PRINTLN("Using Ethernet transport");
    DEBUG_PRINTLN("Setting up micro-ROS Ethernet transport...");
    set_microros_ethernet_transports(ETHERNET_MAC, AGENT_IP, AGENT_PORT);
  #endif
  
  delay(2000); // allow sockets to settle

  // --- micro-ROS Init ---
  allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) { DEBUG_PRINTLN("Support init failed"); return; }

  ret = rclc_node_init_default(&node, "servo_led_node", "", &support);
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
  // quick check: is agent reachable? avoid spinning when transport down
  int ping = rmw_uros_ping_agent(100, 1);
  if (ping != RMW_RET_OK) {
    DEBUG_PRINT("Agent ping failed (skip spin). ret=");
    DEBUG_PRINTLN(ping);

    delay(1000);
    return;
  }

  rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  if (ret != RCL_RET_OK) {
    DEBUG_PRINT("Executor error: ");
    DEBUG_PRINTLN(ret);
    // show a second ping for correlation
    DEBUG_PRINT("Agent ping after error: ");
    DEBUG_PRINTLN(rmw_uros_ping_agent(100, 1));
    delay(500);
  }
  delay(10);
}
