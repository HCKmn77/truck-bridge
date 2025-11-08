#include <Arduino.h>
#include <Servo.h>
#include <micro_ros_platformio.h>

#include <geometry_msgs/msg/vector3.h>
#include <Wire.h>

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
  #define GYRO_X_PIN 34  
  #define GYRO_Y_PIN 35
  #define GYRO_Z_PIN 36

#elif defined(TARGET_RP2040)
  #define LED_PIN LED_BUILDIN
  #define DEBUG_SERIAL Serial1
  #define SERVO_PIN 20
#endif

#define DEBUG_BAUD 115200
#define GYRO_PUBLISH_INTERVAL 100


// ======== micro-ROS Globals ========
rcl_subscription_t led_sub;
rcl_subscription_t servo_sub;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
std_msgs__msg__Bool led_msg;
std_msgs__msg__Int32 servo_msg;
rcl_publisher_t gyro_pub;
geometry_msgs__msg__Vector3 gyro_msg;

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

void read_and_publish_gyro() {
  static unsigned long last_publish = 0;
  unsigned long now = millis();
  
  if (now - last_publish >= GYRO_PUBLISH_INTERVAL) {
    // Read analog values and convert to appropriate range
    // Assuming 0-1023 ADC range mapped to +/- 250 degrees/sec
    float x = (analogRead(GYRO_X_PIN) - 512) * (500.0f / 1023.0f);
    float y = (analogRead(GYRO_Y_PIN) - 512) * (500.0f / 1023.0f);
    float z = (analogRead(GYRO_Z_PIN) - 512) * (500.0f / 1023.0f);
    
    // Update message
    gyro_msg.x = x;
    gyro_msg.y = y;
    gyro_msg.z = z;
    
    // Publish
    rcl_ret_t pub_ret = rcl_publish(&gyro_pub, &gyro_msg, NULL);
    if (pub_ret != RCL_RET_OK) {
      DEBUG_PRINT("Gyro publish failed: "); 
      DEBUG_PRINTLN(pub_ret);
    }
    
    last_publish = now;
  }
}

// ======== Setup ========
void setup()
{
  DEBUG_SERIAL.begin(DEBUG_BAUD);
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
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
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

  // Gyro publisher setup
  ret = rclc_publisher_init_default(
    &gyro_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "gyro_data"
  );
  if (ret != RCL_RET_OK) { 
    DEBUG_PRINTLN("Gyro publisher failed"); 
    return; 
  }
  analogReadResolution(10);
  DEBUG_PRINTLN("Gyro publisher ready!");
  DEBUG_PRINTLN("micro-ROS node setup complete.");
}

// ======== Main Loop ========
void loop()
{
  // // quick check: is agent reachable? avoid spinning when transport down
  // int ping = rmw_uros_ping_agent(100, 1);
  // if (ping != RMW_RET_OK) {
  //   DEBUG_PRINT("Agent ping failed (skip spin). ret=");
  //   DEBUG_PRINTLN(ping);

  //   delay(1000);
  //   return;
  // }
  read_and_publish_gyro();

  rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}
