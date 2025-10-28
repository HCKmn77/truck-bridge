#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <Servo.h>
#include "../include/secrets.h"


#define DEBUG_PRINT(x) {Serial1.print("DEBUG: "); Serial1.print(x);}
#define DEBUG_PRINTLN(x) {Serial1.print(" "); Serial1.println(x);}
#define CHECK_RCL(ret, msg) if ((ret) != RCL_RET_OK) { DEBUG_PRINTLN(msg); DEBUG_PRINTLN(ret); return; }

#define USE_WIFI_TRANSPORT true

//////////////////// GLOBALS & FUNCTION //////////////////////////////

// Servo on pin 3
Servo my_servo;
const int SERVO_PIN = 3;
int angle = 0;

// ROS 2 node handles
rcl_node_t node;
rcl_subscription_t subscriber;
rclc_executor_t executor;
std_msgs__msg__Int32 received_msg;

rcl_publisher_t publisher;
std_msgs__msg__Int32 pub_msg;

// Callback: set servo angle
void servo_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  DEBUG_PRINTLN("Callback triggered!");
  int angle = msg->data;
  DEBUG_PRINT("Received angle: ");
  DEBUG_PRINTLN(angle);

  angle = constrain(angle, 0, 180);
  my_servo.write(angle);
  DEBUG_PRINT("Servo angle set to: ");
  DEBUG_PRINTLN(angle);
  
  // Publish current angle
  pub_msg.data = angle;
  rcl_ret_t pub_ret = rcl_publish(&publisher, &pub_msg, NULL);
  if (pub_ret != RCL_RET_OK) {
    DEBUG_PRINTLN("Error publishing servo angle");
  }
}

void print_wifi_status() {
  DEBUG_PRINT("SSID: ");
  DEBUG_PRINTLN(WiFi.SSID());
  
  DEBUG_PRINT("IP Address: ");
  DEBUG_PRINTLN(WiFi.localIP());
  
  DEBUG_PRINT("Signal strength (RSSI): ");
  DEBUG_PRINT(WiFi.RSSI());
  DEBUG_PRINTLN(" dBm");
}

void print_ros_status() {
  DEBUG_PRINTLN("\n=== ROS Status ===");
  DEBUG_PRINT("Node valid: "); 
  DEBUG_PRINTLN(rcl_node_is_valid(&node));
  
  // Add detailed ping info
  int ping_ret = rmw_uros_ping_agent(100, 1);
  DEBUG_PRINT("Agent ping ret: ");
  DEBUG_PRINTLN(ping_ret);                  // numeric return
  DEBUG_PRINT("Agent ping status: ");
  DEBUG_PRINTLN(ping_ret == RMW_RET_OK ? "OK" : "FAIL");
  
  DEBUG_PRINT("Subscription valid: ");
  DEBUG_PRINTLN(rcl_subscription_is_valid(&subscriber));
  
  DEBUG_PRINT("Executor initialized: ");
  DEBUG_PRINTLN(&executor != NULL);
  
  DEBUG_PRINT("Topic name: ");
  DEBUG_PRINTLN(rcl_subscription_get_topic_name(&subscriber));
  
  DEBUG_PRINT("System uptime (ms): ");
  DEBUG_PRINTLN(millis());
  DEBUG_PRINTLN("================\n");
}

///////////////////////////// SETUP ///////////////////////////////////
void setup() {
  
  // Setup DEBUG-Serial
  Serial1.begin(115200);
  delay(2000);

  DEBUG_PRINTLN("Serial1-DEBUG connected to servo controller.");
  DEBUG_PRINTLN("Starting setup...");
  

  #if USE_WIFI_TRANSPORT
    // Initialize WiFi transport for micro-ROS
    DEBUG_PRINTLN("Setting WiFi transport for micro-ROS...");
    set_microros_wifi_transports(SECRET_SSID, SECRET_PASS, AGENT_IP, AGENT_PORT);
    DEBUG_PRINTLN("WiFi transport set for micro-ROS.");
  #else 
    // Initialize serial connection for micro-ROS
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    DEBUG_PRINTLN("Serial transport set for micro-ROS.");
  #endif

  DEBUG_PRINTLN("Transport set");

  my_servo.attach(SERVO_PIN);
  DEBUG_PRINTLN("Servo attached");

  delay(2000);

  // Allocators and support
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  DEBUG_PRINTLN("Initializing micro-ROS...");
  
  // Wait for agent to become available
  DEBUG_PRINTLN("Waiting for agent connection...");
  bool agent_connected = false;
  for(int i=0; i<10; i++) {
    agent_connected = rmw_uros_ping_agent(1000, 1) == RMW_RET_OK;
    if(agent_connected) {
      DEBUG_PRINTLN("Agent found!");
      break;
    }
    DEBUG_PRINT("Attempt "); 
    DEBUG_PRINT(i+1); 
    DEBUG_PRINTLN("/10 - Agent not found, retrying...");
    delay(1000);
  }
  
  if(!agent_connected) {
    DEBUG_PRINTLN("Agent connection failed after 10 attempts");
    return;
  }

  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) {
    DEBUG_PRINT("Support init failed with error code: ");
    DEBUG_PRINTLN(ret);
    if (ret == RCL_RET_ERROR) DEBUG_PRINTLN("General error in rcl_init");
    if (ret == RCL_RET_TIMEOUT) DEBUG_PRINTLN("Operation timed out");
    if (ret == RCL_RET_INVALID_ARGUMENT) DEBUG_PRINTLN("Invalid arguments passed");
    return;
  }
  DEBUG_PRINTLN("micro-ROS support initialized.");

  // Create node
  ret = rclc_node_init_default(&node, "servo_controller_node", "", &support);
  CHECK_RCL(ret, "Node creation failed");

  // Create subscriber
  ret = rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "servo_angle"
  );
  CHECK_RCL(ret, "Subscription creation failed");

  // Create publisher
  ret = rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "servo_angle_state"
  );
  CHECK_RCL(ret, "Publisher creation failed");

  // Initialize executor
  ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
  CHECK_RCL(ret, "Executor initialization failed");
  ret = rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &received_msg,
    &servo_callback,
    ON_NEW_DATA
  );
  CHECK_RCL(ret, "Adding subscription to executor failed");

  DEBUG_PRINTLN("micro-ROS Servo Subscriber Ready.");

  #if USE_WIFI_TRANSPORT
    print_wifi_status();
  #endif

  // Add after micro-ROS initialization
  DEBUG_PRINTLN("\n=== ROS Configuration ===");
  DEBUG_PRINT("RMW_IMPLEMENTATION: ");
  DEBUG_PRINTLN(rmw_get_implementation_identifier());
  DEBUG_PRINT("Node name: ");
  DEBUG_PRINTLN("servo_controller_node");
  DEBUG_PRINT("Topic name: ");
  DEBUG_PRINTLN("servo_angle");
  DEBUG_PRINT("Message type: ");
  DEBUG_PRINTLN("std_msgs/msg/Int32");
  DEBUG_PRINTLN("======================\n");
}


///////////////////////////// LOOP ///////////////////////////////////
void loop() {
  // Add at start of loop
  static unsigned long last_ros_check = 0;
  if (millis() - last_ros_check > 2000) {  // Check every 2 seconds
    last_ros_check = millis();
    print_ros_status();
  }
  delay(100);

  rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  
  // if (ret != RCL_RET_OK) {
  //   DEBUG_PRINT("Error in executor spin. Error code: ");
  //   DEBUG_PRINTLN(ret);
    
  //   // More detailed error reporting
  //   switch(ret) {
  //     case RCL_RET_ERROR:
  //       DEBUG_PRINTLN("General error occurred");
  //       break;
  //     case RCL_RET_TIMEOUT:
  //       DEBUG_PRINTLN("Operation timed out");
  //       break;
  //     case RCL_RET_INVALID_ARGUMENT:
  //       DEBUG_PRINTLN("Invalid argument");
  //       break;
  //     default:
  //       DEBUG_PRINT("Unknown error code: ");
  //       DEBUG_PRINTLN(ret);
  //   }
// }
    
    // // Check connection state
    // DEBUG_PRINT("Node is alive: ");
    // DEBUG_PRINTLN(rcl_node_is_valid(&node) ? "yes" : "no");
    // DEBUG_PRINT("Agent reachable: ");
    // DEBUG_PRINTLN(rmw_uros_ping_agent(100, 1) == RMW_RET_OK ? "yes" : "no");

   if (Serial1.available()) {
    String input = Serial1.readStringUntil('\n');
    input.trim(); // Remove any extra whitespace

    if (input.length() > 0) {
      int newAngle = input.toInt();

      // Check if input is valid (0-180)
      if (newAngle >= 0 && newAngle <= 180) {
        angle = newAngle;
        my_servo.write(angle);
        DEBUG_PRINT("DEBUG: Moved to: ");
        DEBUG_PRINTLN(angle);
      } else {
        DEBUG_PRINTLN("DEBUG: Invalid angle! Please enter 0-180.");
      }
    }
  }
}