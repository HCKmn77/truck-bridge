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

// ======== Hardware specific config ========
#if defined(TARGET_ESP32)
  #define LED_PIN   2
  #define DEBUG_SERIAL Serial2
  
  // RC reciever
  // Input pins from the RadioMaster receiver
  #define RC_CH1_PIN 23   // Servo control
  #define RC_CH2_PIN 22   // Motor control
  #define RC_CH3_PIN 21 
  #define RC_CH4_PIN 19
  #define RC_CH5_PIN 18
  #define RC_CH6_PIN 5    // Mode switch (<1500 = RC, >1500 = AUTO)
  #define RC_BATT_PIN 4   // Battery voltage monitoring

  // aktuators
  #define MOTOR_PIN 12
  #define SERVO_PIN 13

  // sensors
  #define GYRO_X_PIN 34  
  #define GYRO_Y_PIN 35
  #define GYRO_Z_PIN 36
  #define WHEEL_RPM_PIN 39

#elif defined(TARGET_RP2040)
  #define LED_PIN LED_BUILDIN
  #define DEBUG_SERIAL Serial1
  #define SERVO_PIN 20
#endif

#define DEBUG_BAUD 115200
#define GYRO_PUBLISH_INTERVAL 100

Servo servo;

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

// Add reconnection tracking
bool ros_connected = false;
unsigned long last_reconnect_attempt = 0;
const unsigned long RECONNECT_INTERVAL = 2000;  // Try reconnect every 2 seconds

volatile uint16_t rc_channels[6] = {0, 0, 0, 0, 0, 0}; // RC channel values in microseconds
bool use_rc_control = false;  // false = AUTO, true = RC
unsigned long last_rc_update = 0;
const unsigned long CONNECTTION_TIMEOUT = 500;  // ms - consider lost after this time

bool rc_signal_lost() {
  return (millis() - last_rc_update) > CONNECTTION_TIMEOUT;
}

bool ros_signal_lost() {
  if (rmw_uros_ping_agent(CONNECTTION_TIMEOUT, 4) != RCL_RET_OK) {
    return true;}
  else {
    return false;
  }
}

uint16_t read_pwm_channel(int pin) {
  return pulseIn(pin, HIGH, 25000);  // timeout 25ms
}

// ======== RC Functions ========
void update_rc_channels() {
  static unsigned long last_read = 0;
  unsigned long now = millis();
  
  // Read RC channels every 20ms (50Hz typical RC rate)
  if (now - last_read >= 20) {
    last_read = now;
    
    // Read all 6 channels
    rc_channels[0] = read_pwm_channel(RC_CH1_PIN);
    rc_channels[1] = read_pwm_channel(RC_CH2_PIN);
    rc_channels[2] = read_pwm_channel(RC_CH3_PIN);
    rc_channels[3] = read_pwm_channel(RC_CH4_PIN);
    rc_channels[4] = read_pwm_channel(RC_CH5_PIN);
    rc_channels[5] = read_pwm_channel(RC_CH6_PIN);
    
    // Channel 6 decides control mode (1000-1500 = RC, 1500-2000 = AUTO)
    use_rc_control = (rc_channels[5] < 1500);
    
    last_rc_update = now;
    
    // Debug output every 500ms
    static unsigned long last_debug = 0;
    if (now - last_debug >= 500) {
      last_debug = now;
      DEBUG_SERIAL.print("Status: RC: ");
      DEBUG_SERIAL.print(rc_signal_lost() ? "❌" : "✅");
      DEBUG_SERIAL.print(" ROS: ");
      DEBUG_SERIAL.print(ros_signal_lost() ? "❌" : "✅");

      DEBUG_SERIAL.print(" | Mode: ");
      DEBUG_SERIAL.print(use_rc_control ? "RC" : "AUTO");
      DEBUG_SERIAL.print(" | CH1:");
      DEBUG_SERIAL.print(rc_channels[0]);
      DEBUG_SERIAL.print(" CH2:");
      DEBUG_SERIAL.print(rc_channels[1]);
      DEBUG_SERIAL.print(" CH3:");
      DEBUG_SERIAL.print(rc_channels[2]);
      DEBUG_SERIAL.print(" CH4:");
      DEBUG_SERIAL.print(rc_channels[3]);
      DEBUG_SERIAL.print(" CH5:");
      DEBUG_SERIAL.print(rc_channels[4]);
      DEBUG_SERIAL.print(" CH6:");
      DEBUG_SERIAL.println(rc_channels[5]);
    }
  }
}

// Convert PWM value (1000-2000) to servo angle (0-180)
uint8_t pwm_to_servo_angle(uint16_t pwm) {
  // Clamp to 1000-2000 range
  pwm = constrain(pwm, 1000, 2000);
  // Map to 0-180 degrees
  return map(pwm, 1000, 2000, 0, 180);
}

// Convert PWM value to motor speed (-255 to 255)
int16_t pwm_to_motor_speed(uint16_t pwm) {
  // Clamp to 1000-2000 range
  pwm = constrain(pwm, 1000, 2000);
  // 1500 is neutral (0 speed), 1000-1500 is reverse, 1500-2000 is forward
  if (pwm < 1500) {
    return map(pwm, 1000, 1500, -255, 0);
  } else {
    return map(pwm, 1500, 2000, 0, 255);
  }
}

// ======== ROS Callbacks ========

void led_callback(const void * msgin)
{
  DEBUG_SERIAL.println("LED callback triggered");
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  digitalWrite(LED_PIN, msg->data ? HIGH : LOW);
  DEBUG_SERIAL.print("LED set to: "); DEBUG_SERIAL.println(msg->data ? "ON" : "OFF");
}

void servo_callback(const void * msgin)
{
  DEBUG_SERIAL.println("Servo callback triggered");

  // Only process ROS servo commands if RC mode is OFF
  if (use_rc_control) {
    DEBUG_SERIAL.println("Servo: RC control active, ignoring ROS command");
    return;
  }
  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int angle = constrain(msg->data, 0, 180);
  servo.write(angle);
  DEBUG_SERIAL.print("Servo angle (ROS): "); DEBUG_SERIAL.println(angle);
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
    
    last_publish = now;
  }
}

// ======== Setup ========
void setup()
{
  DEBUG_SERIAL.begin(DEBUG_BAUD);
  DEBUG_SERIAL.println("Booting micro-ROS node...");

  // --- Hardware Setup ---
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  servo.attach(SERVO_PIN);

  // --- RC Receiver Setup ---
  pinMode(RC_CH1_PIN, INPUT);
  pinMode(RC_CH2_PIN, INPUT);
  pinMode(RC_CH3_PIN, INPUT);
  pinMode(RC_CH4_PIN, INPUT);
  pinMode(RC_CH5_PIN, INPUT);
  pinMode(RC_CH6_PIN, INPUT);
  
  // --- micro-ROS Transport mode ---
  
  #if defined(USE_SERIAL_TRANSPORT)
    DEBUG_SERIAL.println("Using Serial transport");
    DEBUG_SERIAL.println("Setting up micro-ROS Serial transport...");
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
  #elif defined(USE_WIFI_TRANSPORT)
    DEBUG_SERIAL.println("Using WiFi transport");
    DEBUG_SERIAL.println("Setting up micro-ROS WiFi transport...");
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
  #elif defined(USE_ETHERNET_TRANSPORT)
    DEBUG_SERIAL.println("Using Ethernet transport");
    DEBUG_SERIAL.println("Setting up micro-ROS Ethernet transport...");
    set_microros_ethernet_transports(ETHERNET_MAC, AGENT_IP, AGENT_PORT);
  #endif
  
  delay(2000); // allow sockets to settle

  // --- micro-ROS Init ---
  allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) { DEBUG_SERIAL.println("Support init failed"); return; }

  ret = rclc_node_init_default(&node, "servo_led_node", "", &support);
  if (ret != RCL_RET_OK) { DEBUG_SERIAL.println("Node creation failed"); return; }

  ret = rclc_subscription_init_default(
      &led_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "led_control");
  if (ret != RCL_RET_OK) { DEBUG_SERIAL.println("LED subscriber failed"); return; }

  ret = rclc_subscription_init_default(
      &servo_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "servo_angle");
  if (ret != RCL_RET_OK) { DEBUG_SERIAL.println("Servo subscriber failed"); return; }

  ret = rclc_executor_init(&executor, &support.context, 2, &allocator);
  if (ret != RCL_RET_OK) { DEBUG_SERIAL.println("Executor init failed"); return; }

  rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servo_callback, ON_NEW_DATA);
  DEBUG_SERIAL.println("micro-ROS subscribers ready!");

  // Gyro publisher setup
  ret = rclc_publisher_init_default(
    &gyro_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "gyro_data"
  );
  if (ret != RCL_RET_OK) { 
    DEBUG_SERIAL.println("Gyro publisher failed"); 
    return; 
  }
  analogReadResolution(10);
  DEBUG_SERIAL.println("Gyro publisher ready!");
  DEBUG_SERIAL.println("micro-ROS node setup complete.");
}

// ======== Main Loop ========
void loop()
{
  // Update RC channels
  update_rc_channels();
  

  // Handle RC control if enabled and signal present
  if (use_rc_control && !rc_signal_lost()) {
    // RC Channel 2 controls servo
    uint8_t servo_angle = pwm_to_servo_angle(rc_channels[0]);
    servo.write(servo_angle);
    DEBUG_SERIAL.print("(RC) Servo: "); DEBUG_SERIAL.print(servo_angle);
    
    // RC Channel 3 controls motor (if you have motor code)
    int16_t motor_speed = pwm_to_motor_speed(rc_channels[1]);
    DEBUG_SERIAL.print(" | Motor: "); DEBUG_SERIAL.println(motor_speed);
    // analogWrite(MOTOR_PIN, abs(motor_speed));  // uncomment when ready
        
  } else if (!use_rc_control) {
    // ROS control mode - process ROS commands
    read_and_publish_gyro();
  } else if (rc_signal_lost() && !ros_connected) {
    DEBUG_SERIAL.println("WARNING: Both RC and ROS signals lost - Safe mode!");
    servo.write(90);  // Center servo
    // analogWrite(MOTOR_PIN, 0);  // Stop motor - uncomment when ready
  }

  rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    if (ret != RCL_RET_OK) {
      DEBUG_SERIAL.print("Executor error: ");
      DEBUG_SERIAL.println(ret);
    }
  
  delay(10);
}
