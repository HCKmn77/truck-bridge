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
#endif

// Timing intervals
#define DEBUG_BAUD 115200
#define GYRO_PUBLISH_INTERVAL 100
#define RC_UPDATE_INTERVAL 20    // 50Hz
#define OUTPUT_UPDATE_INTERVAL 20  // 50Hz
// TODO change to #define 
const unsigned long CONNECTION_TIMEOUT = 500;  // ms
const unsigned long ROS_RECONNECT_INTERVAL = 2000; // ms


Servo servo;

// ======== Shared State & Synchronization ========
struct ControlState {
  uint16_t rc_channels[6];
  bool use_rc_control;
  int16_t desired_servo_angle;
  int16_t desired_motor_speed;
  bool led_state;
  unsigned long last_rc_update;
  unsigned long last_ros_command;
};

ControlState control_state = {
  .rc_channels = {0, 0, 0, 0, 0, 0},
  .use_rc_control = false,
  .desired_servo_angle = 90,
  .desired_motor_speed = 0,
  .led_state = false,
  .last_rc_update = 0,
  .last_ros_command = 0
};

// Mutex for shared state access
SemaphoreHandle_t state_mutex;



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

bool ros_connected = false;

// ======== Signal Detection ========
bool rc_signal_lost() {
  return (millis() - control_state.last_rc_update) > CONNECTION_TIMEOUT;
}

uint16_t read_pwm_channel(int pin) {
  return pulseIn(pin, HIGH, 25000);  // timeout 25ms
}

// ======== Conversion Functions ========
uint8_t pwm_to_servo_angle(uint16_t pwm) {
  pwm = constrain(pwm, 1000, 2000);
  return map(pwm, 1000, 2000, 0, 180);
}

int16_t pwm_to_motor_speed(uint16_t pwm) {
  pwm = constrain(pwm, 1000, 2000);
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
  
  xSemaphoreTake(state_mutex, portMAX_DELAY);
  control_state.led_state = msg->data;
  control_state.last_ros_command = millis();
  xSemaphoreGive(state_mutex);
  
  DEBUG_SERIAL.print("LED set to: "); DEBUG_SERIAL.println(msg->data ? "ON" : "OFF");
}

void servo_callback(const void * msgin) 
{
  DEBUG_SERIAL.println("Servo callback triggered");
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int angle = constrain(msg->data, 0, 180);
  
  xSemaphoreTake(state_mutex, portMAX_DELAY);
  // Only process ROS servo commands if NOT in RC mode
  if (!control_state.use_rc_control) {
    control_state.desired_servo_angle = angle;
    control_state.last_ros_command = millis();
    DEBUG_SERIAL.print("Servo angle (ROS): "); DEBUG_SERIAL.println(angle);
  } else {
    DEBUG_SERIAL.println("Servo: RC control active, ignoring ROS command");
  }
  xSemaphoreGive(state_mutex);
}

// ======== TASK 1: RC Input Handler (Core 1) ========
void rc_input_task(void *pvParameters) {
  DEBUG_SERIAL.println("[RC Task] Starting on Core 1");
  
  unsigned long last_debug = 0;
  
  while (1) {
    unsigned long now = millis();

    // Read RC channels at 50Hz
    uint16_t temp_channels[6];
    temp_channels[0] = read_pwm_channel(RC_CH1_PIN);
    temp_channels[1] = read_pwm_channel(RC_CH2_PIN);
    temp_channels[2] = read_pwm_channel(RC_CH3_PIN);
    temp_channels[3] = read_pwm_channel(RC_CH4_PIN);
    temp_channels[4] = read_pwm_channel(RC_CH5_PIN);
    temp_channels[5] = read_pwm_channel(RC_CH6_PIN);
    
    // Update shared state
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    memcpy(control_state.rc_channels, temp_channels, sizeof(temp_channels));
    control_state.use_rc_control = (temp_channels[5] < 1500);
    control_state.last_rc_update = now;
    xSemaphoreGive(state_mutex);
    
    // Debug output every 500ms
    if (now - last_debug >= 500) {
      last_debug = now;
      xSemaphoreTake(state_mutex, portMAX_DELAY);
      DEBUG_SERIAL.print("[RC] Status: RC:");
      DEBUG_SERIAL.print(rc_signal_lost() ? "❌" : "✅");
      DEBUG_SERIAL.print(" | Mode:");
      DEBUG_SERIAL.print(control_state.use_rc_control ? "RC" : "AUTO");
      DEBUG_SERIAL.print(" | CH1:");
      DEBUG_SERIAL.print(control_state.rc_channels[0]);
      DEBUG_SERIAL.print(" CH2:");
      DEBUG_SERIAL.print(control_state.rc_channels[1]);
      DEBUG_SERIAL.print(" CH6:");
      DEBUG_SERIAL.println(control_state.rc_channels[5]);
      xSemaphoreGive(state_mutex);
    }
    
    vTaskDelay(pdMS_TO_TICKS(RC_UPDATE_INTERVAL));
  }
}

// ======== TASK 2: ROS Communication (Core 0) ========
void ros_task(void *pvParameters) {
  DEBUG_SERIAL.println("[ROS-Task] Starting on Core 0");
  
  unsigned long last_reconnect = 0;
  
  while (1) {
    // Try to reconnect if disconnected
    if (!ros_connected) {
      if (millis() - last_reconnect >= ROS_RECONNECT_INTERVAL) {
        DEBUG_SERIAL.println("[ROS-Task] Attempting to reconnect...");
        rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        if (ret == RCL_RET_OK) {
          ros_connected = true;
          DEBUG_SERIAL.println("[ROS-Task] Connected!");
        }
        last_reconnect = millis();
      }
    } else {
      // Spin executor to process ROS messages
      rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      if (ret != RCL_RET_OK) {
        DEBUG_SERIAL.print("[ROS-Task] Executor error: ");
        DEBUG_SERIAL.println(ret);
        ros_connected = false;
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ======== TASK 3: Sensor Read & Publish (Core 0) ========
void sensor_task(void *pvParameters) {
  DEBUG_SERIAL.println("[Sensor Task] Starting on Core 0");
  
  unsigned long last_publish = 0;
  
  while (1) {
    unsigned long now = millis();
    
    // Publish gyro data at GYRO_PUBLISH_INTERVAL
    if (now - last_publish >= GYRO_PUBLISH_INTERVAL && ros_connected) {
      // Read analog values and convert to appropriate range
      float x = (analogRead(GYRO_X_PIN) - 512) * (500.0f / 1023.0f);
      float y = (analogRead(GYRO_Y_PIN) - 512) * (500.0f / 1023.0f);
      float z = (analogRead(GYRO_Z_PIN) - 512) * (500.0f / 1023.0f);
      
      gyro_msg.x = x;
      gyro_msg.y = y;
      gyro_msg.z = z;
      
      rcl_publish(&gyro_pub, &gyro_msg, NULL);
      last_publish = now;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ======== TASK 4: Output Control (Core 1) ========
void output_control_task(void *pvParameters) {
  DEBUG_SERIAL.println("[Output-Task] Starting on Core 1");
  
  unsigned long last_update = 0;
  
  while (1) {
    unsigned long now = millis();
    
    // Update outputs at OUTPUT_UPDATE_INTERVAL
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool use_rc = control_state.use_rc_control;
    uint16_t *channels = control_state.rc_channels;
    int16_t servo_angle = control_state.desired_servo_angle;
    int16_t motor_speed = control_state.desired_motor_speed;
    bool led_state = control_state.led_state;
    bool signal_lost = rc_signal_lost();
    xSemaphoreGive(state_mutex);
    
    // Handle RC mode - RC commands take priority when active
    if (use_rc && !signal_lost) {
      servo_angle = pwm_to_servo_angle(channels[0]);
      motor_speed = pwm_to_motor_speed(channels[1]);
      DEBUG_SERIAL.print("[Output-Task] RC Mode - Servo: ");
      DEBUG_SERIAL.print(servo_angle);
      DEBUG_SERIAL.print(" | Motor: ");
      DEBUG_SERIAL.println(motor_speed);
    } 
    // Handle loss of both signals
    else if (signal_lost && !ros_connected) {
      servo_angle = 90;  // Center servo
      motor_speed = 0;   // Stop motor
      DEBUG_SERIAL.println("[Output-Task] WARNING: Both RC and ROS lost - Safe mode!");
    }
    // AUTO/ROS mode - use desired values set by ROS callbacks
    else {
      DEBUG_SERIAL.print("[Output-Task] AUTO Mode - Servo: ");
      DEBUG_SERIAL.print(servo_angle);
      DEBUG_SERIAL.print(" | Motor: ");
      DEBUG_SERIAL.println(motor_speed);
    }
    
    // Apply commands to outputs (servo, motor, leds)
    servo.write(servo_angle);
    digitalWrite(LED_PIN, led_state ? HIGH : LOW);
    // analogWrite(MOTOR_PIN, abs(motor_speed));  // uncomment when ready
    
    vTaskDelay(pdMS_TO_TICKS(OUTPUT_UPDATE_INTERVAL));
  }
}

// ======== Setup ========
void setup() {
  DEBUG_SERIAL.begin(DEBUG_BAUD);
  delay(1000);
DEBUG_SERIAL.println("\n[ROS-Setup] === Booting micro-ROS node ===\n");

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
  
  // --- Create mutex for shared state ---
  state_mutex = xSemaphoreCreateMutex();
  if (state_mutex == NULL) {
    DEBUG_SERIAL.println("ERROR: Failed to create mutex!");
    return;
  }
  
  // --- micro-ROS Transport Setup ---
  #if defined(USE_SERIAL_TRANSPORT)
    DEBUG_SERIAL.println("[ROS-Setup] Using Serial transport");
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
  #elif defined(USE_WIFI_TRANSPORT)
    DEBUG_SERIAL.println("[ROS-Setup] Using WiFi transport");
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
  #elif defined(USE_ETHERNET_TRANSPORT)
    DEBUG_SERIAL.println("[ROS-Setup] Using Ethernet transport");
    set_microros_ethernet_transports(ETHERNET_MAC, AGENT_IP, AGENT_PORT);
  #endif
  
  delay(2000);

  // --- micro-ROS Init ---
  allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) { DEBUG_SERIAL.println("[ROS-Setup] Support init failed"); return;}

  ret = rclc_node_init_default(&node, "servo_led_node", "", &support);
  if (ret != RCL_RET_OK) { DEBUG_SERIAL.println("[ROS-Setup] Node creation failed"); return; }

  ret = rclc_subscription_init_default(
      &led_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "led_control");
  if (ret != RCL_RET_OK) { DEBUG_SERIAL.println("[ROS-Setup] LED subscriber failed"); return; }

  ret = rclc_subscription_init_default(
      &servo_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "servo_angle");
  if (ret != RCL_RET_OK) { DEBUG_SERIAL.println("[ROS-Setup] Servo subscriber failed"); return; }

  ret = rclc_executor_init(&executor, &support.context, 2, &allocator);
  if (ret != RCL_RET_OK) { DEBUG_SERIAL.println("[ROS-Setup] Executor init failed"); return; }

  rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servo_callback, ON_NEW_DATA);
  DEBUG_SERIAL.println("[ROS-Setup] micro-ROS subscribers ready!");

  // Gyro publisher setup
  ret = rclc_publisher_init_default(
    &gyro_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "gyro_data"
  );
  if (ret != RCL_RET_OK) { 
    DEBUG_SERIAL.println("[ROS-Setup] Gyro publisher failed"); 
    return; 
  }
  analogReadResolution(10);
  DEBUG_SERIAL.println("[ROS-Setup] Gyro publisher ready!");

  // ======= Create FreeRTOS Tasks =======
  // Task priorities: higher number -> higher priority
  // Core 0 (Protocol CPU): ROS + Sensors
  // Core 1 (App CPU): RC Input + Outputs (lower latency)
  // TODO: Change task distribution: 
  //    - Task0: Communication eg. ROS, RC Input


  xTaskCreatePinnedToCore(
    rc_input_task,      // Task function
    "RC_Input",         // Task name
    3072,               // Stack size (bytes)
    NULL,               // Parameters
    2,                  // Priority (higher = more important)
    NULL,               // Task handle
    1                   // Core (1 = App CPU)
  );

  xTaskCreatePinnedToCore(
    output_control_task,
    "Outputs",
    3072,
    NULL,
    2,
    NULL,
    1                   // Core 1
  );

  xTaskCreatePinnedToCore(
    ros_task,
    "ROS_Comm",
    4096,               // More stack for ROS
    NULL,
    1,                  // Lower priority - can handle delays
    NULL,
    0                   // Core 0 (Protocol CPU)
  );

  xTaskCreatePinnedToCore(
    sensor_task,
    "Sensors",
    3072,
    NULL,
    1,
    NULL,
    0                   // Core 0
  );

  DEBUG_SERIAL.println("[Setup] All tasks created!");
  DEBUG_SERIAL.println("=== Setup Complete - Running Multi-Core ===\n");
}

void loop() {
  // All work is outsourced to FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
  
}
