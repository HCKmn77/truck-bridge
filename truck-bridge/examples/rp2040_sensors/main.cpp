#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/vector3.h>

#include <Arduino_LSM6DSOX.h>
#include <Arduino_HTS221.h>

#define LED_PIN LED_BUILTIN

// ROS entities
rcl_node_t node;
rcl_publisher_t temp_pub;
rcl_publisher_t accel_pub;
rcl_publisher_t gyro_pub;
rcl_subscription_t led_sub;
rcl_timer_t timer;
rclc_executor_t executor;

rclc_support_t support;
rcl_allocator_t allocator;

// Message instances
std_msgs__msg__Float32 temp_msg;
geometry_msgs__msg__Vector3 accel_msg;
geometry_msgs__msg__Vector3 gyro_msg;

// LED control callback
void led_callback(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  digitalWrite(LED_PIN, msg->data ? HIGH : LOW);
}

// Timer callback — publishes IMU and temperature data
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer == NULL)
    return;

  // Temperature
  temp_msg.data = HTS.readTemperature();
  rcl_publish(&temp_pub, &temp_msg, NULL);

  // IMU Accel
  float ax, ay, az;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    accel_msg.x = ax;
    accel_msg.y = ay;
    accel_msg.z = az;
    rcl_publish(&accel_pub, &accel_msg, NULL);
  }

  // IMU Gyro
  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    gyro_msg.x = gx;
    gyro_msg.y = gy;
    gyro_msg.z = gz;
    rcl_publish(&gyro_pub, &gyro_msg, NULL);
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Setup micro-ROS transport
  set_microros_serial_transports(Serial);

  // Initialize sensors
  if (!IMU.begin()) {
    //Serial.println("Failed to initialize LSM6DSOX!");
    while (1);
  }
  if (!HTS.begin()) {
    //Serial.println("Failed to initialize HTS221!");
    while (1);
  }

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "rp2040_nano_node", "", &support);

  // Publishers
  rclc_publisher_init_default(
    &temp_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/env/temp");

  rclc_publisher_init_default(
    &accel_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "/imu/accel");

  rclc_publisher_init_default(
    &gyro_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "/imu/gyro");

  // Subscriber
  rclc_subscription_init_default(
    &led_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/led/control");

  // Timer (publish every 200 ms)
  const unsigned int timer_timeout = 200;
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback);

  // Executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &led_sub, &temp_msg, &led_callback, ON_NEW_DATA);

  //Serial.println("micro-ROS node started on RP2040 Nano Connect!");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
