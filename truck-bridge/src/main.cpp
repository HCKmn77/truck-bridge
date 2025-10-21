#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/vector3.h>
#include <Arduino_LSM6DSOX.h>

#define LED_PIN LED_BUILTIN

// ROS entities
rcl_node_t node;
rcl_publisher_t gyro_pub;
rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

// Message instance
geometry_msgs__msg__Vector3 gyro_msg;

// Timer callback: read and publish gyro
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer == NULL) return;

  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    gyro_msg.x = gx;
    gyro_msg.y = gy;
    gyro_msg.z = gz;
    rcl_publish(&gyro_pub, &gyro_msg, NULL);
  }

  // LED heartbeat
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize IMU
  if (!IMU.begin()) {
    // Blink rapidly if IMU fails
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
    }
  }

  // Setup micro-ROS transport on USB Serial
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();

  // Init micro-ROS
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "rp2040_gyro_node", "", &support);

  // Publisher
  rclc_publisher_init_default(
    &gyro_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "/imu/gyro");

  // Timer (publish every 100 ms)
  const unsigned int timer_timeout = 400;
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback);

  // Executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
