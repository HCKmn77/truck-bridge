#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Setup transport (Serial | WiFi | Ethernet) based on build flags
void ros_setup_transport(void);

// Initialize micro-ROS node, subscriptions, publishers and executor.
void ros_setup_init(void);

// Publish gyro vector to ROS
void ros_publish_gyro(float x, float y, float z);

// Spin the executor once (wrapper)
void ros_spin_some(uint32_t ms);

// Query connection state
bool ros_is_connected(void);

// ROS comm task entry (call via xTaskCreatePinnedToCore)
void ros_comm_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // ROS_INTERFACE_H
