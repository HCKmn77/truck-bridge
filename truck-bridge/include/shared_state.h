#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ControlState {
  uint16_t rc_channels[6];
  bool use_rc_control;
  int16_t desired_servo_angle;
  int16_t desired_motor_speed;
  bool led_state;
  unsigned long last_rc_update;
  unsigned long last_ros_command;
} ControlState;

// Shared instance (defined in src/shared_state.cpp)
extern ControlState control_state;

// Mutex handle (defined in src/shared_state.cpp)
extern SemaphoreHandle_t state_mutex;

// Initialize shared state and create mutex. Returns true on success.
bool shared_state_init(void);

// Helper to check if RC signal is lost
bool shared_rc_signal_lost(void);

#ifdef __cplusplus
}
#endif

#endif // SHARED_STATE_H
