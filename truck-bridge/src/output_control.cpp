#include <Arduino.h>
#include <Servo.h>
#include "shared_state.h"
#include "config.h"
#include "logger.h"
#include "output_control.h"
#include "ros_interface.h"



extern Servo servo;

// LED state tracking
static bool led_mode_state = false;
static bool led_rc_state = false;
static bool led_ros_state = false;

// Conversion helpers
static uint8_t pwm_to_servo_angle(uint16_t pwm) {
  pwm = constrain(pwm, 1000, 2000);
  return map(pwm, 1000, 2000, 0, 180);
}

static int16_t pwm_to_motor_speed(uint16_t pwm) {
  pwm = constrain(pwm, 1000, 2000);
  if (pwm < 1500) {
    return map(pwm, 1000, 1500, -255, 0);
  } else {
    return map(pwm, 1500, 2000, 0, 255);
  }
}

void output_control_init_leds(void) {
  
  digitalWrite(LED_MODE_PIN, LOW);
  digitalWrite(LED_RC_PIN, LOW);
  digitalWrite(LED_ROS_PIN, LOW);

  led_mode_state = false;
  led_rc_state = false;
  led_ros_state = false;
}

void output_control_update_leds(bool use_rc, bool rc_connected, bool ros_connected) {

  if (!use_rc) {
    if (!led_mode_state) {
      led_mode_state = true;
      digitalWrite(LED_MODE_PIN, HIGH);
      LOG_DEBUG("OUT-CTRL", "Mode LED: ON (AUTO)");
    }
  } else {
    if (led_mode_state) {
      led_mode_state = false;
      digitalWrite(LED_MODE_PIN, LOW);
      LOG_DEBUG("OUT-CTRL", "Mode LED: OFF (RC)");
    }
  }
  
  // Update RC Status LED: ON if RC connected and signal is valid
  if (led_rc_state != rc_connected) {
    led_rc_state = rc_connected;
    digitalWrite(LED_RC_PIN, rc_connected ? HIGH : LOW);
    LOG_DEBUG("OUT-CTRL", "RC Status LED: %s", rc_connected ? "ON (Connected)" : "OFF (Lost)");
  }
  
  // Update ROS Status LED: ON if ROS connected
  if (led_ros_state != ros_connected) {
    led_ros_state = ros_connected;
    digitalWrite(LED_ROS_PIN, ros_connected ? HIGH : LOW);
    LOG_DEBUG("OUT-CTRL", "ROS Status LED: %s", ros_connected ? "ON (Connected)" : "OFF (Disconnected)");
  }
}

void output_control_task(void *pvParameters) {
  LOG_INFO("OUT-TASK", "Starting OUTPUT CONTROL TASK...");
  
  // Initialize LEDs on task start
  output_control_init_leds();
  
  while (1) {
    unsigned long now = millis();
    // Read shared state
    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(400)) == pdTRUE) {
      bool use_rc = control_state.use_rc_control;
      uint16_t *channels = control_state.rc_channels;
      int16_t servo_angle = control_state.desired_servo_angle;
      int16_t motor_speed = control_state.desired_motor_speed;
      bool led_state = control_state.led_state;
      
      // TODO: optimize signal loss check of rc and ros signal. 
      // BUG from AI: shared_rc_signal_lost() want to take the mutex again -> blocking mutex
      //   -> poor performance (waits til timout expires)
      bool signal_lost = 0; // shared_rc_signal_lost(); 
      xSemaphoreGive(state_mutex);

      // Determine connection states for LED indication
      bool rc_connected = !signal_lost;
      bool ros_connected = ros_is_connected();
      
      // Update status LEDs based on current state
      output_control_update_leds(use_rc, rc_connected, ros_connected);

      if (use_rc && !signal_lost) {
        // RC mode - RC commands take priority when active
        // Set values from RC controller 
        servo_angle = pwm_to_servo_angle(channels[0]);
        motor_speed = pwm_to_motor_speed(channels[1]);
        
        LOG_DEBUG("OUT-TASK", "RC Mode - Servo: %d | Motor: %d", servo_angle, motor_speed);
      } 
      // Handle loss of both signals
      else if (signal_lost && !ros_is_connected()) {
        servo_angle = 90;  // Center servo
        motor_speed = 0;   // Stop motor
        LOG_WARN("OUT-TASK", "Both RC and ROS lost - Safe mode!");
      }
      else {
        // Using the latest values from control_state (ROS commands)
        LOG_DEBUG("OUT-TASK", "AUTO Mode - Servo: %d | Motor: %d", servo_angle, motor_speed);
      }

      // Apply commands to outputs (servo, motor, leds)
      servo.write(servo_angle);
      digitalWrite(LED_BUILDIN, led_state ? HIGH : LOW);
      // analogWrite(MOTOR_PIN, abs(motor_speed));  // uncomment when ready
    }
    else{
      LOG_ERROR("OUT-TASK", "ERROR: Failed to acquire state_mutex within 400ms!");
    }

    vTaskDelay(pdMS_TO_TICKS(OUTPUT_UPDATE_INTERVAL));
  }
}

