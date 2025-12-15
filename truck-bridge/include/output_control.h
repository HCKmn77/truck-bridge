#ifndef OUTPUT_CONTROL_H
#define OUTPUT_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Output control task - manages all output ports e.g. servo, motor, and status LEDs
 * Status LEDs:
 * - MODE LED (LED_MODE_PIN): OFF = RC mode, ON = ROS mode
 * - RC Status LED (LED_RC_PIN): ON = RC connected, OFF = RC lost
 * - ROS Status LED (LED_ROS_PIN): ON = ROS connected, OFF = ROS disconnected
 */
void output_control_task(void *pvParameters);

/**
 * Initialize LED pins and set initial states
 */
void output_control_init_leds(void);

/**
 * Update status LEDs based on control mode and connection states
 */
void output_control_update_leds(bool use_rc, bool rc_connected, bool ros_connected);

#ifdef __cplusplus
}
#endif

#endif // OUTPUT_CONTROL_H
