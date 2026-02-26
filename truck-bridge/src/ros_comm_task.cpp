#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Servo.h>
#include "ros_interface.h"
#include "logger.h"
#include "config.h"

extern Servo servo;

void ros_comm_task(void *pvParameters) {
  LOG_INFO("ROS-COMM-TASK", "Starting ROS-COM-TASK ...");
  unsigned long last_reconnect = 0;
  unsigned long last_servo_feedback_published = 0;

  while (1) {

    unsigned long now = millis();

    // TODO: Implementing a robust reconnection strategy

    // Checking connection to mirco-ROS agent
    // if (rmw_uros_ping_agent(100, 1)== RMW_RET_OK) {
      
      set_ros_connected(true);
      
      // Spin executor to process ROS messages
      ros_spin_some(100);

    // } else {
      // connection lost - attempting to reconnect
      // set_ros_connected(false);
      // LOG_WARN("ROS-COMM-TASK", "Failed to ping ROS agent! Trying to reconnect...");
      
      // TODO: rmw_uros_disconnect function ist not avaiable for micro-ROS
      // WORKAROUND: Manually resetting the Microcontroller to clear the connection state.

      // disconnect old session 
      // rmw_uros_disconnect(); 

      // // Wait until agent is reachable again 
      // while (rmw_uros_ping_agent(200, 1) != RMW_RET_OK) 
      // {
      //    vTaskDelay(pdMS_TO_TICKS(200)); 
      //   } 
      // LOG_INFO("ROS-COMM-TASK", "Agent reachable. Reinitializing micro-ROS..."); 
      // // Reinitialize 
      // ros_setup_transport();
      // ros_setup_init();
    // }

       
      // TODO: Optimize servo feedback publishing performance. Feedback is currently slowing down the system significantly
      // Publish servo angle feedback to ROS topic /servo_angle/state
 
      if (now - last_servo_feedback_published >= SERVO_FEEDBACK_PUBLISH_INTERVAL) {
        int16_t current_servo_angle = servo.read();
        ros_publish_servo_feedback(current_servo_angle);
        last_servo_feedback_published = now;
        LOG_DEBUG("ROS-COMM-TASK", "Servo feedback published: %d°", current_servo_angle);
      }
    

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
