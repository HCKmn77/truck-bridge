#include <Arduino.h>
#include <micro_ros_platformio.h>
#include "ros_interface.h"
#include "logger.h"
#include "config.h"

void ros_comm_task(void *pvParameters) {
  LOG_INFO("ROS-COMM-TASK", "Starting ROS-COM-TASK ...");
  unsigned long last_reconnect = 0;
  
  while (1) {

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

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
