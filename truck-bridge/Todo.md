# List of improvements
Not all features are fully implemented yet. 
It is highly recommended to address these issues before starting further development regarding the AD-control system.

## Open improvements: 

### TODO: Implement Servo feedback topic: `/servo_angle/state`
Currently, the servo can be controlled via the `/servo_angle/cmd` topic. Due to performance issues, the feedback topic `/servo_angle/state` is not yet implemented.

ENTRYPOINT: `output_control.cpp`

- [ ] Implement the `/servo_angle/state` feedback topic
- [ ] Update the RViz visualization to display the current servo angle based on the feedback topic
- [ ] Update the keyboard controller to display the current servo angle based on the feedback topic
- [ ] Update the documentation if necessary

### TODO: Detect connection loss for micro-ROS and RC
In the current implementation, there is no reliable mechanism to detect if the connection to micro-ROS or the RC controller is lost.

ENTRYPOINT: `ros_comm_task.cpp` and `rc_task.cpp`

- [ ] Implement a reliable mechanism that detects disconnects from micro-ROS.
- [ ] Implement a reliable mechanism that detects disconnects from the RC controller

### TODO: Optimize micro-ROS reconnection
When the connection to micro-ROS is lost, the system automatically tries to reconnect. 
However, the old connection is not properly cleared, which can lead to issues when trying to reconnect.

WORKAROUND: Pressing the reset button on the ESP32 can help to clear the previous connection and allow for a fresh reconnection attempt. 

ENTRYPOINT: `ros_comm_task.cpp`
- [ ] Implement a automatic reconnection withouth resetting the ESP32.


### TODO: Improve the Logging-Sync
Because of the multithreading aproach, the logging of each individual task is no longer in sync.
An implemented Logger with a queue is already in place, but the synchronization of the logs is not yet perfect.
- [ ] Implement a more robust logging system that ensures logs from different threads are synchronized and ordered

ENTRYPOINT: `Logger.cpp`

### TODO: Optimize task orchestration
The current implementation leads to a lot of overhead due to task switching and synchronization.
Consider one task per core to reduce the need for synchronization and task switching.
- [ ] Optimize the task orchestration to reduce overhead and improve performance. 

ENTRYPOINT: `main.cpp`
---

## Well Known Issues:

### FIX: RC-Connection loss when input changes too fast
A common issue with the ExpressLRS RC system is that if the input changes too fast (e.g. quickly moving the stick), the connection can be lost and the receiver starts blinking.
This can be mitigated by adjusting the settings of the ExpressLRS system, such as increasing the refresh rate or adjusting the stick sensitivity. However, it is important to note that this is a known issue with the ExpressLRS system and may not be completely avoidable in all cases.

### FIX: CH340 driver issues
During the development a different ESP32 board with a CH340 USB-to-serial converter was used, which caused some issues with the drivers.

This issue should be resolved by switching to a different ESP32 board.

---

## Nice to have:
- 

