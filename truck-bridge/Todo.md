# List of improvements
Not all features are fully implemented yet. 
It is highly recommended to address these issues before starting further development regarding the AD-control system.

## Open improvements: 

### TODO: Implement Servo feedback topic: `/servo_angle/state`
Currently, the servo can be controlled via the `/servo_angle/cmd` topic. Due to performance issues, the feedback topic `/servo_angle/state` is not yet in use, but can be found here:

ENTRYPOINT: `output_control.cpp`

- [x] Implement the `/servo_angle/state` feedback topic
- [x] Update the RViz visualization to display the current servo angle instead of `/servo_angle/cmd`
- [x] Update the keyboard controller to display the current servo angle instead of `/servo_angle/cmd`
- [x] Update the documentation if necessary

### TODO: Detect connection loss for micro-ROS and RC
In the current implementation, there is no reliable mechanism to detect if the connection to micro-ROS or the RC controller is lost. Attempts to check for timeouts could not be finished in time.

ENTRYPOINT: `ros_comm_task.cpp` and `rc_task.cpp`

- [ ] Implement a reliable mechanism that detects disconnects from micro-ROS -> Update status LED
- [ ] Implement a reliable mechanism that detects disconnects from the RC controller -> Update status LED

### TODO: Optimize micro-ROS reconnection
When the connection to micro-ROS is lost, the system automatically tries to reconnect. 
However, the old connection is not properly cleared, which can lead to issues when trying to reconnect.

WORKAROUND: Pressing the reset button on the ESP32 can help to clear the previous connection and allow for a fresh reconnection attempt. 

ENTRYPOINT: `ros_comm_task.cpp`
- [ ] Implement a automatic reconnection withouth manually resetting the ESP32.


### TODO: Improve Logging-Sync
Because of the multithreading aproach, the logging of each individual task is no longer in sync.
An Logger-task with a queue is already in place, but the synchronization of the logs is not yet perfect.
- [ ] Implement a more robust logging system that ensures logs from different threads are synchronized and ordered

ENTRYPOINT: `Logger.cpp`

### TODO: Optimize task orchestration
The current implementation leads to a lot of overhead due to task switching and synchronization.
One task per core might reduce the need for synchronization and task switching.
- [ ] Optimize the task orchestration to reduce overhead and improve performance. 

ENTRYPOINT: `main.cpp`


### TODO: Unit-Testing
In the moment there are no automated tests implemented.
For a more maintainable codebase it is recommended to test some specific usecases.
- [ ] Implement unit tests for important functions (hardware constraints may limit testing)

### TODO: Increase servo jitter in RC mode
In RC mode, the servo jitters notably.
- [ ] Increase servo jitter


---
## Well Known Issues:

### RC-Connection loss when input changes too fast
A common issue with the ExpressLRS RC system is that if the input changes too fast (e.g. quickly moving the stick), the connection can be lost and the receiver starts blinking. 

**Fix:**
In most cases the reconnection is automatically establisehed after a moment.
If not, try to disconnect the main power source and the microcontroller and reconnecting it after a few seconds.


### Fix CH340 driver issues
*Note: Issue should be resolved due to a new ESP32 with a different chipset.*
Problem: Upload error on the MCU: "ERROR 2: Cannont access  '/dev/tty/USB0': No such file or directory"

Cause: The CH340 USB driver is not installed correctly on the Jetson. See [link](https://learn.sparkfun.com/tutorials/how-to-install-ch340-drivers/all)

**Fix:**

Preparation: Download driver: https://cdn.sparkfun.com/assets/learn_tutorials/8/4/4/CH341SER_LINUX.ZIP

```
cd into the directory where the files are saved
make clean
make
sudo make load
sudo rmmod ch341
lsmod | grep ch34
```
Reconnect the device
```
dmesg
```
Output should show "ch34x"

---

## Nice to have:
- 

