#!/usr/bin/env python3
"""
Truck Control Launch File

Launches all necessary components for the truck bridge system:
- micro-ROS agent via serial
- ESP32 debug logs terminal
- Terminal displaying all current ROS topics
- Terminal displaying /servo_angle/state topic (actual servo position)
- Terminal displaying /imu/data topic
- Keyboard servo controller
- RViz visualization

Usage:
    ros2 launch truck_control.launch.py

Note: Requires a desktop environment (cannot be run via SSH)
      Make sure to reset the ESP32 after launch if connection fails
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    # Path to RViz launch file
    rviz_launch_file = os.path.join(
        os.path.dirname(__file__),
        'utils',
        'rviz_visualization',
        'truck_viz.launch.py'
    )
    
    # Path to keyboard controller
    keyboard_controller_script = os.path.join(
        os.path.dirname(__file__),
        'utils',
        'keyboard_servo_controller.py'
    )
    
    # 1. Start micro-ROS agent via serial in a terminal (wait for reset)
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--',
            'bash', '-c',
            'rm -f /tmp/truck_bridge_ready; '
            'echo "micro-ROS agent ready to start"; '
            'echo "Reset the ESP32 now, then press Enter to continue"; '
            'read -r -p "Press Enter to start the agent..."; '
            'echo "Starting micro-ROS agent..."; '
            'echo ""; '
            'touch /tmp/truck_bridge_ready; '
            'ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6'
        ],
        output='screen',
        name='micro_ros_agent_terminal'
    )
    
    # 2. Terminal displaying debug logs from ESP32 (via /dev/ttyTHS1)
    debug_logs = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--',
            'bash', '-c',
            'echo "Waiting for micro-ROS agent confirmation..."; '
            'while [ ! -f /tmp/truck_bridge_ready ]; do sleep 1; done; '
            'echo "Displaying ESP32 debug logs from /dev/ttyTHS1"; '
            'echo ""; '
            'sudo screen /dev/ttyTHS1 115200'
        ],
        output='screen',
        name='debug_logs_terminal'
    )
    
    # 3. Terminal displaying all current ROS topics (updates periodically)
    topics_list = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--',
            'bash', '-c',
            'while [ ! -f /tmp/truck_bridge_ready ]; do sleep 1; done; '
            'watch -n 2 "ros2 topic list -v"'
        ],
        output='screen',
        name='topics_list_terminal'
    )
    
    # 4. Terminal displaying /servo_angle/state topic (actual servo position feedback)
    servo_angle_echo = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--',
            'bash', '-c',
            'echo "Waiting for micro-ROS agent confirmation..."; '
            'while [ ! -f /tmp/truck_bridge_ready ]; do sleep 1; done; '
            'echo "Monitoring /servo_angle/state topic (actual servo position)"; '
            'echo ""; '
            'ros2 topic echo /servo_angle/state'
        ],
        output='screen',
        name='servo_angle_terminal'
    )
    
    # 5. Terminal displaying /imu/data topic
    imu_data_echo = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--',
            'bash', '-c',
            'echo "Waiting for micro-ROS agent confirmation..."; '
            'while [ ! -f /tmp/truck_bridge_ready ]; do sleep 1; done; '
            'echo "Monitoring /imu/data topic"; '
            'echo ""; '
            'ros2 topic echo /imu/data'
        ],
        output='screen',
        name='imu_data_terminal'
    )
    
    # 6. Start RViz after micro-ROS confirmation
    rviz_launch = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--',
            'bash', '-c',
            'echo "Waiting for micro-ROS agent confirmation..."; '
            'while [ ! -f /tmp/truck_bridge_ready ]; do sleep 1; done; '
            f'ros2 launch {rviz_launch_file}'
        ],
        output='screen',
        name='rviz_terminal'
    )
    
    # 7. Start keyboard controller in a terminal
    keyboard_controller = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--',
            'bash', '-c',
            f'echo "Waiting for micro-ROS agent confirmation..."; '
            f'while [ ! -f /tmp/truck_bridge_ready ]; do sleep 1; done; '
            f'echo "Starting Keyboard Servo Controller..."; '
            f'python3 {keyboard_controller_script}'
        ],
        output='screen',
        name='keyboard_controller_terminal'
    )
    
    # 8. Blocking process to keep launch alive and cleanable with Ctrl+C
    keep_alive = ExecuteProcess(
        cmd=['bash', '-c', 'echo "Truck Control System Running - Press Ctrl+C to stop"; sleep infinity'],
        output='screen',
        name='keep_alive'
    )
    
    return LaunchDescription([
        micro_ros_agent,
        debug_logs,
        topics_list,
        servo_angle_echo,
        imu_data_echo,
        rviz_launch,
        keyboard_controller,
        keep_alive,
    ])
