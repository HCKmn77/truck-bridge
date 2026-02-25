#!/usr/bin/env python3
"""
RViz Visualization Launch File for Truck Bridge

Launches RViz with the truck URDF model, displaying:
- Front axle steering controlled by /servo_angle/cmd topic
- Truck orientation from /imu/data topic (with Madgwick filter for orientation computation)

IMU Data Flow:
  ESP32 → /imu/data (raw: gyro + accel)
       → imu_filter_madgwick (computes quaternion)
       → /imu/data_filtered (with orientation)
       → imu_to_tf.py (creates TF transform)
       → RViz visualization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Get the path to the URDF file
    urdf_file = os.path.join(
        os.path.dirname(__file__),
        'truck.urdf'
    )
    
    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Robot state publisher node (publishes TF transforms from URDF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # IMU Filter Madgwick Node (computes orientation from raw gyro + accel)
    # Takes raw IMU data from ESP32 and computes quaternion orientation
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': False,  # No magnetometer available on BMI270
            'use_magnetic_field_msg': False,
            'world_frame': 'enu',  # East-North-Up frame
            'publish_tf': False,  # We handle TF separately with imu_to_tf.py
            
            # Filter tuning parameters (optimized for stability and drift reduction)
            'gain': 0.01,  # Madgwick filter gain (beta): Lower = smoother, less sensitive
                          # Range: 0.01-0.5, Default: 0.1
                          # Reduced from 0.1 to 0.03 for less sensitivity
            
            'zeta': 0.02,  # Gyroscope bias drift compensation gain
                          # Range: 0.0-0.1, Default: 0.0
                          # Increased from 0.0 to 0.01 to reduce drift
            
            'stateless': False,  # Maintain filter state between updates
            
            # Optional: Reject accelerometer updates during high acceleration
            # This prevents the filter from trusting accel data during movement
            # Uncomment if you want even more stability during truck movement:
            # 'mag_bias_x': 0.0,
            # 'mag_bias_y': 0.0, 
            # 'mag_bias_z': 0.0,
        }],
        remappings=[
            # Input: remapped data from imu_to_tf.py (axis-corrected)
            ('imu/data_raw', '/imu/data_remapped'),
            # Output: filtered data with computed orientation quaternion
            ('imu/data', '/imu/data_filtered')
        ]
    )
    
    # Servo to Joint State converter (converts /servo_angle/cmd to joint_states)
    servo_to_joint_state_script = os.path.join(
        os.path.dirname(__file__),
        'servo_to_joint_state.py'
    )
    
    servo_converter_node = ExecuteProcess(
        cmd=['python3', servo_to_joint_state_script],
        output='screen',
        name='servo_to_joint_state'
    )
    
    # IMU to TF converter (converts /imu/data_filtered to odom->base_link transform)
    # Uses the orientation quaternion computed by imu_filter_madgwick
    imu_to_tf_script = os.path.join(
        os.path.dirname(__file__),
        'imu_to_tf.py'
    )
    
    imu_to_tf_node = ExecuteProcess(
        cmd=['python3', imu_to_tf_script],
        output='screen',
        name='imu_to_tf'
    )
    

    
    # RViz node
    rviz_config_file = os.path.join(
        os.path.dirname(__file__),
        'truck_config.rviz'
    )
    
    # Check if rviz config exists, otherwise RViz will start with default config
    rviz_node_args = {
        'package': 'rviz2',
        'executable': 'rviz2',
        'name': 'rviz2',
        'output': 'screen',
    }
    
    if os.path.exists(rviz_config_file):
        rviz_node_args['arguments'] = ['-d', rviz_config_file]
    
    rviz_node = Node(**rviz_node_args)
    
    return LaunchDescription([
        robot_state_publisher_node,
        imu_to_tf_node,  # Remap raw IMU axes first
        imu_filter_node,  # Then filter the remapped data for orientation computation
        servo_converter_node,
        rviz_node
    ])
