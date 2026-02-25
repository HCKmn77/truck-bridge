#!/usr/bin/env python3
"""
IMU to TF Converter + Axis Remapper

1. Subscribes to /imu/data (raw) and remaps axes (X<->Y for pitch/roll correction)
   Published to /imu/data_remapped
   
2. Subscribes to /imu/data_filtered (with orientation from Madgwick filter)
   and publishes the truck's orientation as a TF transform from odom to base_link.

This integrates IMU axis remapping without needing a separate node.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class ImuToTF(Node):
    def __init__(self):
        super().__init__('imu_to_tf')
        
        # Publisher for remapped IMU data (for filter to use)
        self.remapped_publisher = self.create_publisher(Imu, '/imu/data_remapped', 10)
        
        # Subscriber for RAW IMU data (to remap and republish)
        self.raw_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.raw_imu_callback,
            10
        )
        
        # Subscriber for FILTERED IMU data (with orientation computed by imu_filter_madgwick)
        self.filtered_subscription = self.create_subscription(
            Imu,
            '/imu/data_filtered',
            self.filtered_imu_callback,
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Position (stays at origin, only orientation changes)
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        
        self.get_logger().info('IMU to TF converter + remapper started')
        self.get_logger().info('  - Raw /imu/data (X<->Y swapped) → /imu/data_remapped')
        self.get_logger().info('  - Filtered /imu/data_filtered → TF (odom → base_link)')
    
    def is_valid_quaternion(self, quat):
        """Check if quaternion contains valid (non-NaN) values"""
        # Check for NaN
        is_nan = (math.isnan(quat.x) or 
                  math.isnan(quat.y) or 
                  math.isnan(quat.z) or 
                  math.isnan(quat.w))
        
        if is_nan:
            return False
        
        # Check if all zeros (completely invalid)
        is_all_zeros = (quat.x == 0.0 and 
                        quat.y == 0.0 and 
                        quat.z == 0.0 and 
                        quat.w == 0.0)
        
        if is_all_zeros:
            self.get_logger().warn('Quaternion is all zeros - IMU orientation not available')
            return False
        
        # Check magnitude (should be close to 1.0 for normalized quaternion)
        magnitude = math.sqrt(quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2)
        if magnitude < 0.1:  # Very small magnitude indicates invalid data
            self.get_logger().warn(f'Quaternion magnitude too small: {magnitude}')
            return False
        
        return True
    
    def raw_imu_callback(self, msg):
        """Remap raw IMU axes and republish"""
        remapped_msg = Imu()
        remapped_msg.header = msg.header
        
        # Remap axes: swap X and Y (pitch and roll were reversed on this IMU)
        remapped_msg.linear_acceleration.x = msg.linear_acceleration.y
        remapped_msg.linear_acceleration.y = msg.linear_acceleration.x
        remapped_msg.linear_acceleration.z = msg.linear_acceleration.z
        
        remapped_msg.angular_velocity.x = msg.angular_velocity.y
        remapped_msg.angular_velocity.y = msg.angular_velocity.x
        remapped_msg.angular_velocity.z = msg.angular_velocity.z
        
        # Remap covariances (3x3 matrix swap)
        orig_lin = msg.linear_acceleration_covariance
        orig_ang = msg.angular_velocity_covariance
        
        remapped_msg.linear_acceleration_covariance = [
            orig_lin[4], orig_lin[3], orig_lin[5],
            orig_lin[1], orig_lin[0], orig_lin[2],
            orig_lin[7], orig_lin[6], orig_lin[8]
        ]
        
        remapped_msg.angular_velocity_covariance = [
            orig_ang[4], orig_ang[3], orig_ang[5],
            orig_ang[1], orig_ang[0], orig_ang[2],
            orig_ang[7], orig_ang[6], orig_ang[8]
        ]
        
        # Keep orientation as-is (will be recomputed by filter)
        remapped_msg.orientation = msg.orientation
        remapped_msg.orientation_covariance = msg.orientation_covariance
        
        # Publish remapped data
        self.remapped_publisher.publish(remapped_msg)
    
    def filtered_imu_callback(self, msg):
        """Convert filtered IMU orientation to TF transform"""
        # Create transform message
        t = TransformStamped()
        
        # Timestamp
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Position (truck stays at origin, only rotates)
        t.transform.translation.x = self.position_x
        t.transform.translation.y = self.position_y
        t.transform.translation.z = self.position_z
        
        # Orientation from IMU (with validation)
        if self.is_valid_quaternion(msg.orientation):
            t.transform.rotation.x = msg.orientation.x
            t.transform.rotation.y = msg.orientation.y
            t.transform.rotation.z = msg.orientation.z
            t.transform.rotation.w = msg.orientation.w
            self.get_logger().debug(f'Using IMU quaternion: [{msg.orientation.x:.3f}, {msg.orientation.y:.3f}, {msg.orientation.z:.3f}, {msg.orientation.w:.3f}]')
        else:
            # Use identity quaternion if IMU data is invalid
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.get_logger().warn(f'Using identity quaternion. Received invalid: [{msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}, {msg.orientation.w}]')
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ImuToTF()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
