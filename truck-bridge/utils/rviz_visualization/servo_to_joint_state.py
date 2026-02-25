#!/usr/bin/env python3
"""
Servo Angle to Joint State Converter

Subscribes to /servo_angle/cmd (Int32) and publishes joint states
for the truck's front axle steering joint.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
import math


class ServoToJointState(Node):
    def __init__(self):
        super().__init__('servo_to_joint_state')
        
        # Subscriber for servo angle command
        self.subscription = self.create_subscription(
            Int32,
            '/servo_angle/cmd',
            self.servo_callback,
            10
        )
        
        # Publisher for joint states
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # Servo angle parameters
        self.servo_min = 10    # degrees
        self.servo_max = 170   # degrees
        self.servo_center = 90 # degrees
        
        # Joint angle parameters (in radians)
        self.joint_max = 0.524  # ~30 degrees (matches URDF limit)

        # Joint names from URDF (steering + wheels)
        self.joint_names = [
            'base_to_front_axle',
            'front_axle_to_left_wheel',
            'front_axle_to_right_wheel',
            'base_to_rear_left_wheel',
            'base_to_rear_right_wheel'
        ]

        # Maintain last steering angle so TF is always available
        self.last_joint_angle = 0.0

        # Publish joint states periodically to keep TF updated
        self.publish_rate_hz = 10.0
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_joint_state)
        
        self.get_logger().info('Servo to Joint State converter started')
    
    def servo_callback(self, msg):
        """Convert servo angle to joint state and publish"""
        servo_angle = msg.data
        
        # Convert servo angle (10-170°) to joint angle (-0.524 to +0.524 rad)
        # Center at 90°, map to 0 rad
        servo_normalized = (servo_angle - self.servo_center) / (self.servo_max - self.servo_center)
        self.last_joint_angle = servo_normalized * self.joint_max

        # Publish immediately on new command
        self.publish_joint_state()

    def publish_joint_state(self):
        """Publish current joint states for steering and wheels"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names

        # Steering angle for front axle, wheel joints default to 0.0
        joint_state.position = [
            self.last_joint_angle,  # base_to_front_axle
            0.0,  # front_axle_to_left_wheel
            0.0,  # front_axle_to_right_wheel
            0.0,  # base_to_rear_left_wheel
            0.0   # base_to_rear_right_wheel
        ]

        self.publisher.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = ServoToJointState()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
