#!/usr/bin/env python3
"""
Keyboard Servo Controller for micro-ROS truck

Controls servo via ROS 2 /servo_angle/cmd topic using arrow keys:
- Left: Continuously decrease angle while held
- Right: Continuously increase angle while held
- Up/Down: Reserved for future use
- ESC or Ctrl+C: Exit


"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from pynput import keyboard
import sys
import threading


class KeyboardServoController(Node):
    def __init__(self):
        super().__init__('keyboard_servo_controller')
        
        self.publisher = self.create_publisher(Int32, 'servo_angle/cmd', 10)
        
        # Servo constraints
        self.min_angle = 10  # Safe minimum to avoid 0° issues
        self.max_angle = 170  # Safe maximum
        self.current_angle = 90  # Start centered
        self.step_size = 5  # Degrees per update
        self.update_interval = 0.1  # Seconds between updates
        
        # Key state tracking
        self.keys_pressed = set()
        self.running = True
        
        self.get_logger().info('Keyboard Servo Controller started')
        self.get_logger().info('Controls: Hold LEFT/RIGHT to move servo, ESC=exit')
        self.get_logger().info(f'Current angle: {self.current_angle}°')
        
        # Publish initial position
        self.publish_angle(self.current_angle)
        
        # Start update thread
        self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.update_thread.start()

    def publish_angle(self, angle):
        """Constrain and publish servo angle"""
        angle = max(self.min_angle, min(self.max_angle, angle))
        if angle != self.current_angle:
            self.current_angle = angle
            
            msg = Int32()
            msg.data = angle
            self.publisher.publish(msg)
            self.get_logger().info(f'Published angle: {angle}°')

    def _update_loop(self):
        """Continuously update angle based on held keys"""
        import time
        while self.running:
            if keyboard.Key.left in self.keys_pressed:
                new_angle = self.current_angle - self.step_size
                self.publish_angle(new_angle)
                
            elif keyboard.Key.right in self.keys_pressed:
                new_angle = self.current_angle + self.step_size
                self.publish_angle(new_angle)
            
            time.sleep(self.update_interval)

    def on_key_press(self, key):
        """Handle key press - track held keys"""
        try:
            if key in [keyboard.Key.left, keyboard.Key.right]:
                self.keys_pressed.add(key)
                
            elif key == keyboard.Key.esc:
                self.get_logger().info('ESC pressed, exiting...')
                self.running = False
                return False  # Stop listener
                
        except AttributeError:
            pass
        
        return True

    def on_key_release(self, key):
        """Handle key release - stop movement"""
        try:
            if key in self.keys_pressed:
                self.keys_pressed.discard(key)
        except AttributeError:
            pass
        
        return True


def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardServoController()
    
    # Start keyboard listener with press and release handlers
    listener = keyboard.Listener(
        on_press=controller.on_key_press,
        on_release=controller.on_key_release
    )
    listener.start()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        controller.running = False
        listener.stop()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
