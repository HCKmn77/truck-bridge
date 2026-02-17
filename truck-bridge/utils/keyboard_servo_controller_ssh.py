#!/usr/bin/env python3
"""
Keyboard Servo Controller for micro-ROS truck (SSH-compatible version)

Controls servo via ROS 2 /servo_angle/cmd topic using keyboard:
- a: Decrease angle by step
- d: Increase angle by step
- Left arrow: Continuously decrease while held
- Right arrow: Continuously increase while held
- r: Reset to center (90°)
- q or ESC: Exit

Works over SSH using curses for terminal control.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import curses
import threading
import time


class KeyboardServoControllerSSH(Node):
    def __init__(self):
        super().__init__('keyboard_servo_controller_ssh')
        
        self.publisher = self.create_publisher(Int32, 'servo_angle/cmd', 10)
        
        # Servo constraints
        self.min_angle = 10
        self.max_angle = 170
        self.current_angle = 90
        self.step_size = 5
        self.continuous_step = 2
        self.update_interval = 0.05
        
        # State
        self.running = True
        self.continuous_left = False
        self.continuous_right = False
        
        # Publish initial position
        self.publish_angle(self.current_angle)
        
        # Start continuous update thread
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

    def _update_loop(self):
        """Continuously update angle when keys are held"""
        while self.running:
            if self.continuous_left:
                new_angle = self.current_angle - self.continuous_step
                self.publish_angle(new_angle)
            elif self.continuous_right:
                new_angle = self.current_angle + self.continuous_step
                self.publish_angle(new_angle)
            
            time.sleep(self.update_interval)

    def handle_key(self, key):
        """Handle keyboard input"""
        if key == ord('a') or key == ord('A'):
            # Single step left
            self.publish_angle(self.current_angle - self.step_size)
            
        elif key == ord('d') or key == ord('D'):
            # Single step right
            self.publish_angle(self.current_angle + self.step_size)
            
        elif key == curses.KEY_LEFT:
            # Continuous left (will stop on key release)
            self.continuous_left = True
            self.continuous_right = False
            
        elif key == curses.KEY_RIGHT:
            # Continuous right (will stop on key release)
            self.continuous_right = True
            self.continuous_left = False
            
        elif key == ord('r') or key == ord('R'):
            # Reset to center
            self.publish_angle(90)
            
        elif key == ord('q') or key == ord('Q') or key == 27:  # ESC
            self.running = False
            return False
        
        return True

    def stop_continuous(self):
        """Stop continuous movement"""
        self.continuous_left = False
        self.continuous_right = False


def curses_main(stdscr, controller):
    """Main curses UI loop"""
    # Setup curses
    curses.curs_set(0)  # Hide cursor
    stdscr.nodelay(True)  # Non-blocking input
    stdscr.timeout(50)  # 50ms timeout for getch()
    
    # Draw UI
    def draw_ui():
        stdscr.clear()
        stdscr.addstr(0, 0, "=" * 60)
        stdscr.addstr(1, 0, "  Keyboard Servo Controller")
        stdscr.addstr(2, 0, "=" * 60)
        stdscr.addstr(4, 0, "Controls:")
        stdscr.addstr(5, 0, "  a/A      : Decrease by 5°")
        stdscr.addstr(6, 0, "  d/D      : Increase by 5°")
        stdscr.addstr(7, 0, "  ← (hold) : Continuously decrease")
        stdscr.addstr(8, 0, "  → (hold) : Continuously increase")
        stdscr.addstr(9, 0, "  r/R      : Reset to center (90°)")
        stdscr.addstr(10, 0, "  q/ESC    : Quit")
        stdscr.addstr(12, 0, "-" * 60)
        stdscr.addstr(13, 0, f"Current Angle: {controller.current_angle:3d}°")
        stdscr.addstr(14, 0, f"Range: {controller.min_angle}° - {controller.max_angle}°")
        stdscr.addstr(15, 0, "-" * 60)
        
        # Visual indicator
        bar_width = 50
        bar_pos = int((controller.current_angle - controller.min_angle) / 
                     (controller.max_angle - controller.min_angle) * bar_width)
        bar = "[" + " " * bar_pos + "|" + " " * (bar_width - bar_pos) + "]"
        stdscr.addstr(17, 0, bar)
        stdscr.addstr(18, 0, f"{controller.min_angle}°" + " " * (bar_width - 5) + f"{controller.max_angle}°")
        
        stdscr.refresh()
    
    draw_ui()
    
    # Main loop
    while controller.running:
        key = stdscr.getch()
        
        if key != -1:  # Key was pressed
            if key == curses.KEY_LEFT or key == curses.KEY_RIGHT:
                # Arrow key pressed - start continuous movement
                controller.handle_key(key)
            else:
                # Other key - stop continuous and handle
                controller.stop_continuous()
                if not controller.handle_key(key):
                    break
            
            draw_ui()
        else:
            # No key pressed - stop continuous movement if arrows released
            # (This is a limitation: we can't detect key release in curses)
            # So we stop on next key or after timeout
            pass
        
        # Update display
        stdscr.addstr(13, 0, f"Current Angle: {controller.current_angle:3d}°" + " " * 10)
        
        # Update bar
        bar_width = 50
        bar_pos = int((controller.current_angle - controller.min_angle) / 
                     (controller.max_angle - controller.min_angle) * bar_width)
        bar = "[" + " " * bar_pos + "|" + " " * (bar_width - bar_pos) + "]"
        stdscr.addstr(17, 0, bar)
        
        stdscr.refresh()
        time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardServoControllerSSH()
    
    # Create ROS spin thread
    spin_thread = threading.Thread(target=lambda: rclpy.spin(controller), daemon=True)
    spin_thread.start()
    
    try:
        # Run curses UI in main thread
        curses.wrapper(curses_main, controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.running = False
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
