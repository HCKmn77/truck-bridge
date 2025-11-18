import sys
import select
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

def read_key(timeout=0.05):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if not dr:
        return ''
    c1 = sys.stdin.read(1)
    if c1 != '\x1b':
        return c1
    # Read the rest of the escape sequence
    seq = sys.stdin.read(2)
    return c1 + seq

class ServoTeleop(Node):
    def __init__(self):
        super().__init__('servo_teleop')
        self.pub = self.create_publisher(Int32, '/servo_angle', 10)
        self.angle = 90
        self.get_logger().info('Use ←/→ or a/d to change angle, q to quit.')

    def publish(self):
        m = Int32()
        m.data = int(self.angle)
        self.pub.publish(m)

def main():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    rclpy.init()
    node = ServoTeleop()
    node.publish()

    try:
        while rclpy.ok():
            key = read_key()
            if not key:
                time.sleep(0.01)
                continue

            # left arrow or 'a'
            if key in ('\x1b[D', 'a'):
                node.angle = max(0, node.angle - 5)
                node.publish()
                node.get_logger().info(f'Angle: {node.angle}\n')
            # right arrow or 'd'
            elif key in ('\x1b[C', 'd'):
                node.angle = min(180, node.angle + 5)
                node.publish()
                node.get_logger().info(f'Angle: {node.angle}\n')
            # quit with q
            elif key == 'q':
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.get_logger().info('Shutting down teleop')
        rclpy.shutdown()

if __name__ == '__main__':
    main()