import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

msg = """
Dieu khien robot:
---------------------------
        w
   a    s    d

q : quay trai
e : quay phai

x : dung

CTRL-C de thoat
"""

move_bindings = {
    'w': (1.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0),
    'd': (0.0, -1.0, 0.0),
    'q': (0.0, 0.0, 1.0),
    'e': (0.0, 0.0, -1.0),
    'x': (0.0, 0.0, 0.0),
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopOmni(Node):
    def __init__(self):
        super().__init__('teleop_omni')
        self.publisher_ = self.create_publisher(
            TwistStamped,
            '/mobile_base_controller/reference',
            10
        )
        self.linear_speed = 0.5
        self.angular_speed = 0.5

    def publish_cmd(self, x, y, yaw):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.twist.linear.x = x * self.linear_speed
        msg.twist.linear.y = y * self.linear_speed
        msg.twist.angular.z = yaw * self.angular_speed

        self.publisher_.publish(msg)

def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = TeleopOmni()

    print(msg)

    try:
        while True:
            key = get_key(settings)

            if key in move_bindings:
                x, y, yaw = move_bindings[key]
                node.publish_cmd(x, y, yaw)
            else:
                node.publish_cmd(0.0, 0.0, 0.0)
                if key == '\x03':
                    break

    except Exception as e:
        print(e)

    finally:
        node.publish_cmd(0.0, 0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()