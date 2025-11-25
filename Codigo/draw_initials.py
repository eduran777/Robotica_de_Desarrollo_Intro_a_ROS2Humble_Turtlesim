#!/usr/bin/env python3
import sys
import termios
import tty
import select
import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen


def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class TurtleWriter(Node):
    def __init__(self):
        super().__init__('turtle_writer')

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen = self.create_client(SetPen, '/turtle1/set_pen')

        self.teleport.wait_for_service()
        self.pen.wait_for_service()

        self.x_top = 2.0
        self.x_bottom = 2.0

        self.y_top = 8.0
        self.y_bottom = 4.5

        self.spacing = 2.0
        self.size = 1.5

        self.set_pen(True)

        self.get_logger().info("Press E D J / A M O R to write. q to quit.")

    def set_pen(self, on):
        req = SetPen.Request()
        req.off = 0 if on else 1
        req.r = 255
        req.g = 255
        req.b = 255
        req.width = 2
        self.pen.call_async(req)

    def move(self, lin=0.0, ang=0.0, t=0.5):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)

        start = time.time()
        while time.time() - start < t:
            self.pub.publish(msg)

        # stop at the end
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)


    def go_to(self, x, y, theta=0.0):
        self.set_pen(False)
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        self.teleport.call_async(req)
        time.sleep(0.2)
        self.set_pen(True)


    # ===== LETTERS =====

    def draw_E(self):
        self.move(self.size, 0, 1)
        self.move(0,2.5,(math.pi / 2) /2.5)
        self.move(self.size, 0, 0.5)
        self.move(0,2.5,(math.pi / 2) /2.5)
        self.move(self.size, 0, 1)
        self.move(-self.size, 0, 1)
        self.move(0,2.5,(3*math.pi / 2) /2.5)
        self.move(self.size, 0, 0.5)
        self.move(0,2.5,(math.pi / 2) /2.5)
        self.move(self.size, 0, 1)


    def draw_D(self):
        self.go_to(self.x_top-10, self.y_top,2*math.pi)
        self.move(self.size, 0, 1)
        self.move(0, 2.5, 0.5)
        self.move(self.size, 0, 0.7)
        self.move(0, 2.5, 0.5)
        self.move(-self.size, 0, 1)

    def draw_J(self):
        self.move(self.size, 0, 1)
        self.move(0, 2.5, 0.6)
        self.move(self.size * 0.5, 0, 0.5)

    def draw_A(self):
        self.move(self.size, 0, 1)
        self.move(0, 2.5, 0.6)
        self.move(self.size, 0, 1)
        self.move(0, -2.5, 0.4)
        self.move(-self.size * 0.6, 0, 0.5)

    def draw_M(self):
        self.move(self.size, 0, 1)
        self.move(0, 2.5, 0.6)
        self.move(self.size, 0, 0.8)
        self.move(0, -2.5, 1)

    def draw_O(self):
        for _ in range(4):
            self.move(self.size, 0, 0.6)
            self.move(0, 2.5, 0.5)

    def draw_R(self):
        self.move(self.size, 0, 1)
        self.move(0, 2.5, 0.6)
        self.move(self.size * 0.8, 0, 0.5)
        self.move(0, 2.5, 0.5)
        self.move(-self.size * 0.6, 0, 0.6)

    def run(self):
        # Force start position
        self.set_pen(False)
        req = TeleportAbsolute.Request()
        req.x = 1.0
        req.y = 10.0
        req.theta = math.pi / 2
        self.teleport.call_async(req)
        time.sleep(0.3)
        self.set_pen(True)

        while rclpy.ok():
            key = get_key()

            if key == 'q':
                break

            if key.lower() in ['e', 'd', 'j']:
                self.go_to(self.x_top, self.y_top,math.pi)

                if key.lower() == 'e': self.draw_E()
                if key.lower() == 'd': self.draw_D()
                if key.lower() == 'j': self.draw_J()

                self.x_top += self.spacing

            if key.lower() in ['a', 'm', 'o', 'r']:
                self.go_to(self.x_bottom, self.y_bottom,math.pi)

                if key.lower() == 'a': self.draw_A()
                if key.lower() == 'm': self.draw_M()
                if key.lower() == 'o': self.draw_O()
                if key.lower() == 'r': self.draw_R()

                self.x_bottom += self.spacing


def main():
    rclpy.init()
    node = TurtleWriter()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
