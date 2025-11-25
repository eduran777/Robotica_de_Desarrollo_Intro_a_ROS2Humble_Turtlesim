#!/usr/bin/env python3
import sys
import termios
import tty
import select
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LIN_SPEED = 2.0    # velocidad lineal
ANG_SPEED = 2.0    # velocidad angular


def get_key(timeout=0.1):
    """
    Lee una tecla sin bloquear.
    Devuelve '' si no se presiona nada en 'timeout' segundos.
    Funciona en Linux en una terminal normal.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


class TurtleKeyboard(Node):
    def __init__(self):
        super().__init__('turtle_keyboard_arrows')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info(
            'Listo. Usa flechas para mover, ESPACIO para parar, q para salir.'
        )

    def send_cmd(self, lin_x=0.0, ang_z=0.0):
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleKeyboard()

    try:
        while rclpy.ok():
            key = get_key()

            if key == '':
                continue

            # Flechas llegan como secuencia: ESC [ A/B/C/D
            if key == '\x1b':
                second = get_key()
                third = get_key()

                if second == '[':
                    if third == 'A':        # flecha arriba
                        node.get_logger().info('↑ adelante')
                        node.send_cmd(lin_x=LIN_SPEED, ang_z=0.0)

                    elif third == 'B':      # flecha abajo
                        node.get_logger().info('↓ atrás')
                        node.send_cmd(lin_x=-LIN_SPEED, ang_z=0.0)

                    elif third == 'C':      # flecha derecha
                        node.get_logger().info('→ gira derecha')
                        node.send_cmd(lin_x=0.0, ang_z=-ANG_SPEED)

                    elif third == 'D':      # flecha izquierda
                        node.get_logger().info('← gira izquierda')
                        node.send_cmd(lin_x=0.0, ang_z=ANG_SPEED)

                # ya procesamos la secuencia, pasamos al siguiente ciclo
                time.sleep(0.01)
                continue

            # Otras teclas
            if key == ' ':
                node.get_logger().info('ESPACIO: stop')
                node.send_cmd(0.0, 0.0)

            elif key.lower() == 'q':
                node.get_logger().info('Q: salir')
                node.send_cmd(0.0, 0.0)
                break

            time.sleep(0.01)

    except KeyboardInterrupt:
        node.get_logger().info('Interrumpido por el usuario.')
        node.send_cmd(0.0, 0.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
