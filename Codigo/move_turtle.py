#!/usr/bin/env python3
## ========================= CABECERA E IMPORTACIONES =========================
## Shebang para ejecutar el script como programa en Linux usando python3.

import sys          ## Acceso a stdin y funciones de sistema (file descriptor de teclado)
import termios      ## Configuración de terminal POSIX (modo raw, atributos de la TTY)
import tty          ## Utilidades para cambiar la TTY a modo raw/canonical
import select       ## Multiplexado de E/S para lectura no bloqueante
import time         ## Manejo de tiempos (pausas pequeñas entre iteraciones)

import rclpy        ## Cliente de Python para ROS 2
from rclpy.node import Node
from geometry_msgs.msg import Twist   ## Mensaje estándar para velocidades lineales y angulares


## ======================== CONSTANTES DE VELOCIDAD ========================

LIN_SPEED = 2.0    ## Velocidad lineal base (m/s en el modelo de la tortuga)
ANG_SPEED = 2.0    ## Velocidad angular base (rad/s) para giros izquierda/derecha


## ===================== FUNCIÓN PARA LEER TECLAS (NO BLOQUEANTE) =====================

def get_key(timeout=0.1):
    """
    ## Lee una tecla del teclado de forma no bloqueante.
    ## - timeout: tiempo máximo (s) que se espera por la pulsación.
    ## Devuelve:
    ## - Un carácter (str de longitud 1) si se ha pulsado algo.
    ## - '' (cadena vacía) si no se pulsó nada durante el timeout.
    ##
    ## Implementado usando:
    ## - termios/tty para poner el terminal en modo raw.
    ## - select.select para esperar con límite de tiempo.
    """
    fd = sys.stdin.fileno()              ## Descriptor de archivo asociado a stdin (teclado)
    old_settings = termios.tcgetattr(fd) ## Guardamos configuración actual del terminal

    try:
        tty.setraw(fd)                   ## Modo raw: sin buffer de línea, sin necesidad de Enter
        ## select.select:
        ##   rlist: lista de descriptores a monitorear para lectura.
        ##   timeout: tiempo máx. a esperar.
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            ## Hay datos listos en stdin → se lee 1 carácter
            key = sys.stdin.read(1)
        else:
            ## Nadie pulsó nada dentro del timeout
            key = ''
    finally:
        ## Restaurar la configuración original del terminal
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return key


## ============================ DEFINICIÓN DEL NODO ============================

class TurtleKeyboard(Node):
    """
    ## Nodo ROS 2 que:
    ## - Lee las flechas del teclado (y algunas teclas especiales).
    ## - Traduce la entrada del usuario en comandos de velocidad Twist.
    ## - Publica en /turtle1/cmd_vel para mover la tortuga de turtlesim.
    """

    def __init__(self):
        ## Inicialización de la clase base Node con nombre 'turtle_keyboard_arrows'
        super().__init__('turtle_keyboard_arrows')

        ## Creamos un publicador al tópico /turtle1/cmd_vel de tipo Twist
        ## El tamaño de la cola (10) indica cuántos mensajes se pueden bufferizar
        ## si el suscriptor (turtlesim) no lee inmediatamente.
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        ## Mensaje informativo al arrancar el nodo
        self.get_logger().info(
            'Listo. Usa flechas para mover, ESPACIO para parar, q para salir.'
        )

    def send_cmd(self, lin_x=0.0, ang_z=0.0):
        """
        ## Publica un comando de velocidad en /turtle1/cmd_vel.
        ## Parámetros:
        ## - lin_x: velocidad lineal en el eje x (adelante/atrás).
        ## - ang_z: velocidad angular alrededor del eje z (giro).
        ##
        ## El modelo que usa turtlesim es de robot tipo uniciclo:
        ##   ẋ = v cos(θ)
        ##   ẏ = v sin(θ)
        ##   θ̇ = ω
        ## donde v = lin_x y ω = ang_z.
        """
        msg = Twist()
        msg.linear.x = lin_x    ## Se establece la velocidad lineal
        msg.angular.z = ang_z   ## Se establece la velocidad angular
        self.pub.publish(msg)   ## Se envía el mensaje al tópico


## ============================ FUNCIÓN PRINCIPAL ============================

def main(args=None):
    """
    ## Punto de entrada de la aplicación:
    ## - Inicializa el cliente rclpy.
    ## - Crea el nodo TurtleKeyboard.
    ## - Ejecuta un bucle de lectura de teclado y publicación de comandos.
    ## - Maneja cierre limpio ante 'q' o Ctrl+C.
    """
    ## Inicialización del sistema de comunicación de ROS 2
    rclpy.init(args=args)

    ## Instancia del nodo
    node = TurtleKeyboard()

    try:
        ## Bucle principal: se ejecuta mientras ROS no haya sido apagado
        while rclpy.ok():
            ## Lectura no bloqueante de una tecla
            key = get_key()

            ## Si no se ha pulsado ninguna tecla, se continúa al siguiente ciclo
            if key == '':
                continue

            ## ----------------- TRATAMIENTO DE TECLAS ESPECIALES (FLECHAS) -----------------
            ## En muchas terminales, las flechas se envían como una secuencia de 3 caracteres:
            ##   ESC [ A  → flecha arriba
            ##   ESC [ B  → flecha abajo
            ##   ESC [ C  → flecha derecha
            ##   ESC [ D  → flecha izquierda
            ##
            ## Donde:
            ##   ESC  = '\x1b'
            ##   '['  = carácter delimitador
            ##   'A','B','C','D' = código de flecha
            if key == '\x1b':              ## Primer carácter de la secuencia: ESC
                second = get_key()         ## Segundo carácter (se espera '[')
                third = get_key()          ## Tercer carácter (A, B, C o D)

                if second == '[':
                    ## Identificación de la flecha según el tercer caracter
                    if third == 'A':        ## 'A' → flecha arriba
                        node.get_logger().info('↑ adelante')
                        ## Avanzar con velocidad lineal positiva
                        node.send_cmd(lin_x=LIN_SPEED, ang_z=0.0)

                    elif third == 'B':      ## 'B' → flecha abajo
                        node.get_logger().info('↓ atrás')
                        ## Retroceder con velocidad lineal negativa
                        node.send_cmd(lin_x=-LIN_SPEED, ang_z=0.0)

                    elif third == 'C':      ## 'C' → flecha derecha
                        node.get_logger()..info('→ gira derecha')
                        ## Giro horario (angular negativa)
                        node.send_cmd(lin_x=0.0, ang_z=-ANG_SPEED)

                    elif third == 'D':      ## 'D' → flecha izquierda
                        node.get_logger().info('← gira izquierda')
                        ## Giro antihorario (angular positiva)
                        node.send_cmd(lin_x=0.0, ang_z=ANG_SPEED)

                ## Pequeña pausa para no saturar la CPU y para “consumir” la secuencia
                time.sleep(0.01)
                ## Se hace continue para saltar el resto de la lógica y leer la siguiente tecla
                continue

            ## ----------------- OTRAS TECLAS (NO FLECHAS) -----------------

            ## ESPACIO: detener completamente la tortuga
            if key == ' ':
                node.get_logger().info('ESPACIO: stop')
                node.send_cmd(0.0, 0.0)

            ## 'q' o 'Q': ordenar salida del programa
            elif key.lower() == 'q':
                node.get_logger().info('Q: salir')
                node.send_cmd(0.0, 0.0)  ## Enviar stop antes de salir
                break

            ## Pequeña pausa para aliviar carga de CPU
            time.sleep(0.01)

    except KeyboardInterrupt:
        ## Manejo de Ctrl+C: salida limpia del programa
        node.get_logger().info('Interrumpido por el usuario.')
        node.send_cmd(0.0, 0.0)  ## Detener la tortuga antes de cerrar

    ## Destrucción explícita del nodo
    node.destroy_node()
    ## Apagado del sistema de ROS 2
    rclpy.shutdown()


if __name__ == '__main__':
    ## Si el archivo se ejecuta directamente (no importado como módulo),
    ## se llama a main().
    main()
