#!/usr/bin/env python3
## ========================= CABECERA E IMPORTACIONES =========================
## Shebang para permitir ejecutar el script como ejecutable en Linux
## usando el intérprete de Python 3 del entorno actual.

import sys          ## Acceso a stdin y funciones de sistema
import termios      ## Configuración de terminal POSIX (modo raw, etc.)
import tty          ## Utilidades para cambiar el modo del terminal
import select       ## Multiplexado de E/S para lectura no bloqueante
import time         ## Manejo de tiempos (sleep, medición de duración)
import math         ## Funciones matemáticas (π, etc.)

import rclpy        ## Cliente de Python para ROS 2
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen


## ===================== FUNCIÓN PARA LEER TECLADO (NO BLOQUEANTE) =====================

def get_key(timeout=0.1):
    """
    ## Lee una tecla del teclado en modo "raw" con un tiempo máximo de espera.
    ## - timeout: tiempo máximo (en segundos) que se espera por una tecla.
    ## Devuelve:
    ## - Un carácter (str de longitud 1) si se presiona una tecla a tiempo.
    ## - Cadena vacía '' si no se presionó ninguna tecla dentro del timeout.
    """
    fd = sys.stdin.fileno()             ## Descriptor de archivo de la entrada estándar (teclado)
    old = termios.tcgetattr(fd)         ## Se guarda la configuración actual del terminal para restaurarla luego
    try:
        tty.setraw(fd)                  ## Se pone el terminal en modo "raw": lectura carácter a carácter, sin Enter
        ## select.select:
        ##   - Espera hasta 'timeout' segundos a que haya datos disponibles en stdin.
        ##   - r: lista de descriptores listos para lectura.
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            ## Hay al menos un carácter disponible, se lee 1 byte/char
            return sys.stdin.read(1)
        ## Si no hay nada, devolvemos cadena vacía
        return ''
    finally:
        ## Muy importante: restaurar la configuración original del terminal
        ## para no dejar la consola en modo raw permanentemente.
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


## ============================ CLASE PRINCIPAL DEL NODO ============================

class TurtleWriter(Node):
    """
    ## Nodo ROS2 que:
    ## - Lee teclas del teclado.
    ## - Interpreta ciertas teclas como letras (E, D, J, A, M, O, R).
    ## - Utiliza turtlesim para "escribir" esas letras con la tortuga.
    """

    def __init__(self):
        ## Inicializa el nodo con el nombre 'turtle_writer'
        super().__init__('turtle_writer')

        ## -------------------- PUBLICADOR DE VELOCIDADES --------------------
        ## Publisher al tópico /turtle1/cmd_vel de tipo Twist.
        ## Este tópico controla el movimiento de la tortuga (modelo uniciclo).
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        ## -------------------- CLIENTES DE SERVICIO DE TURTLESIM --------------------
        ## Cliente para teletransportar la tortuga a una posición/ángulo absolutos.
        self.teleport = self.create_client(
            TeleportAbsolute, '/turtle1/teleport_absolute'
        )

        ## Cliente para configurar el "lápiz" (color, grosor, encendido/apagado).
        self.pen = self.create_client(SetPen, '/turtle1/set_pen')

        ## Esperar a que los servicios estén disponibles antes de usarlos.
        self.teleport.wait_for_service()
        self.pen.wait_for_service()

        ## -------------------- PARÁMETROS DE POSICIÓN INICIAL DE TEXTO --------------------
        ## Coordenadas de inicio de la línea superior (para E, D, J).
        self.x_top = 0.5     ## Coordenada x inicial de la primera letra en la línea superior
        self.y_top = 8.0     ## Coordenada y de la línea superior

        ## Coordenadas de inicio de la línea inferior (para A, M, O, R).
        self.x_bottom = 0.5  ## Coordenada x inicial de la primera letra en la línea inferior
        self.y_bottom = 4.5  ## Coordenada y de la línea inferior

        ## Distancia horizontal entre letras (espaciado).
        self.spacing = 2.0   ## "Ancho" entre letras consecutivas en una misma línea

        ## Factor de escala de la letra.
        ## Se utiliza como base para la duración/longitud de los trazos.
        self.size = 1.5

        ## Se enciende el lápiz al iniciar (color y grosor se definen en set_pen).
        self.set_pen(True)

        ## Mensaje informativo al usuario.
        self.get_logger().info("Press E D J / A M O R to write. q to quit.")

    ## ============================ CONTROL DEL LÁPIZ ============================

    def set_pen(self, on):
        """
        ## Configura el estado del lápiz de la tortuga.
        ## - on = True  → lápiz encendido (la tortuga dibuja).
        ## - on = False → lápiz apagado (la tortuga se mueve sin dejar trazo).
        """
        req = SetPen.Request()
        ## Campo 'off' controla el estado:
        ##   0 → lápiz encendido
        ##   1 → lápiz apagado
        req.off = 0 if on else 1

        ## Color blanco (RGB = 255,255,255).
        req.r = 255
        req.g = 255
        req.b = 255

        ## Grosor de la línea (en píxeles).
        req.width = 2

        ## Llamada asíncrona al servicio /turtle1/set_pen
        self.pen.call_async(req)

    ## ====================== PRIMITIVA DE MOVIMIENTO (v, ω, t) ======================

    def move(self, lin=0.0, ang=0.0, t=0.5):
        """
        ## Aplica un movimiento con velocidad constante durante un tiempo t.
        ## Parámetros:
        ## - lin: velocidad lineal (Twist.linear.x)
        ## - ang: velocidad angular (Twist.angular.z)
        ## - t:   duración del movimiento (segundos)
        ##
        ## Esta función genera un movimiento tipo "uniciclo":
        ##   - Si ang = 0 → movimiento rectilíneo.
        ##   - Si lin = 0 → giro puro sobre el eje.
        ##   - Si ambos ≠ 0 → arco de circunferencia.
        """
        msg = Twist()
        msg.linear.x = float(lin)    ## Se fuerza a float para cumplir con el tipo del msg
        msg.angular.z = float(ang)

        start = time.time()          ## Tiempo inicial del movimiento
        ## Bucle mientras no se alcance la duración t
        while time.time() - start < t:
            self.pub.publish(msg)    ## Se publica continuamente el mismo Twist

        ## Tras el movimiento, se envía un Twist nulo para detener la tortuga.
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    ## ====================== TELETRANSPORTE A UNA POSICIÓN ABSOLUTA ======================

    def go_to(self, x, y, theta=0.0):
        """
        ## Teletransporta la tortuga a la posición (x, y) con orientación theta.
        ## - x, y: coordenadas en el espacio de turtlesim.
        ## - theta: orientación (rad), 0 hacia la derecha, π/2 hacia arriba, π hacia la izquierda, etc.
        ##
        ## Durante el teletransporte se apaga el lápiz para evitar trazos no deseados.
        """
        ## Apagar lápiz antes de mover la tortuga instantáneamente
        self.set_pen(False)

        ## Se construye el request para el servicio TeleportAbsolute.
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta

        ## Llamada asíncrona al servicio /turtle1/teleport_absolute
        self.teleport.call_async(req)

        ## Pequeña pausa para dar tiempo al servicio a procesar el teletransporte.
        time.sleep(0.2)

        ## Encender el lápiz de nuevo
        self.set_pen(True)

    # ============================ FUNCIONES PARA CADA LETRA ============================

    def draw_E(self):
        """
        ## Dibuja la letra 'E' mediante una secuencia de giros y traslaciones.
        ## Se asume que la tortuga ya está correctamente posicionada y orientada
        ## (en el run() se controla la orientación inicial con theta=π para apuntar a la izquierda).
        """
        self.move(0, 2.5, (math.pi) / 2.5)
        self.move(self.size, 0, 0.7)
        self.move(0, 2.5, (math.pi) / 2.5)
        self.move(self.size, 0, 0.7)
        self.move(0, 2.5, (math.pi / 2) / 2.5)
        self.move(self.size, 0, 0.5)
        self.move(0, 2.5, (math.pi / 2) / 2.5)
        self.move(self.size, 0, 0.7)
        self.move(0, 2.5, (math.pi) / 2.5)
        self.move(self.size, 0, 0.7)
        self.move(0, 2.5, (math.pi / 2) / 2.5)
        self.move(self.size, 0, 0.5)
        self.move(0, 2.5, (math.pi / 2) / 2.5)
        self.move(self.size, 0, 0.7)

    def draw_D(self):
        """
        ## Dibuja la letra 'D'.
        ## Combina un tramo recto con un "barrido" semicircular (aproximado) en la parte derecha.
        """
        self.move(0, 2.5, (math.pi / 2) / 2.5)
        self.move(self.size, 0, 1)
        self.move(0, 2.5, (math.pi / 2) / 2.5)
        ## Aquí se usa lin = self.size y ang = 2 para generar un arco
        ## (note que la velocidad angular aquí es 2 y no 2.5).
        self.move(self.size, 2, (math.pi) / 2)

    def draw_J(self):
        """
        ## Dibuja la letra 'J'.
        ## Trazos: parte superior, giro hacia abajo, tallo vertical y curvatura en la base.
        """
        self.move(self.size, 0, 0.5)
        self.move(0, 2.5, (math.pi) / 2.5)
        self.move(self.size, 0, 0.5)
        self.move(0, -2.5, (math.pi / 2) / 2.5)
        self.move(self.size, 0, 1)
        ## Último tramo incluye simultáneamente componente lineal y angular.
        self.move(self.size, -4, (math.pi) / 4)

    def draw_A(self):
        """
        ## Dibuja la letra 'A'.
        ## Incluye líneas verticales y diagonales, además de una barra horizontal intermedia.
        """
        self.move(0, 2.5, (math.pi / 2) / 2.5)
        # Left vertical down
        self.move(self.size, 0, 1)
        self.move(0, -2.5, (math.pi) / 2.5)
        self.move(self.size, 0, 1)
        self.move(0, -2.5, (math.pi / 2) / 2.5)
        self.move(self.size, 0, 0.7)
        self.move(0, -2.5, (math.pi / 2) / 2.5)
        self.move(self.size, 0, 1)
        self.move(0, -2.5, (math.pi) / 2.5)
        self.move(self.size, 0, 0.4)
        self.move(0, 2.5, (math.pi / 2) / 2.5)
        self.move(self.size, 0, 0.7)

    def draw_M(self):
        """
        ## Dibuja la letra 'M'.
        ## Se construye a partir de dos verticales y trazos inclinados que forman el "pico" central.
        """
        self.move(0, 2.5, (math.pi / 2) / 2.5)
        self.move(self.size, 0, 1)
        self.move(0, -2.5, (math.pi) / 2.5)
        self.move(self.size, 0, 1)
        self.move(0, -2.5, ((math.pi / 2) + (math.pi / 4)) / 2.5)
        self.move(self.size, 0, 0.4)
        self.move(0, 2.5, ((math.pi / 4) + (math.pi / 4)) / 2.5)
        self.move(self.size, 0, 0.4)
        self.move(0, -2.5, ((math.pi / 2) + (math.pi / 4)) / 2.5)
        self.move(self.size, 0, 1)

    def draw_O(self):
        """
        ## Dibuja la letra 'O' como un círculo completo.
        ## Se emplea la cinemática de un robot diferencial con:
        ##   - velocidad lineal constante (self.size)
        ##   - velocidad angular constante (omega)
        ##   - tiempo de integración t = 2π / omega
        """
        omega = 2.5
        full_circle_time = (2 * math.pi) / omega
        self.move(self.size, omega, full_circle_time)

    def draw_R(self):
        """
        ## Dibuja la letra 'R'.
        ## Esencialmente combina la parte inicial de una 'P' con una "pierna" diagonal.
        """
        self.move(0, 2.5, (math.pi / 2) / 2.5)
        self.move(self.size, 0, 1)
        self.move(0, -2.5, (math.pi) / 2.5)
        self.move(self.size, 0, 1)
        self.move(0, -2.5, (math.pi / 2) / 2.5)
        self.move(self.size, -4, (math.pi) / 4)
        self.move(0, 2.5, ((math.pi / 2) + (math.pi / 4)) / 2.5)
        self.move(self.size, 0, 0.8)

    ## ============================ BUCLE PRINCIPAL DEL NODO ============================

    def run(self):
        """
        ## Bucle principal del nodo:
        ## - Inicializa la posición absoluta de la tortuga.
        ## - Espera teclas del usuario.
        ## - Según la tecla, dibuja letras en la línea superior (E,D,J) o inferior (A,M,O,R).
        """
        # Force start position
        ## Teletransportamos inicialmente a la tortuga a una posición conocida,
        ## orientada hacia arriba (π/2).
        self.set_pen(False)
        req = TeleportAbsolute.Request()
        req.x = 1.0
        req.y = 10.0
        req.theta = math.pi / 2
        self.teleport.call_async(req)
        time.sleep(0.3)
        self.set_pen(True)

        ## Bucle principal: se ejecuta mientras ROS2 esté activo.
        while rclpy.ok():
            ## Lectura no bloqueante de una tecla.
            key = get_key()

            ## Tecla 'q' se usa para salir del programa.
            if key == 'q':
                break

            ## ---------------- LÍNEA SUPERIOR (E, D, J) ----------------
            if key.lower() in ['e', 'd', 'j']:
                ## Posiciona el "cursor" de la línea superior y orienta a la tortuga
                ## hacia la izquierda (theta = π).
                self.go_to(self.x_top, self.y_top, math.pi)

                ## Llama a la rutina de la letra correspondiente.
                if key.lower() == 'e':
                    self.draw_E()
                if key.lower() == 'd':
                    self.draw_D()
                if key.lower() == 'j':
                    self.draw_J()

                ## Avanza el cursor horizontalmente para la siguiente letra
                self.x_top += self.spacing

            ## ---------------- LÍNEA INFERIOR (A, M, O, R) ----------------
            if key.lower() in ['a', 'm', 'o', 'r']:
                ## Posiciona el "cursor" de la línea inferior y orienta a la tortuga
                ## hacia la izquierda (theta = π).
                self.go_to(self.x_bottom, self.y_bottom, math.pi)

                ## Llama a la rutina de la letra correspondiente.
                if key.lower() == 'a':
                    self.draw_A()
                if key.lower() == 'm':
                    self.draw_M()
                if key.lower() == 'o':
                    self.draw_O()
                if key.lower() == 'r':
                    self.draw_R()

                ## Avanza el cursor de la línea inferior
                self.x_bottom += self.spacing


## ============================ ENTRADA PRINCIPAL DEL PROGRAMA ============================

def main():
    """
    ## Punto de entrada del nodo:
    ## - Inicializa el cliente de ROS 2.
    ## - Crea el nodo TurtleWriter.
    ## - Ejecuta su bucle principal.
    ## - Destruye el nodo y apaga ROS 2 al finalizar.
    """
    rclpy.init()
    node = TurtleWriter()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    ## Si se ejecuta este archivo directamente (no importado como módulo),
    ## se llama a main().
    main()
