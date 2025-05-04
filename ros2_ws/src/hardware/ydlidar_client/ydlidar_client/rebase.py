import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64, Float64MultiArray, Bool
import numpy as np
import time  # Importar para medir el tiempo transcurrido


class RebaseNode(Node):
    def __init__(self):
        super().__init__('rebase_node')

        # Variables de estado
        self.rebasando = False
        self.fase_actual = 0  # 0: No rebasando, 1: Verificar izquierda, 2: Cambiar carril, 3: Reincorporarse, 4: Regresar al carril original, 5: Finalizar
        self.tiempo_cambio_carril = 0  # Tiempo registrado durante el cambio de carril
        self.colision_detectada = False  # Nueva variable para controlar si hay colisión

        # Crear publicadores
        self.driver_publisher = self.create_publisher(Int32, '/target_speed', 10)
        self.servo_publisher = self.create_publisher(Float64, 'steering', 10)

        # Crear suscripciones a los datos procesados del LiDAR
        self.degrees_subscription = self.create_subscription(
            Float64MultiArray, 'degreesLiDar', self.degrees_callback, 10)
        self.ranges_subscription = self.create_subscription(
            Float64MultiArray, 'rangesLiDar', self.ranges_callback, 10)

        # Suscripción al tópico de colisión
        self.colision_subscription = self.create_subscription(
            Bool, '/colision', self.colision_callback, 10)

        # Variables para almacenar los datos recibidos
        self.angles = []
        self.ranges = []

    def colision_callback(self, msg):
        # Actualizar el estado de colisión
        self.colision_detectada = msg.data
        if self.colision_detectada:
            self.get_logger().info("Colisión detectada, deteniendo procesamiento de rebase")

    def degrees_callback(self, msg):
        # Guardar los ángulos recibidos
        self.angles = np.array(msg.data)

    def ranges_callback(self, msg):
        # Guardar las distancias recibidas
        self.ranges = np.array(msg.data)

        # Procesar los datos si ambos están disponibles
        if len(self.angles) > 0 and len(self.ranges) > 0:
            self.procesar_rebase()

    def procesar_rebase(self):
        # Detener el procesamiento si hay colisión
        if self.colision_detectada:
            self.get_logger().info("Procesamiento de rebase detenido por colisión")
            return

        if self.rebasando:
            if self.fase_actual == 1:
                if self.verificar_izquierda():
                    self.fase_actual = 2
                    self.get_logger().info("Carril izquierdo despejado, cambiando de carril")
            elif self.fase_actual == 2:
                if self.cambiar_carril():
                    self.fase_actual = 3
                    self.get_logger().info("Cambio de carril completado, iniciando reincorporación")
            elif self.fase_actual == 3:
                if self.reincorporarse():
                    self.fase_actual = 4
                    self.get_logger().info("Reincorporación completada, regresando al carril original")
            elif self.fase_actual == 4:
                if self.regresar_carril_original():
                    self.fase_actual = 5
                    self.get_logger().info("Regreso al carril original completado")
            elif self.fase_actual == 5:
                self.finalizar_rebase()
        else:
            if self.detectar_objeto_enfrente():
                self.rebasando = True
                self.fase_actual = 1
                self.get_logger().info("Iniciando maniobra de rebase")

    def detectar_objeto_enfrente(self):
        # Detectar objeto justo enfrente (-20° a 20°)
        indices_frente = (self.angles > -20) & (self.angles < 20)
        if np.any((self.ranges[indices_frente] < 0.45) & (self.ranges[indices_frente] != 0)):
            self.get_logger().info("Objeto detectado enfrente, iniciando rebase")
            return True
        return False

    def verificar_izquierda(self):
        # Verificar si el carril izquierdo está libre (-50° a -35°)
        indices_izquierda = (self.angles > -50) & (self.angles < -35)
        if np.any((self.ranges[indices_izquierda] < 0.35) & (self.ranges[indices_izquierda] != 0)):
            self.get_logger().info("Obstáculo detectado a la izquierda, deteniendo vehículo")
            self.driver_publisher.publish(Int32(data=1500))  # Detener el vehículo
            return False
        else:
            self.driver_publisher.publish(Int32(data=1560))  # Reanudar velocidad para cambiar de carril
            return True

    def cambiar_carril(self):
        # Cambiar al carril izquierdo (0.5 en servo) y avanzar hasta 25° < degree < 30°
        self.servo_publisher.publish(Float64(data=0.5))  # Girar a la izquierda
        start_time = time.time()  # Registrar el tiempo de inicio
        indices_cambio = (self.angles > 25) & (self.angles < 30)
        if np.any(indices_cambio):  # Verificar si hay algo en el rango de ángulos
            self.get_logger().info("Avanzando en el carril izquierdo")
            self.tiempo_cambio_carril = time.time() - start_time  # Calcular el tiempo transcurrido
            return False
        return True

    def reincorporarse(self):
        # Mover el servo al máximo a la derecha (-0.5) y avanzar hasta detectar un coche en el lado derecho (30° a 50°)
        self.servo_publisher.publish(Float64(data=-0.5))  # Girar a la derecha
        self.driver_publisher.publish(Int32(data=1560))  # Mantener velocidad mínima
        indices_derecha = (self.angles > 30) & (self.angles < 50)
        if np.any((self.ranges[indices_derecha] < 0.40) & (self.ranges[indices_derecha] != 0)):
            self.get_logger().info("Coche detectado a la derecha, ajustando dirección")
            self.servo_publisher.publish(Float64(data=0.2))  # Ajustar dirección
            self.driver_publisher.publish(Int32(data=1560))  # Mantener velocidad mínima
            # Esperar hasta que ya no haya nada en el lado derecho
            if not np.any((self.ranges[indices_derecha] < 0.40) & (self.ranges[indices_derecha] != 0)):
                return True
        return False

    def regresar_carril_original(self):
        # Cambiar al carril derecho (-0.5 en servo) y avanzar hasta 25° < degree < 30°
        self.servo_publisher.publish(Float64(data=-0.5))  # Girar a la derecha
        indices_cambio = (self.angles > 25) & (self.angles < 30)
        if np.any(indices_cambio):  # Verificar si hay algo en el rango de ángulos
            self.get_logger().info("Avanzando para regresar al carril original")
            self.servo_publisher.publish(Float64(data=0.5))  # Girar al máximo a la izquierda
            time.sleep(self.tiempo_cambio_carril)  # Girar por el tiempo registrado
            self.servo_publisher.publish(Float64(data=0.0))  # Enderezar dirección
            return False
        return True

    def finalizar_rebase(self):
        self.get_logger().info("Rebase completado, regresando al carril original")
        self.rebasando = False
        self.fase_actual = 0
        self.servo_publisher.publish(Float64(data=-0.2))  # Enderezar dirección
        self.driver_publisher.publish(Int32(data=1540))  # Reanudar velocidad normal


def main(args=None):
    rclpy.init(args=args)
    node = RebaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()