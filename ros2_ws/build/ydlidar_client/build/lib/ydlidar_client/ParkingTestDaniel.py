import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64, String, Float64MultiArray
import numpy as np


class ParkingAssistant(Node):
    def __init__(self):
        super().__init__('parking_assistant')

        # Suscripciones a los arreglos de ángulos y distancias
        self.degrees_subscription = self.create_subscription(
            Float64MultiArray, 'degreesLiDar', self.degrees_callback, 10)
        self.ranges_subscription = self.create_subscription(
            Float64MultiArray, 'rangesLiDar', self.ranges_callback, 10)

        # Publicadores para el control del vehículo
        self.speed_publisher = self.create_publisher(Int32, '/target_speed', 10)
        self.steering_publisher = self.create_publisher(Float64, '/steering', 10)
        self.lights_publisher = self.create_publisher(String, '/rebase', 10)

        # Variables de detección de estacionamiento
        self.detecting_space = True 
        self.space_start = None
        self.space_end = None
        self.min_space_length = 0.5  # Longitud mínima del espacio (en metros)
        self.car_width = 0.185  # Ancho del carro (en metros)
        self.safety_distance = 0.1  # Distancia mínima de seguridad al fondo (en metros)

        # Variables de detección de estacionamiento
        self.angles = np.array([])
        self.ranges = np.array([])
        self.is_parking = False
        self.detected_first_car = False
        self.detected_wall = False
        self.detected_second_car = False

        # Mensaje inicial
        self.get_logger().info("Nodo ParkingAssistant corriendo...")

    def degrees_callback(self, msg):
        # Guardar los ángulos recibidos
        self.angles = np.array(msg.data)

    def ranges_callback(self, msg):
        # Guardar las distancias recibidas
        self.ranges = np.array(msg.data)

        # Procesar los datos si ambos están disponibles y tienen el mismo tamaño
        if len(self.angles) > 0 and len(self.ranges) > 0:
            if len(self.angles) == len(self.ranges):
                self.process_parking()
            else:
                self.get_logger().warn("Los tamaños de angles y ranges no coinciden. Esperando sincronización...")

    def process_parking(self):
        # Paso 1: Detectar el primer coche a la derecha
        if not self.detected_first_car:
            if self.detect_car_on_right():
                self.detected_first_car = True
                self.get_logger().info("Primer coche detectado a la derecha.")
            return

        # Paso 2: Detectar la pared a la derecha
        if not self.detected_wall:
            if self.is_wall_near_right():
                self.detected_wall = True
                self.get_logger().info("Pared detectada a la derecha.")
            return

        # Paso 3: Detectar el segundo coche a la derecha
        if not self.detected_second_car:
            if self.detect_car_on_right():
                self.detected_second_car = True
                self.get_logger().info("Segundo coche detectado a la derecha. Iniciando maniobra de estacionamiento.")
                self.park_vehicle()
            return

    def detect_car_on_right(self):
        """
        Detecta si hay un coche a la derecha en un rango de ángulos.
        """
        # Filtrar los ángulos entre -90° y -45° (lado derecho)
        indices_derecha = (self.angles >= -90) & (self.angles <= -45)

        # Verificar si hay al menos 5 puntos consecutivos con distancias <= 0.3 m
        distances = self.ranges[indices_derecha]
        consecutive_points = np.convolve(distances <= 0.3, np.ones(5, dtype=int), mode='valid')
        return np.any(consecutive_points >= 5)

    def is_wall_near_right(self):
        """
        Verifica si hay una pared a la derecha.
        """
        # Filtrar los ángulos entre -90° y -45° (lado derecho)
        indices_derecha = (self.angles >= -90) & (self.angles <= -45)

        # Verificar si hay al menos 5 puntos consecutivos con distancias <= 0.7 m
        distances = self.ranges[indices_derecha]
        consecutive_points = np.convolve(distances <= 0.7, np.ones(5, dtype=int), mode='valid')
        return np.any(consecutive_points >= 5)

    def park_vehicle(self):
        if self.is_parking:
            return  # Evitar iniciar otra maniobra si ya está estacionando

        self.is_parking = True
        self.get_logger().info("Iniciando maniobra de estacionamiento...")

        # Encender las luces intermitentes
        self.lights_publisher.publish(String(data='T'))  # Enviar comando para encender las luces
        self.get_logger().info("Luces intermitentes encendidas.")

        # Paso 1: Retroceder hacia el lado contrario
        self.steering_publisher.publish(Float64(data=0.5))  # Girar hacia el lado contrario
        self.speed_publisher.publish(Int32(data=1380))  # Velocidad de reversa
        self.get_logger().info("Retrocediendo hacia el lado contrario...")
        self.create_timer(5.0, self.stop_vehicle)

    def stop_vehicle(self):
        # Detener el vehículo
        self.speed_publisher.publish(Int32(data=1500))  # Detener el vehículo
        self.steering_publisher.publish(Float64(data=0.0))  # Dirección recta
        self.get_logger().info("Vehículo detenido. Maniobra completada.")
        self.is_parking = False

        # Apagar las luces intermitentes
        self.lights_publisher.publish(String(data='N'))  # Enviar comando para apagar las luces
        self.get_logger().info("Luces intermitentes apagadas.")


def main(args=None):
    rclpy.init(args=args)
    node = ParkingAssistant()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()