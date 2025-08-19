import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Float64, String
import math
from rclpy.qos import qos_profile_sensor_data


class ParkingAssistant(Node):
    def __init__(self):
        super().__init__('parking_assistant')

        # Suscripción al LiDAR
        self.subscription_lidar = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data
        )

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

        # Estado del vehículo
        self.is_parking = False
        self.detected_first_car = False
        self.detected_wall = False
        self.detected_second_car = False

        # Mensaje inicial
        self.get_logger().info("Nodo ParkingAssistant corriendo...")

        # Configuración inicial del coche
        self.initialize_vehicle()

    def initialize_vehicle(self):
        """
        Configura el coche al inicio del nodo.
        """
        # Configurar la velocidad inicial a 1540
        self.speed_publisher.publish(Int32(data=1570))
        self.get_logger().info("Velocidad inicial configurada a 1540.")

        # Configurar el servo en 0
        self.steering_publisher.publish(Float64(data=0.0))
        self.get_logger().info("Servo inicial configurado en 0.")

    def lidar_callback(self, msg):
        # Procesar los datos del LiDAR
        ranges = msg.ranges
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        # Paso 1: Detectar el primer coche a la derecha
        if not self.detected_first_car:
            if self.detect_car_on_right(ranges, angle_min, angle_increment):
                self.detected_first_car = True
                self.get_logger().info("Primer coche detectado a la derecha.")
                self.create_timer(1.0, lambda: None)  # Esperar 1 segundo antes de continuar
            return

        # Paso 2: Detectar la pared a la derecha
        if not self.detected_wall:
            if self.is_wall_near_right(ranges, angle_min, angle_increment):
                self.detected_wall = True
                self.get_logger().info("Pared detectada a la derecha.")
                self.create_timer(1.0, lambda: None)  # Esperar 1 segundo antes de continuar
            return

        # Paso 3: Detectar el segundo coche a la derecha
        if not self.detected_second_car:
            if self.detect_car_on_right(ranges, angle_min, angle_increment):
                self.detected_second_car = True
                self.get_logger().info("Segundo coche detectado a la derecha. Iniciando maniobra de estacionamiento.")
                self.create_timer(1.0, self.park_vehicle)  # Esperar 1 segundo antes de iniciar la maniobra
            return

    def detect_car_on_right(self, ranges, angle_min, angle_increment):
        """
        Detecta si hay un coche a la derecha en un rango de ángulos.
        """
        consecutive_points = 0
        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            if -math.pi / 2 <= angle <= -math.pi / 4:  # Lado derecho
                if 0 < distance <= 0.3:  # Detectar un coche a 30 cm
                    consecutive_points += 1
                    if consecutive_points >= 2:  # Requiere al menos 5 puntos consecutivos
                        self.get_logger().info(f"Coche detectado a la derecha a {distance:.2f} m.")
                        return True
                else:
                    consecutive_points = 0  # Reiniciar si no cumple la condición
        return False

    def is_wall_near_right(self, ranges, angle_min, angle_increment):
        """
        Verifica si hay una pared a la derecha.
        """
        consecutive_points = 0
        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            if -math.pi / 2 <= angle <= -math.pi / 4:  # Lado derecho
                if 0 < distance <= .7:  # Detectar pared a la derecha
                    consecutive_points += 1
                    if consecutive_points >= 40:  # Requiere al menos 5 puntos consecutivos
                        self.get_logger().info(f"Pared detectada a la derecha a {distance:.2f} m.")
                        return True
                else:
                    consecutive_points = 0  # Reiniciar si no cumple la condición
        return False

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
        self.speed_publisher.publish(Int32(data=1340))  # Velocidad de reversa
        self.get_logger().info("Retrocediendo hacia el lado contrario...")
        self.create_timer(5.0, self.align_vehicle)  # Retroceder por 5 segundos

    def align_vehicle(self):
        # Paso 2: Verificar la posición de la pared y los coches laterales
        self.get_logger().info("Verificando posición de la pared y coches laterales...")
        self.adjust_position()

    def adjust_position(self):
        """
        Ajusta la posición del vehículo basándose en la distancia a la pared y a los coches laterales.
        """
        # Configurar velocidad y servo antes de ajustar
        self.speed_publisher.publish(Int32(data=1560))
        self.steering_publisher.publish(Float64(data=0.0))

        # Obtener las distancias laterales y frontales
        left_distance = self.get_side_distance(-math.pi / 2, -math.pi / 4)  # Lado izquierdo
        right_distance = self.get_side_distance(math.pi / 4, math.pi / 2)  # Lado derecho

        # Ajustar la posición si las distancias no son iguales
        if abs(left_distance - right_distance) > 0.05:  # Tolerancia de 5 cm
            if left_distance > right_distance:
                self.steering_publisher.publish(Float64(data=0.2))  # Girar ligeramente a la derecha
            else:
                self.steering_publisher.publish(Float64(data=-0.2))  # Girar ligeramente a la izquierda
            self.speed_publisher.publish(Int32(data=1540))  # Avanzar lentamente
            self.get_logger().info(f"Ajustando posición: Izquierda={left_distance:.2f}m, Derecha={right_distance:.2f}m")
            self.create_timer(1.0, self.enter_parking_space)
        else:
            self.get_logger().info("Posición alineada. Entrando al espacio de estacionamiento.")
            self.enter_parking_space()

    def enter_parking_space(self):
        # Paso 3: Entrar de frente al espacio de estacionamiento
        self.steering_publisher.publish(Float64(data=0.0))  # Dirección recta
        self.speed_publisher.publish(Int32(data=1540))  # Avanzar hacia adelante
        self.get_logger().info("Entrando de frente al espacio de estacionamiento...")
        self.create_timer(3.0, self.stop_vehicle)

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
