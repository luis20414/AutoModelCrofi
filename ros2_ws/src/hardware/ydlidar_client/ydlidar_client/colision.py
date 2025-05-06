import rclpy
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np


class Colision_Lidar(Node):
    def __init__(self):
        super().__init__('ydlidar_client')

        # Crear perfil QoS
        self.qos_profile = qos.qos_profile_sensor_data

        self.qos_profile_driver = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # Crear publicadores
        self.colision_publisher = self.create_publisher(Bool, '/colision', 10)
        self.driver_publisher = self.create_publisher(Int32, '/target_speed', 10)  # SOLO PARA PRUEBAS. BORRAR

        # Crear suscripciones a los datos procesados del LiDAR
        self.degrees_subscription = self.create_subscription(
            Float64MultiArray, 'degreesLiDar', self.degrees_callback, 10)
        self.ranges_subscription = self.create_subscription(
            Float64MultiArray, 'rangesLiDar', self.ranges_callback, 10)

        # Variables para almacenar los datos recibidos
        self.angles = []
        self.ranges = []

    def degrees_callback(self, msg):
        # Guardar los ángulos recibidos
        self.angles = np.array(msg.data)

    def ranges_callback(self, msg):
        # Guardar las distancias recibidas
        self.ranges = np.array(msg.data)

        # Procesar los datos si ambos están disponibles y tienen el mismo tamaño
        if len(self.angles) > 0 and len(self.ranges) > 0:
            if len(self.angles) == len(self.ranges):
                self.procesar_rebase()
            else:
                self.get_logger().warn("Los tamaños de angles y ranges no coinciden. Esperando sincronización...")

def procesar_rebase(self):
        # Verificar colisiones
        colision_detectada = self.deteccionColisiones()
        colision_msg = Bool()

        if colision_detectada:
            colision_msg.data = True
            self.colision_publisher.publish(colision_msg)
            self.driver_publisher.publish(Int32(data=1500))  # SOLO PARA PRUEBAS. BORRAR
        else:
            colision_msg.data = False
            self.colision_publisher.publish(colision_msg)

def deteccionColisiones(self):
    # Filtrar los ángulos dentro del rango de -35° a 35°
    indices_frente = (self.angles > -35) & (self.angles < 35)

    # Verificar si hay alguna distancia menor a 0.20 m en el rango frontal
    colision = np.any((self.ranges[indices_frente] < 0.20) & (self.ranges[indices_frente] != 0))
    return colision


def main(args=None):
    rclpy.init(args=args)
    node = Colision_Lidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()