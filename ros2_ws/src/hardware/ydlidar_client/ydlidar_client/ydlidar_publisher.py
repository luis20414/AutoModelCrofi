import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
import numpy as np
from rclpy.qos import qos_profile_sensor_data


class YDLidarPublisher(Node):
    def __init__(self):
        super().__init__('ydlidar_ros2_driver_client')
        # Publicadores para ángulos y distancias
        self.degrees_publisher = self.create_publisher(Float64MultiArray, 'degreesLiDar', 10)
        self.ranges_publisher = self.create_publisher(Float64MultiArray, 'rangesLiDar', 10)
        # Suscripción al tópico del LiDAR con QoS
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

    def scan_callback(self, scan):
        # Calcular el número de puntos detectados
        #count = int(scan.scan_time / scan.time_increment)
        count = len(scan.ranges)
        
        # Calcular los ángulos en grados
        angles = np.degrees(scan.angle_min + scan.angle_increment * np.arange(count))
        
        # Obtener los rangos (distancias)
        ranges = np.array(scan.ranges)

        # Crear mensajes de tipo Float64MultiArray
        angles_msg = Float64MultiArray()
        ranges_msg = Float64MultiArray()

        # Asignar los datos a los mensajes
        angles_msg.data = angles.tolist()  # Convertir a lista para el mensaje
        ranges_msg.data = ranges.tolist()  # Convertir a lista para el mensaje

        # Publicar los mensajes
        self.degrees_publisher.publish(angles_msg)
        self.ranges_publisher.publish(ranges_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YDLidarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
