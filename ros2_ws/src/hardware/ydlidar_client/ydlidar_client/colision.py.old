import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math


class YDLidarClient(Node):
    def __init__(self):
        super().__init__('ydlidar_client')

        # Crear perfil QoS
        self.qos_profile = qos.qos_profile_sensor_data

        # Crear publicadores
        self.colision_publisher = self.create_publisher(Bool, '/colision', 10)

        # Crear suscripción
        self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            self.qos_profile
        )

    def scan_callback(self, scan):
        count = int(scan.scan_time / scan.time_increment)
        #print(f"[YDLIDAR INFO]: Escaneo recibido {scan.header.frame_id}[{count}]:")
        #print(f"[YDLIDAR INFO]: Rango angular : [{math.degrees(scan.angle_min):.2f}, {math.degrees(scan.angle_max):.2f}]")

        # Detectar colisiones
        publicacion = self.deteccionColisiones(scan, count)
        colision_msg = Bool()

        if publicacion:
            #print("Colision detectada")
            colision_msg.data = True
            self.colision_publisher.publish(colision_msg)
        else:
            #print("No hay colision")
            colision_msg.data = False
            self.colision_publisher.publish(colision_msg)

    def deteccionColisiones(self, scan, count):
        colision = False
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            if degree > -35 and degree < 35:
                if scan.ranges[i] < 0.35 and scan.ranges[i] != 0:
                    colision = True
                    break
        return colision


def main(args=None):
    rclpy.init(args=args)
    node = YDLidarClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()