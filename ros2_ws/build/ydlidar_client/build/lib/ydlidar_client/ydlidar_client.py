import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

def scan_callback(scan):
    count = int(scan.scan_time / scan.time_increment)
    print(f"[YDLIDAR INFO]: Escaneo recibido {scan.header.frame_id}[{count}]:")
    print(f"[YDLIDAR INFO]: Rango angular : [{math.degrees(scan.angle_min):.2f}, {math.degrees(scan.angle_max):.2f}]")

    for i in range(count):
        degree = math.degrees(scan.angle_min + scan.angle_increment * i)
        if -45 < degree < 45:
            print(f"[YDLIDAR INFO]: Angulo-distancia : [{degree:.2f}, {scan.ranges[i]:.2f}]")

def main(args=None):
    rclpy.init(args=args)
    node = Node("ydlidar_ros2_driver_client")
    
    qos_profile = qos.qos_profile_sensor_data
    node.create_subscription(
        LaserScan,
        'scan',
        scan_callback,
        qos_profile
    )
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()