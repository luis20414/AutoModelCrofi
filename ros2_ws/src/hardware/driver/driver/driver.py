import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial
import time
import struct


class DriverController(Node):
    def __init__(self):
        super().__init__('driver_controller')

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.subscription_angle = self.create_subscription(Int32, '/target_speed', self.listener_callback, qos_profile)
        self.publisher_intermittent_lights = self.create_publisher(String, 'rebase', qos_profile) 

        try:
            self.arduino = serial.Serial('/dev/arduino_nano', 115200, timeout=1)  # Cambiar a 115200 si es necesario
            time.sleep(2)  # Esperar a que el Arduino esté listo
            self.get_logger().info('Arduino ready')
        except Exception as e:
            self.get_logger().error(f'Error connecting with Arduino: {e}')
            self.arduino = None
           
    def listener_callback(self, msg):
        if self.arduino:
            try:
                self.arduino.write(struct.pack('<i', msg.data)) # Enviar el valor como entero
                self.arduino.flush()
                self.get_logger().info(f"Sent speed: {msg.data}")
                if msg.data == 1500:
                    self.publisher_intermittent_lights.publish(Int32(data='T'))
                else: 
                    self.publisher_intermittent_lights.publish(Int32(data='N'))
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
        else:
            self.get_logger().error("Arduino not connected.")


def main(args=None):
    rclpy.init(args=args)
    node = DriverController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
