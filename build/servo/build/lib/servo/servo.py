import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial
import time
import struct

#
class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.subscription_angle = self.create_subscription(Float64, 'angle_servo', self.listener_callback, qos_profile)
        
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # cambiar a /dev/ttyACM0 o /dev/ttyACM1 dependiendo de la conexión
            time.sleep(2)  # Esperar a que el Arduino esté listo
            self.get_logger().info('Arduino ready')
        except Exception as e:
            self.get_logger().error(f'Error connecting with Arduino: {e}')
            self.arduino = None
           
    def listener_callback(self, msg):
        if self.arduino:
            try:
                wheel_angle = round(msg.data, 3)
                self.arduino.write(struct.pack('<f', wheel_angle))
                self.arduino.flush()
                self.get_logger().info(f"Sent angle: {msg.data}")
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
        else:
            self.get_logger().error("Arduino not connected.")


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



