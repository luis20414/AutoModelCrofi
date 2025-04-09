import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64
import serial
import time
import math

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        self.subscription_angle = self.create_subscription(Float64, 'angle_servo', self.listener_callback, 10)
        
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Cambiar a 115200 si es necesario
            time.sleep(2)  # Esperar a que el Arduino esté listo
            self.get_logger().info('Arduino ready')
        except Exception as e:
            self.get_logger().error(f'Error connecting with Arduino: {e}')
            self.arduino = None
           
    def listener_callback(self, msg):
        if self.arduino:
            wheel_angle = str(round((-1) * msg.data, 3)) + '\n'
            self.arduino.write(wheel_angle.encode())
            self.get_logger().info(f"Sent angle: {msg.data}")
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



