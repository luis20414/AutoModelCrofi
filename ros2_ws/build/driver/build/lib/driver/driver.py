import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Bool
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
        
        self.allow_drive = False  # <--- Bandera para permitir movimiento

        self.subscription_angle = self.create_subscription(Int32, '/target_speed', self.listener_callback, qos_profile)
        self.publisher_intermittent_lights = self.create_publisher(String, 'rebase', 10) 

        self.go_sub = self.create_subscription(Bool, '/go_state', self.go_callback, 10)
        self.stop_sub = self.create_subscription(Bool, '/stop_state', self.stop_callback, 10)
        self.go_on_sub = self.create_subscription(Bool, '/go_on_state', self.go_on_callback, 10)

        try:
            self.arduino = serial.Serial('/dev/arduino_nano', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info('Arduino ready')
        except Exception as e:
            self.get_logger().error(f'Error connecting with Arduino: {e}')
            self.arduino = None

    def go_callback(self, msg):
        if msg.data:
            self.allow_drive = True
            self.get_logger().info("Permitir conducción (go_state = True)")

    def stop_callback(self, msg):
        if msg.data:
            self.allow_drive = True
            self.get_logger().info("Permitir conducción (stop_state = True)")

    def go_on_callback(self, msg):
        if msg.data:
            self.allow_drive = True
            self.get_logger().info("Permitir conducción (go_on_state = True)")

    def listener_callback(self, msg):
        if self.allow_drive and self.arduino:
            try:
                self.arduino.write(struct.pack('<i', msg.data))  # Enviar el valor como entero
                self.arduino.flush()
                self.get_logger().info(f"Sent speed: {msg.data}")
                if msg.data == 1500:
                    self.publisher_intermittent_lights.publish(String(data='T'))
                else:
                    self.publisher_intermittent_lights.publish(String(data='N'))
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
        elif not self.allow_drive:
            self.get_logger().info("Movimiento no permitido. Ignorando velocidad recibida.")
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