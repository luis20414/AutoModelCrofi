import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial
import time
import struct


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self.allow_movement = False  # <--- Bandera para permitir movimiento

        self.subscription_angle = self.create_subscription(Float64, 'steering', self.listener_callback, qos_profile)
        self.go_sub = self.create_subscription(Bool, '/go_state', self.avanzar_callback, 10)
        self.go_on_sub = self.create_subscription(Bool, '/go_on_state', self.continuar_callback, 10)

        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info('Arduino ready')
        except Exception as e:
            self.get_logger().error(f'Error connecting with Arduino: {e}')
            self.arduino = None

    def avanzar_callback(self, msg):
        if msg.data:
            self.allow_movement = True
            self.get_logger().info("Movimiento permitido (go_state = True)")
        else:
            self.allow_movement = False
            self.get_logger().info("Movimiento detenido (go_state = False)")

    def continuar_callback(self, msg):
        if msg.data:
            self.allow_movement = True
            self.get_logger().info("Movimiento permitido (go_on_state = True)")
        else:
            self.allow_movement = False
            self.get_logger().info("Movimiento detenido (go_on_state = False)")

    def listener_callback(self, msg):
        if self.allow_movement and self.arduino:
            try:
                wheel_angle = round(msg.data, 3)
                self.arduino.write(struct.pack('<f', wheel_angle))
                self.arduino.flush()
                self.get_logger().info(f"Sent angle: {msg.data}")
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
        elif not self.allow_movement:
            self.get_logger().info("Movimiento no permitido. Ignorando ángulo recibido.")
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
