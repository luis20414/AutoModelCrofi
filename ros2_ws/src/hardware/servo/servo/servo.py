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

        # Banderas individuales para cada estado
        self.in_go_state = False
        self.in_overtake_state = False

        self.subscription_angle = self.create_subscription(Float64, 'steering', self.listener_callback, qos_profile)
        self.go_sub = self.create_subscription(Bool, '/go_state', self.go_callback, 10)
        self.overtake_sub = self.create_subscription(Bool, '/overtake_state', self.overtake_callback, 10)

        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info('Arduino ready')
        except Exception as e:
            self.get_logger().error(f'Error connecting with Arduino: {e}')
            self.arduino = None

    def go_callback(self, msg):
        self.in_go_state = msg.data
        self._update_movement_permission()

    def overtake_callback(self, msg):
        self.in_overtake_state = msg.data
        self._update_movement_permission()

    def _update_movement_permission(self):
        # Solo permitimos movimiento si está en GO o OVERTAKE
        if self.in_go_state or self.in_overtake_state:
            self.get_logger().info("Movimiento permitido (GO u OVERTAKE = True)")
        else:
            self.get_logger().info("Movimiento detenido (ambos estados False)")

    def listener_callback(self, msg):
        if (self.in_go_state or self.in_overtake_state) and self.arduino:
            try:
                wheel_angle = round(msg.data, 3)
                self.arduino.write(struct.pack('<f', wheel_angle))
                self.arduino.flush()
                self.get_logger().info(f"Ángulo enviado: {wheel_angle}")
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
        elif not (self.in_go_state or self.in_overtake_state):
            self.get_logger().info("Movimiento no permitido. Ignorando ángulo recibido.")
        else:
            self.get_logger().error("Arduino no conectado.")

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()