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
        
        self.in_go_state = False
        self.in_overtake_state = False

        self.subscription_angle = self.create_subscription(Int32, '/target_speed', self.listener_callback, qos_profile)
        self.publisher_intermittent_lights = self.create_publisher(String, 'rebase', 10) 

        self.go_sub = self.create_subscription(Bool, '/go_state', self.go_callback, 10)
        self.overtake_sub = self.create_subscription(Bool, '/overtake_state', self.overtake_callback, 10)

        try:
            self.arduino = serial.Serial('/dev/arduino_nano', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info('Arduino ready')
        except Exception as e:
            self.get_logger().error(f'Error connecting with Arduino: {e}')
            self.arduino = None

    def go_callback(self, msg):
        self.in_go_state = msg.data
        self._update_drive_permission()

    def overtake_callback(self, msg):
        self.in_overtake_state = msg.data
        self._update_drive_permission()

    def _update_drive_permission(self):
        if self.in_go_state or self.in_overtake_state:
            self.get_logger().info("Conducción permitida (GO u OVERTAKE = True)")
        else:
            self.get_logger().info("Conducción detenida (ambos estados False)")

    def listener_callback(self, msg):
        if (self.in_go_state or self.in_overtake_state) and self.arduino:
            try:
                self.arduino.write(struct.pack('<i', msg.data))  # Enviar el valor como entero
                self.arduino.flush()
                self.get_logger().info(f"Velocidad enviada: {msg.data}")
                if msg.data == 1500:
                    self.publisher_intermittent_lights.publish(String(data='T'))
                else:
                    self.publisher_intermittent_lights.publish(String(data='N'))
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
        elif not (self.in_go_state or self.in_overtake_state):
            self.get_logger().info("Conducción no permitida. Ignorando velocidad recibida.")
        else:
            self.get_logger().error("Arduino no conectado.")


def main(args=None):
    rclpy.init(args=args)
    node = DriverController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
