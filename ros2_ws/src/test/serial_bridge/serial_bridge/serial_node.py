import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import struct
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.servo_sub = self.create_subscription(
            Float32,
            'servo_angle',
            self.servo_callback,
            qos)
            
        self.motor_sub = self.create_subscription(
            Float32,
            'motor_command',
            self.motor_callback,
            qos)
            
        self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1) # Cambia el puerto (/dev/ttyXXXX) según tu configuración
        
    def servo_callback(self, msg):
        # Enviar como float (0-180)
        if 0 <= msg.data <= 180:
            self.arduino.write(struct.pack('<f', msg.data))

    def motor_callback(self, msg):
        # Enviar como float especial
        self.arduino.write(struct.pack('<f', msg.data))

def main(args=None):
    rclpy.init(args=args)
    bridge = SerialBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()
