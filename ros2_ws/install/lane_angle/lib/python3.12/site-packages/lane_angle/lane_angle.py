import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
    
bridge = CvBridge()

class LaneAngle(Node):
    def __init__(self):
        super().__init__('lane_angle')
        
        self.subscription_lines = self.create_subscription(
            Float64MultiArray, 'lane_deviation', self.listener_callback_lines, 1)
        
        self.subscription_image = self.create_subscription(
            Image, 'camera_recorted', self.listener_callback_image, 1)
        
        self.publisher_ = self.create_publisher(Float64, 'angle_servo', 10)

        self.timer = self.create_timer(0.05, self.publish_angle)

        self.linesP = None
        self.last_frame = None

    def listener_callback_lines(self, msg):
        self.linesP = np.array(msg.data).reshape(-1, 1, 4)

    def listener_callback_image(self, msg):
        self.last_frame = bridge.imgmsg_to_cv2(msg)

    def publish_angle(self):
        if self.linesP is None or self.last_frame is None:
            return
        
        height, width, _ = self.last_frame.shape
        center_image = width // 2
        y_target = height  # Parte inferior de la imagen

        line_bases = []

        for line in self.linesP:
            x1, y1, x2, y2 = line[0]
            
            # Interpolación para encontrar x en y = altura (abajo)
            x = x1 + (y_target - y1) * (x2 - x1) / (y2 - y1)
            line_bases.append(x)

        if len(line_bases) < 2:
            self.get_logger().warn('No se detectaron al menos dos líneas válidas.')
            return

        # Tomamos los dos valores más extremos como bordes del carril
        left = min(line_bases)
        right = max(line_bases)
        lane_center = (left + right) / 2

        error = lane_center - center_image

        # Si el error es muy pequeño, asumimos que está alineado
        if abs(error) < 1.0:
            angle = 0.0
        else:
            D = 250.0  # Distancia horizontal de referencia (puedes ajustar)
            angle = np.round(np.arctan2(error, D),3)

        # Publicar el ángulo en r msg.data = np.clip(angle, -0.5, 0.5)adianes
        msg = Float64()
        msg.data = angle
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    lane_angle = LaneAngle()
    rclpy.spin(lane_angle)
    lane_angle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
