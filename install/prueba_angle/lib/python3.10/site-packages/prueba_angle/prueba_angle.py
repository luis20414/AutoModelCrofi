import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

class LaneCalibrator(Node):
    def __init__(self):
        super().__init__('lane_calibrator')

        self.subscription_lines = self.create_subscription(
            Float64MultiArray, 'lane_deviation', self.listener_callback_lines, 1)
        
        self.subscription_image = self.create_subscription(
            Image, 'camera_recorted', self.listener_callback_image, 1)
        
        self.publisher_dist = self.create_publisher(Float64MultiArray, 'lane_distances', 10)

        self.linesP = None
        self.last_frame = None

        self.timer = self.create_timer(0.1, self.compute_and_publish)

    def listener_callback_lines(self, msg):
        self.linesP = np.array(msg.data).reshape(-1, 1, 4)

    def listener_callback_image(self, msg):
        self.last_frame = bridge.imgmsg_to_cv2(msg)

    def compute_and_publish(self):
        if self.linesP is None or self.last_frame is None:
            return

        height, width, _ = self.last_frame.shape
        center_image = width // 2
        y_target = height

        line_bases = []

        for line in self.linesP:
            x1, y1, x2, y2 = line[0]
            if y2 == y1:
                continue
            x = x1 + (y_target - y1) * (x2 - x1) / (y2 - y1)
            line_bases.append(x)

        if len(line_bases) < 2:
            self.get_logger().warn('No se detectaron al menos dos líneas válidas.')
            return

        left = min(line_bases)
        right = max(line_bases)

        dist_left = center_image - left
        dist_right = right - center_image

        # Publicar las distancias
        msg = Float64MultiArray()
        msg.data = [dist_left, dist_right]
        self.publisher_dist.publish(msg)

        self.get_logger().info(f"Distancia Izquierda: {dist_left:.2f} px | Derecha: {dist_right:.2f} px")

        # Mostrar imagen para depuración
        self.draw_debug(self.last_frame, left, right, center_image)

    def draw_debug(self, frame, left_x, right_x, center_image):
        y = frame.shape[0]
        cv2.line(frame, (int(left_x), y), (int(left_x), y - 30), (255, 0, 0), 2)
        cv2.line(frame, (int(right_x), y), (int(right_x), y - 30), (0, 255, 0), 2)
        cv2.line(frame, (int(center_image), y), (int(center_image), y - 40), (0, 0, 255), 2)
        cv2.putText(frame, "Izq", (int(left_x)-20, y-40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)
        cv2.putText(frame, "Der", (int(right_x)-20, y-40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        cv2.imshow("Lane Calibrator", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LaneCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
