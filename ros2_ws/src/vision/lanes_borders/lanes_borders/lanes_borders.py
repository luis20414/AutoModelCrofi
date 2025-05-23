import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Bool
from cv_bridge import CvBridge

bridge = CvBridge()

blackBajo = np.array([0, 0, 0], np.uint8)
blackAlto = np.array([255, 255, 80], np.uint8)
whiteBajo = np.array([0, 0, 200], np.uint8)  # Esta para blanco
whiteAlto = np.array([255, 255, 255], np.uint8)



class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.subscription = self.create_subscription(Image, 'camera', self.listener_callback, 1)
        self.publisher_ = self.create_publisher(Float64MultiArray, 'lane_borders', 10)
        self.image_publisher = self.create_publisher(Image, 'processed_image', 10)

        self.allow_processing = False  # Solo permitimos procesar en estado GO
        self.go_sub = self.create_subscription(Bool, '/go_state', self.go_callback, 10)

    def go_callback(self, msg):
        self.allow_processing = msg.data
        self.get_logger().info(f"go_state: {msg.data}")

    def listener_callback(self, msg):
        #if not self.allow_processing:
        #    return

        frame = bridge.imgmsg_to_cv2(msg)
        height, width = frame.shape[:2]

        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(frameHSV, whiteBajo, whiteAlto)
        kernel = np.ones((2, 2), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)

        left_border = mask_clean[height-2:height, 0:width//2]  
        right_border = mask_clean[height-2:height, width//2:width]
        left_indexes = cv2.findNonZero(left_border)
        right_indexes = cv2.findNonZero(right_border)

        left_point, right_point = -1, -1
        if left_indexes is not None:
            left_indexes = left_indexes[:, 0, 0]
            (_, left_point, __, ___) = cv2.minMaxLoc(left_indexes)
            left_point = int(left_point)
        if right_indexes is not None:
            right_indexes = right_indexes[:, 0, 0]
            (right_point, _, __, ___) = cv2.minMaxLoc(right_indexes)
            right_point = int(right_point)

        result_img = cv2.cvtColor(mask_clean, cv2.COLOR_GRAY2BGR) 
        if left_point > 0:
            cv2.circle(result_img, (left_point, height-2), 5, (0, 255, 0), -1)
        if right_point > 0:
            cv2.circle(result_img, (right_point + width//2, height-2), 5, (0, 255, 0), -1)

        img_msg = bridge.cv2_to_imgmsg(result_img, encoding="bgr8")
        self.image_publisher.publish(img_msg)

        msg = Float64MultiArray()
        msg.data = [float(left_point), float(right_point)]
        #self.get_logger().info("Puntos enviados")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    lane_detector = LaneDetector()
    rclpy.spin(lane_detector)
    lane_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()