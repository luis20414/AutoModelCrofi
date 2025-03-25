import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(Image,'camera',self.red_callback,10)
        self.publisher_ = self.create_publisher(Image, 'camera_red', 10)
        
    def red_callback(self, msg):    
        frame = bridge.imgmsg_to_cv2(msg)
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Definir el rango de color rojo
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        # Crear máscaras para los rangos de color rojo
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Aplicar la máscara a la imagen original
        roi = cv2.bitwise_and(frame, frame, mask=mask)

        # Convertir la región de interés (ROI) a escala de grises (solo el rojo detectado)
        img_scene = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    
        # Publica la imagen recortada
        self.publisher_.publish(bridge.cv2_to_imgmsg(img_scene))


def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
