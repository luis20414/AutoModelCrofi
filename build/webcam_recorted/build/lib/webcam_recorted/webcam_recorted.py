import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(Image,'camera',self.listener_callback,10)
        self.publisher_ = self.create_publisher(Image, 'camera_recorted', 10)
        

    def listener_callback(self, msg):
        # Convierte el mensaje ROS a imagen de OpenCV
        try:
            frame = bridge.imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Error al convertir la imagen: {e}")
            return

        # Extrae la mitad superior de la imagen
        height, width = frame.shape[:2]
        roi = frame[0:height // 2, :]

        # Publica la imagen recortada
        self.publisher_.publish(bridge.cv2_to_imgmsg(roi, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
