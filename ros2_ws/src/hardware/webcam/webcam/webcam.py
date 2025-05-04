import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()


class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera', 10)
        self.cam = cv2.VideoCapture(0, cv2.CAP_V4L2)
        
        # Set lower resolution for the webcam
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Set width to 320 pixels
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # Set height to 240 pixels
        
        self.cam.set(cv2.CAP_PROP_POS_FRAMES, 0)
        
        fps = self.cam.get(cv2.CAP_PROP_FPS)
        timer_period = 1.0 / fps if fps > 0 else 0.05  # Usa el FPS del video o un valor predeterminado
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        ret, frame = self.cam.read()
        if not ret or frame is None:
            self.get_logger().warn("No se pudo leer un frame del video. Fin del video o error.")
            return
        frame=frame[80:195, :]
        self.get_logger().debug(f"Tipo de frame: {type(frame)}")
        
        self.publisher_.publish(bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

def main(args=None):
    print("Hola")
    rclpy.init(args=args)

    webcam_publisher = WebcamPublisher()

    rclpy.spin(webcam_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    webcam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
