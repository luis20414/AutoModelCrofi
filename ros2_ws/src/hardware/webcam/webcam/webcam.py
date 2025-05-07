import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera', 10)
        self.cam = cv2.VideoCapture(0, cv2.CAP_V4L2)

        self.allow_camera = False  # <--- Bandera de control

        self.go_sub = self.create_subscription(Bool, '/go_state', self.go_callback, 10)
        self.stop_sub = self.create_subscription(Bool, '/stop_state', self.stop_callback, 10)
        self.go_on_sub = self.create_subscription(Bool, '/go_on_state', self.go_on_callback, 10)

        # Configura la resolución
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cam.set(cv2.CAP_PROP_POS_FRAMES, 0)

        fps = self.cam.get(cv2.CAP_PROP_FPS)
        timer_period = 1.0 / fps if fps > 0 else 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def go_callback(self, msg):
        if msg.data:
            self.allow_camera = True
            self.get_logger().info("Captura de cámara activada (go_state)")

    def stop_callback(self, msg):
        if msg.data:
            self.allow_camera = True
            self.get_logger().info("Captura de cámara activada (stop_state)")

    def go_on_callback(self, msg):
        if msg.data:
            self.allow_camera = True
            self.get_logger().info("Captura de cámara activada (go_on_state)")

    def timer_callback(self):
        if not self.allow_camera:
            return

        ret, frame = self.cam.read()
        if not ret or frame is None:
            self.get_logger().warn("No se pudo leer un frame del video.")
            return

        frame = frame[50:160, :]  # Recorte vertical
        self.publisher_.publish(bridge.cv2_to_imgmsg(frame, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()