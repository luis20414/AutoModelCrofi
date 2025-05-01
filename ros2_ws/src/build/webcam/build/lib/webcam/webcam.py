import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()


class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, '/hardware/camera/rgb', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cam=cv2.VideoCapture(0)
        
        
    def timer_callback(self):
        #msg = Image()
        #msg.data = 'Hello World: %d' % self.i
        #self.get_logger().info('Publishing: ')
        ret, frame = self.cam.read()
        if not ret:
            return
        #cv2.imshow("rgb",frame)
        #cv2.waitKey(10)
        self.publisher_.publish(bridge.cv2_to_imgmsg(frame))

def main(args=None):
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
