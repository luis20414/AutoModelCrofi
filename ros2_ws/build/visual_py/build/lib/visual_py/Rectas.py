import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Image
from cv_bridge import CVBridge

class WebcamSubscriber(Node):
    def __init__(self):
        super().__init__('webcam_subscriber')
        self.subscription = self.create_subscription(Image, '/hardware/camera/rgb', self.image_callback, 10)
        self.subscription

def main(args=None):
    rclpy.init(args=args)
    webcam_subscriber = WebcamSubscriber()
    rclpy.spin(webcam_subscriber)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    webcam_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
