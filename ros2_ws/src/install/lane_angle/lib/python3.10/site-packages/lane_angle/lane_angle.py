import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from std_msgs_msg import string
from cv_bridge import CvBridge
import cv2
#from geometry_msgs import Polygon
bridge = CvBridge()

class LaneAngle(Node):
    def __int__(self):
        super().__init__('lane_angle')
        self.subscription=self.create_subscription(Image, '/visual/lane_detector/rgb', self.listener_callback, 1)
        timer_period = 0.03
        self.publisher_ = self.create_publisher(Image, '/vision/lane_angle/rgb', 10)
        
    

def main():
    print('Hi from lane_angle.')


if __name__ == '__main__':
    main()
