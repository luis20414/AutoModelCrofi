import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
#from geometry_msgs import Polygon
bridge = CvBridge()

whiteBajo = np.array([14, 0, 176], np.uint8) #modificar de acuerdo a como se vea mejor la imagen usando el codigo B_Y_N
whiteAlto = np.array([179, 40, 255], np.uint8)#Esta parte es para blanco

cv2.namedWindow('3. Bordes + Lineas Detectadas', cv2.WINDOW_NORMAL)

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.subscription = self.create_subscription(Image, '/hardware/camera/rgb', self.listener_callback, 1)
        timer_period = 0.03  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_ = self.create_publisher(Image, '/vision/lane_detector/rgb', 10)
        self.linesP=None
        
    def listener_callback(self, msg):
        self.get_logger().info("Received image: %dx%d" %(msg.width,msg.height))
        frame=bridge.imgmsg_to_cv2(msg)
        
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(frameHSV, whiteBajo, whiteAlto)
        kernel = np.ones((2, 2), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
        edges = cv2.Canny(mask_clean, 100, 200)
        result_img = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        linesP = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, None, 50, 10)

        # Dibujar las líneas detectadas
        if linesP is not None:
            for i in range(len(linesP)):
                l = linesP[i][0]
                cv2.line(result_img, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)

        cv2.imshow('3. Bordes + Lineas Detectadas', result_img)
        cv2.waitKey(10)
        
    def lane_Publisher(self):
        if not self.linesP:
            return
        result_img=cv2.cvtColor(self.linesP,cv2.COLOR_BGR2HSV)
        
        self.publisher_.publish(bridge.cv2_to_imgmsg(result_img))    
        
def main(args=None):
    print('Hi from lane_detector.')
    rclpy.init(args=args)
    lane_detector = LaneDetector()
    rclpy.spin(lane_detector)
    
    lane_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
