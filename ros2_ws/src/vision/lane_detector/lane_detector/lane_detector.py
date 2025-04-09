import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

blackBajo = np.array([0, 0, 0], np.uint8)  # Esta para negro
blackAlto = np.array([255, 255, 80], np.uint8)

#cv2.namedWindow('Bordes + Lineas Detectadas', cv2.WINDOW_NORMAL)

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.subscription = self.create_subscription(Image, 'camera_recorted', self.listener_callback, 1)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.lane_publisher)
        self.publisher_ = self.create_publisher(Float64MultiArray, 'lane_deviation', 10)

        self.image_publisher = self.create_publisher(Image, 'processed_image', 10)


        self.linesP = None

    def listener_callback(self, msg):
        #self.get_logger().info("Received image: %dx%d" % (msg.width, msg.height))
        frame = bridge.imgmsg_to_cv2(msg)
         # Calcular el centro de la imagen
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2

        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(frameHSV, blackBajo, blackAlto)
        kernel = np.ones((2, 2), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
        #edges = cv2.Canny(mask_clean, 100, 200)
        result_img = cv2.cvtColor(mask_clean, cv2.COLOR_GRAY2BGR) #Quitamos el canny para que vea mejorlas lineas



        self.linesP = cv2.HoughLinesP(mask_clean, 1, np.pi / 180, 50, None, 50, 10)
        if self.linesP is not None:
            filtered_lines = []        
            for line in self.linesP:
                x1, y1, x2, y2 = line[0]
                angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi  # Calcula el ángulo en grados
                # Filtrar líneas con ángulo entre -60° y 60°
                if 60 <= abs(angle) <= 120 :
                    filtered_lines.append(line)  # Guardar la línea filtrada
                    cv2.line(result_img, (x1, y1), (x2, y2), (0, 0, 255), 3, cv2.LINE_AA)  # Dibujar la línea
            # Actualizar self.linesP con las líneas filtradas en base al angulo
            self.linesP = np.array(filtered_lines) if filtered_lines else None
        #cv2.imshow('Bordes + Lineas Detectadas', result_img)
        cv2.waitKey(10)

        img_msg = bridge.cv2_to_imgmsg(result_img, encoding="bgr8")
        self.image_publisher.publish(img_msg)

    def lane_publisher(self):
        if self.linesP is not None:
            msg = Float64MultiArray()
            msg.data = [float(x) for x in self.linesP.flatten().tolist()]
            self.publisher_.publish(msg) #envia las lineas entre 60 y 120

def main(args=None):
    rclpy.init(args=args)
    lane_detector = LaneDetector()
    rclpy.spin(lane_detector)
    lane_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
