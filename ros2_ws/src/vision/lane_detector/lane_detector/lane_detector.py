import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

blackBajo = np.array([0, 0, 0], np.uint8)  # Esta para negro
blackAlto = np.array([255, 255, 80], np.uint8)

#cv2.namedWindow('Bordes + Lineas Detectadas', cv2.WINDOW_NORMAL)

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.subscription = self.create_subscription(Image, 'camera', self.listener_callback, 1)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.lane_publisher)
        self.publisher_ = self.create_publisher(Float64MultiArray, 'lane_deviation', 10)
        self.image_publisher = self.create_publisher(Image, 'processed_image', 10)
        self.curvature_publisher = self.create_publisher(Float64, 'curvature_radius', 10)  # Nuevo publicador

        self.linesP = None

    def listener_callback(self, msg):
        frame = bridge.imgmsg_to_cv2(msg)
         # Calcular el centro de la imagen
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2

        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(frameHSV, blackBajo, blackAlto)
        kernel = np.ones((2, 2), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
        result_img = cv2.cvtColor(mask_clean, cv2.COLOR_GRAY2BGR) 

        self.linesP = cv2.HoughLinesP(mask_clean, 1, np.pi / 180, 20, None, 50, 10)
        if self.linesP is not None:
            filtered_lines = []        
            for line in self.linesP:
                x1, y1, x2, y2 = line[0]
                angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi  # Calcula el ángulo en grados
                # Filtrar líneas con ángulo entre -40° y 40°
                if 40 <= abs(angle) <= 140 :
                    filtered_lines.append(line)  # Guardar la línea filtrada
                    cv2.line(result_img, (x1, y1), (x2, y2), (0, 0, 255), 3, cv2.LINE_AA)  # Dibujar la línea
            # Agrupar líneas cercanas
            if filtered_lines:
                grouped_lines = []
                for line in filtered_lines:
                    x1, y1, x2, y2 = line[0]
                    added = False
                    for group in grouped_lines:
                        gx1, gy1, gx2, gy2 = group
                        # Si las líneas están cerca en posición y pendiente, agruparlas
                        if abs(x1 - gx1) < 20 and abs(y1 - gy1) < 20 and abs(x2 - gx2) < 20 and abs(y2 - gy2) < 20:
                            group[0] = (gx1 + x1) // 2
                            group[1] = (gy1 + y1) // 2
                            group[2] = (gx2 + x2) // 2
                            group[3] = (gy2 + y2) // 2
                            added = True
                            break
                    if not added:
                        grouped_lines.append([x1, y1, x2, y2])

                # Verificar que grouped_lines tiene la estructura correcta
                if all(isinstance(group, list) and len(group) == 4 for group in grouped_lines):
                    self.linesP = np.array(grouped_lines)
                else:
                    self.get_logger().error("Grouped lines have an invalid structure.")
                    self.linesP = None
                    
            else:
                self.linesP = None

            if self.linesP is not None:
                left_lines = []
                right_lines = []

                for line in self.linesP:
                    x1, y1, x2, y2 = line[0]
                    slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0
                    if slope < 0:  # Línea izquierda
                        left_lines.append(line)
                    elif slope > 0:  # Línea derecha
                        right_lines.append(line)

                def average_line(lines):
                    if len(lines) == 0:
                        return None
                    x1_avg = int(np.mean([line[0][0] for line in lines]))
                    y1_avg = int(np.mean([line[0][1] for line in lines]))
                    x2_avg = int(np.mean([line[0][2] for line in lines]))
                    y2_avg = int(np.mean([line[0][3] for line in lines]))
                    return [x1_avg, y1_avg, x2_avg, y2_avg]

                left_avg = average_line(left_lines)
                right_avg = average_line(right_lines)

                self.linesP = np.array([left_avg, right_avg]) if left_avg and right_avg else None

            # Calcular el radio de curvatura
            curvature_radius = self.estimate_curvature(self.linesP)
            if curvature_radius is not None:
                curvature_msg = Float64()
                curvature_msg.data = curvature_radius
                self.curvature_publisher.publish(curvature_msg)  # Publicar el radio de curvatura

        cv2.imshow('Bordes + Lineas Detectadas', result_img)
        cv2.waitKey(10)

        img_msg = bridge.cv2_to_imgmsg(result_img, encoding="bgr8")
        self.image_publisher.publish(img_msg)

    def estimate_curvature(self, lines):
        if lines is None or len(lines) < 2:
            return None
        # Suponiendo lines = [left_line, right_line]
        left_x = np.mean([lines[0][0], lines[0][2]])
        right_x = np.mean([lines[1][0], lines[1][2]])
        lane_width = abs(right_x - left_x)
        # Modelo simplificado: radio ≈ (ancho_carril_píxeles * ancho_carril_real) / (2 * desviación_angular)
        return (lane_width * 0.4) / (2 * np.sin(np.mean([lines[0][1], lines[1][1]])))  # 0.4m = ancho real del carril

    def lane_publisher(self):
        if self.linesP is not None:
            msg = Float64MultiArray()
            msg.data = [float(x) for x in self.linesP.flatten().tolist()]
            self.publisher_.publish(msg) #envia las lineas entre 40 y 140

def main(args=None):
    rclpy.init(args=args)
    lane_detector = LaneDetector()
    rclpy.spin(lane_detector)
    lane_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
