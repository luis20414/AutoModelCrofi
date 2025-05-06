#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import time
import pytesseract

class TrafficSignDetector(Node):
    def __init__(self):
        super().__init__('traffic_sign_detector')
        #Configurar Tesseract (ajusta según tu instalación)
        pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'
        # Inicializar puente OpenCV-ROS
        self.bridge = CvBridge()
        # Publicador para el valor numérico
        self.stop_pub = self.create_publisher(Bool, '/stop', 10)
        # Publicador para la imagen procesada
        self.processed_pub = self.create_publisher(Image, '/red_image', 10)
        # Suscriptor a la cámara
        self.camera_sub = self.create_subscription(Image, '/camera', self.image_callback, 10)
        
        self.frame_count = 0
        self.last_detection_time = 0
        self.get_logger().info("Nodo de detección de señales inicializado")

    def image_callback(self, msg):
        try:
            # Verificar el formato de la imagen
            if msg.encoding != 'bgr8':
                self.get_logger().error(f"Formato de imagen no compatible: {msg.encoding}")
                return
            
            # Convertir mensaje ROS a imagen OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Reducir resolución
            frame = cv2.resize(frame, (640, 480))
            # Procesamiento de imagen
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Rangos para color rojo
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            
            # Publicar la imagen procesada
            processed_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            self.processed_pub.publish(processed_msg)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            detected = False
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 500:  # Filtrar áreas pequeñas
                    continue
                    
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                
                if len(approx) == 8:  # Verificar si es un octágono
                    self.get_logger().info("Octágono detectado")
                    detected = True
                    # Extraer región de interés (ROI) del contorno detectado
                    x, y, w, h = cv2.boundingRect(contour)
                    roi = frame[y:y+h, x:x+w]
                    
                    # Convertir ROI a escala de grises
                    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                    
                    # Aplicar umbral para mejorar el contraste
                    _, thresh_roi = cv2.threshold(gray_roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                    
                    # Usar OCR para detectar texto en el ROI
                    text = pytesseract.image_to_string(thresh_roi, config='--psm 8').strip()
                    
                    if text:  # Si se detecta texto
                        self.get_logger().info(f"Texto detectado en el octágono: {text}")
                        detected = True
                        
            # Publicar valores según detección
            msg = Bool()
            msg.data = detected
            self.stop_pub.publish(msg)
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()