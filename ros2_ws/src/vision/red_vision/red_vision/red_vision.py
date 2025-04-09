import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Importar String para publicar "s" o "n"
from cv_bridge import CvBridge
import pytesseract  # Importar Tesseract OCR

bridge = CvBridge()


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(Image, 'camera', self.red_callback, 10)
        self.publisher_ = self.create_publisher(String, 'camera_red', 10)  # Cambiar a String
        
    def red_callback(self, msg):    
        frame = bridge.imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Definir el rango de color rojo
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        # Crear máscaras para los rangos de color rojo
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Encontrar contornos en la máscara
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        detected = False  # Variable para rastrear si se detectó un octágono válido

        for contour in contours:
            # Aproximar el contorno a un polígono
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Verificar si el polígono tiene 8 lados (octágono)
            if len(approx) == 8:
                # Procesar la señal de alto detectada
                x, y, w, h = cv2.boundingRect(contour)
                roi = frame[y:y+h, x:x+w]

                # Convertir la ROI a escala de grises
                img_scene = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

                # Usar OCR para buscar texto dentro del octágono
                text = pytesseract.image_to_string(img_scene, lang='eng')  # Cambia 'eng' a 'spa' si buscas texto en español
                text = text.strip().upper()  # Normalizar texto a mayúsculas

                # Verificar si el texto contiene "ALTO" o "STOP"
                if "ALTO" in text or "STOP" in text:
                    self.get_logger().info(f"Señal detectada con texto: {text}")
                    self.publisher_.publish(String(data="s"))  # Publicar "s" si se detecta
                    detected = True
                    break  # Salir del bucle si se detecta un octágono válido

        if not detected:
            self.publisher_.publish(String(data="n"))  # Publicar "n" si no se detectó nada


def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
