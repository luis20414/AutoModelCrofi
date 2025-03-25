import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        # Suscribirse al tópico donde se publica la imagen de la cámara
        self.subscription = self.create_subscription(Image, 'camera_red', self.image_callback, 10)
        self.bridge = CvBridge()

        # Cargar imagen de referencia
        imageStop = r'/home/crofi/ros2_ws/src/vision/Stop1.png'
        self.img_object = cv2.imread(imageStop, cv2.IMREAD_GRAYSCALE)

        # Verificar si la imagen de referencia se cargó correctamente
        if self.img_object is None:
            raise FileNotFoundError("No se pudo abrir la imagen de referencia")

        # Crear detector ORB para detectar 1000 características en las imágenes
        self.orb = cv2.ORB_create(nfeatures=300)

        # Extraer keypoints y descriptores de la imagen de referencia
        self.keypoints_obj, self.descriptors_obj = self.orb.detectAndCompute(self.img_object, None)

        # Configurar el emparejador FLANN para encontrar coincidencias entre descriptores
        index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def image_callback(self, msg):
        # Convertir imagen ROS2 a OpenCV
        img_scene = self.bridge.imgmsg_to_cv2(msg)
        
        #img_scene = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detectar características en la imagen de la cámara
        keypoints_scene, descriptors_scene = self.orb.detectAndCompute(img_scene, None)

        # Verificar si se detectaron descriptores en la imagen de la cámara
        if descriptors_scene is not None and len(descriptors_scene) > 2:
            # Coincidencias con FLANN
            matches = self.flann.knnMatch(self.descriptors_obj, descriptors_scene, k=2)

            # Filtrar coincidencias usando la regla de Lowe
            good_matches = []
            for match in matches:
                
                if len(match) >= 2: 
                    print("match found")
                    m,n = match
                    if m.distance < (0.75 * n.distance): 
                        good_matches.append(m)

            # Dibujar coincidencias entre la imagen de referencia y la escena
            img_matches = cv2.drawMatches(
                self.img_object, self.keypoints_obj, img_scene, keypoints_scene,
                good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
            )

            # Si hay suficientes coincidencias, estimar la homografía
            if len(good_matches) > 6:
                
                # Obtener puntos clave de la imagen de referencia y la escena
                obj_pts = np.float32([self.keypoints_obj[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                scene_pts = np.float32([keypoints_scene[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

                # Verificar que haya suficientes puntos para calcular la homografía
                if len(obj_pts) >= 4 and len(scene_pts) >= 4:
                    
                    H, _ = cv2.findHomography(obj_pts, scene_pts, cv2.RANSAC)

                    if H is not None:
                        # Definir esquinas del objeto en la imagen de referencia
                        h, w = self.img_object.shape
                        obj_corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)

                        # Transformar esquinas a la escena
                        scene_corners = cv2.perspectiveTransform(obj_corners, H)

                        # Dibujar una caja alrededor del objeto detectado en la imagen original
                        cv2.polylines(img_scene, [np.int32(scene_corners)], True, (0, 255, 0), 3)
                    else:
                        self.get_logger().warn("No hay suficientes puntos para calcular la homografía")                        
            # Mostrar la imagen con coincidencias
            cv2.imshow('Matches', img_matches)

        # Mostrar la imagen en vivo con detección del objeto
        cv2.imshow('Camera View with Detection', img_scene)

        # Cerrar con la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    # Inicializar el nodo de ROS2
    rclpy.init(args=args)
    node = ObjectDetector()

    # Mantener el nodo en ejecución
    rclpy.spin(node)

    # Cerrar todas las ventanas de OpenCV al finalizar
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
