import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

class LaneAngle(Node):
    def __init__(self):  # hay que enviar un dato entero de 16bits y recibir un arreglo de datos (las lineas)
        super().__init__('lane_angle')
        self.subscription_lines = self.create_subscription(Float64MultiArray, 'lane_deviation', self.listener_callback_lines, 1)
        self.subscription_image = self.create_subscription(Image, 'camera_recorted', self.listener_callback_image, 1)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.publish_angle)
        self.publisher_ = self.create_publisher(Float64, 'angle_servo', 10)
        self.linesP = None
        self.last_frame = None

    def listener_callback_lines(self, msg):
        self.linesP = np.array(msg.data).reshape(-1, 1, 4)

    def listener_callback_image(self, msg):
        #self.get_logger().info("Received image: %dx%d" % (msg.width, msg.height))
        self.last_frame = bridge.imgmsg_to_cv2(msg)

    def Angle_Calc(self, frame):
        # Calcular el centro de la imagen
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2

        left_lines = []
        right_lines = []

        if self.linesP is not None:
            for line in self.linesP:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else 0

                if slope < 0:
                    left_lines.append((x1, y1, x2, y2))
                else:
                    right_lines.append((x1, y1, x2, y2))
        #print(f"left_lines: {len(left_lines)}") #cuantas lineas en cada direccion detecto 
        #print(f"right lines: {len(right_lines)}")            
        if left_lines and right_lines:  # Si hay líneas a la derecha y a la izquierda
            left_x = np.mean([x1 for x1, _, _, _ in left_lines])  # Punto inicial en x de la primera línea izquierda que detecto 
            right_x = np.mean([x1 for x1, _, _, _ in right_lines])  # Punto inicial en x de la primera línea derecha que detecto 
            print(f"left_x: {left_x}")
            print(f"right_x: {right_x}")
            lane_center = (left_x + right_x) / 2  # Centro entre las dos líneas en el eje x
            print(f"lane_center: {lane_center}")
            deviation_x = center_x - lane_center  # Diferencia entre el centro de la imagen y el centro de las líneas
            print(f"deviation_x: {deviation_x}")
            deviation_y = height - center_y  # Diferencia entre el centro de la imagen y la altura del frame
            print(f"deviation_y: {deviation_y}")
            return self.Angle_Rad(-deviation_x, deviation_y)  # Enviar -diferencia en x y diferencia en y

        return 0.0

    def Angle_Rad(self, Dx, Dy):
        angle = np.round(np.arctan2(Dy, Dx)-(np.pi/2), 3)  # Corrige el uso de arctan2
        print(f"angle: {angle}")
        return np.clip(angle, -0.5, 0.5)  # Delimita el ángulo obtenido a -0.5 y 0.5

    def publish_angle(self):
        if self.last_frame is not None:  # Verificar si hay un frame disponible
            deviation = self.Angle_Calc(self.last_frame)  # Calcular el ángulo usando el último frame
        else:
            deviation = 0.0  # Si no hay frame, publicar 0.0
        print(deviation)
        msg = Float64()
        msg.data = float(deviation)
        self.publisher_.publish(msg)

def main(args=None):
    print('Hi from lane_angle.')
    rclpy.init(args=args)
    lane_angle = LaneAngle()
    rclpy.spin(lane_angle)

    lane_angle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
