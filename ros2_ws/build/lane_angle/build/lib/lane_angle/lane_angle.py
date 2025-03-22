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

    def listener_callback_lines(self, msg):
        self.linesP = np.array(msg.data).reshape(-1, 1, 4)

    def listener_callback_image(self, msg):
        self.get_logger().info("Received image: %dx%d" % (msg.width, msg.height))
        frame = bridge.imgmsg_to_cv2(msg)
        deviation = self.Angle_Calc(frame)
        self.publish_angle(deviation)

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
#esto segun yo esta mal, nunca esta calculando el angulo respecto al centro de la imagen 
        if left_lines and right_lines: #si hay lineas a la derecha y a la izquierda
            left_x = np.mean([x1 for x1, _, _, _ in left_lines]) #obtiene el punto inicial en x de la linea izq.
            right_x = np.mean([x1 for x1, _, _, _ in right_lines])#obtiene el punto inical en x de la linea derecha
            lane_center = (left_x + right_x) / 2 #obtiene el centro entre las dos lineas 
            deviation = center_x - lane_center #obtiene la diferencia entre el centro de la imagen y el centro de las lineas 
            return self.Angle_Rad(-deviation) #Envia la -diferencia entre el centro de la imagen y el de las lineas

        return 0.0
    
    def Angle_Rad(self, deviation):
        deviation= deviation * np.pi/180#multiplica deviation por pi/180
        if deviation <= -0.5: #si deviation es menor o igual a -0.5 mandara al codigo servo.py -0.5
            return 0.5
        elif deviation >= 0.5: #si deviation es mayor o igual a 0.5 mandara al codigo servo.py 0.5
            return -0.5
        elif deviation == 0.5: #si deviation es igual a 0.5 regresa 0???? Eso pa que?
            return 0
        else: return round(deviation, 3) #si esta [-0.5,0.5] manda el "angulo" redondeado a 3 decimales
            
    
    
    def publish_angle(self, deviation):
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
