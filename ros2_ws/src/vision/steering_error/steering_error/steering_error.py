import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, Bool, Int32
import time

class SteeringError(Node):
    def __init__(self):
        super().__init__('steering_error')
        
        self.steering_publisher = self.create_publisher(Float64, 'steering', 10)
        self.speed_publisher = self.create_publisher(Int32, '/target_speed', 10)
        self.continue_publisher = self.create_publisher(Bool, '/end_stop', 10)
        self.subscription_enable = self.create_subscription(Bool, 'enable_Auto', self.listener_enable, 10)
        #self.subscription_stop = self.create_subscription(Bool, '/stop', self.listener_stop, 10)
        #self.subscription_stop = self.create_subscription(Bool, '/stop', self.listener_stop_callback, 10)

        self.go_sub = self.create_subscription(Bool, '/go_state', self.avanzar_callback, 10)

        self.selectorVelocidades = True # True para el inicial, False para el segundo

        self.startSpeed1 = 1540

        self.startSpeed = 1548
        self.max_speed = 1553



        self.max_speed2 = 1610 # test
        self.currentSpeed1 = self.startSpeed1
        self.currentSpeed = self.startSpeed
        #timerT = 1
        #timerT = 0.1
        timerTInicial = 1
        self.timerInicial = self.create_timer(timerTInicial, self.timerInicial_callback)
        #self.timer = self.create_timer(timerT, self.timer_callback)
        
        # Variables de estado
        self.allow_processing = False  # <-- Bandera de control agregada
        self.enable = False
        self.stop_signal = False
        self.stop_detected = False
        self.left_point = 0
        self.right_point = 0
        self.left_pointR = 45
        self.right_pointR = 116
        self.previous_error = 0.0
        self.kp = 1.0
        self.kd = 0.1
        self.current_speed = 1500
        self.contador = 0



        self.subscription_borders = self.create_subscription(Float64MultiArray, 'lane_borders', self.listener_callback_points, 10)
        #self.get_logger().info("FIN INIT AAAAAAAAAAAAAAAAAAAAAAaa")
    
    def timerInicial_callback(self):
        if self.selectorVelocidades:
            if self.currentSpeed <= self.max_speed:
                self.currentSpeed += 1
            else:
                #self.selectorVelocidades = False
                self.currentSpeed = self.startSpeed
    """
    def timer_callback(self): 
        if not self.selectorVelocidades:
            if self.contador < 5:
                self.contador += 1
                self.currentSpeed = self.startSpeed1
            else:
                self.contador = 0
                self.currentSpeed = self.max_speed2
    """



    def avanzar_callback(self, msg):
        if msg.data:
            self.allow_processing = True
            self.get_logger().info("Procesamiento activado por /go_state")
        else:
            self.allow_processing = False
            self.get_logger().info("Procesamiento Desactivado por /go_state")


    def listener_callback_points(self, msg):
        self.left_point = msg.data[0]
        self.right_point = msg.data[1]
        self.get_logger().info("Puntos recibidos")
        self.correction_mov()

    def listener_enable(self, msg):
        self.enable = msg.data
        if not self.enable:
            self.steering_publisher.publish(Float64(data=0.0))
            self.speed_publisher.publish(Int32(data=1500))
            self.get_logger().info("Modo automático desactivado.")

    def correction_mov(self):
        if not self.enable or not self.allow_processing:
            #self.get_logger().info("No entró")
            return
        
        error = 0.0
        if self.left_point > 0 and self.right_point > 0:
            error_left = self.left_point - self.left_pointR
            error_right = self.right_point - self.right_pointR
            error = (error_left + error_right) / 2
        elif self.left_point > 0:
            error = self.left_point - self.left_pointR
        elif self.right_point > 0:
            error = self.right_point - self.right_pointR
        else:
            self.get_logger().warn("No se recibieron puntos válidos.")
            return

        derivative = error - self.previous_error
        corrected_error = self.kp * error + self.kd * derivative
        self.previous_error = error

        #kp_steering = 0.035
        kp_steering = 0.03
        steering_angle = max(min(kp_steering * -corrected_error, 0.5), -0.5)
        self.steering_publisher.publish(Float64(data=steering_angle))
        self.speed_publisher.publish(Int32(data=self.currentSpeed))
        self.get_logger().info("Publicacion de puntos")
        


def main(args=None):
    rclpy.init(args=args)
    node = SteeringError()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
