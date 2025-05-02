import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, Bool, Int32

class SteeringError(Node):
    print("In steering error")
    def __init__(self):
        super().__init__('steering_error')
        self.subscription_borders = self.create_subscription(Float64MultiArray, 'lane_borders', self.listener_callback_points, 10)
        self.steering_publisher = self.create_publisher(Float64, 'steering', 10)
        self.speed_publisher = self.create_publisher(Int32, '/target_speed', 10)
        self.max_speed = 1570
        self.subscription_enable = self.create_subscription(Bool, 'enable_Auto', self.listener_enable, 10)
        self.enable = 0
        self.left_point = 64
        self.right_point = 105
        self.previous_error = 0.0
        self.kp = 1.0  # Constante proporcional
        self.kd = 0.1  # Constante derivativa

    def listener_callback_points(self, msg):
        self.left_point = msg.data[0]
        self.right_point = msg.data[1]

    def listener_enable(self, msg):
        print("In listener_enable")
        self.enable = msg.data
        self.correction_mov()

    def correction_mov(self):
        print("In correction_mov")
        if self.enable:
            error = 0.0
            if self.left_point > 0 and self.right_point > 0:
                # Ambos puntos válidos, calcular error promedio
                error_left = self.left_point - 0  # Suponiendo que 0 es el valor de referencia
                error_right = self.right_point - 0
                error = (error_left + error_right) / 2
            elif self.left_point > 0:
                # Solo punto izquierdo válido
                error = self.left_point - 0
            elif self.right_point > 0:
                # Solo punto derecho válido
                error = self.right_point - 0
            else:
                self.get_logger().warn("No se recibieron puntos válidos.")
                return

            # Si el error es 0, mantener velocidad máxima y dirección recta
            if error <= 0.01:
                self.steering_publisher.publish(Float64(data=0.0))
                self.speed_publisher.publish(Int32(data=self.max_speed))
                return

            # Controlador PD
            derivative = error - self.previous_error
            steering = self.kp * error + self.kd * derivative
            self.previous_error = error

            # Escalar el ángulo de dirección al rango permitido (-0.5 a 0.5 rad)
            kp_steering = 0.005  # Constante para ajustar el rango del ángulo
            steering_angle = max(min(kp_steering * steering, 0.5), -0.5)
            # Publicar el ángulo de dirección
            self.steering_publisher.publish(Float64(data=steering_angle))

            # Publicar la velocidad (puedes ajustar esta lógica según el error)
            speed = max(self.max_speed - abs(error) * 10, 1530)  # Reducir velocidad con error
            print(speed)
            self.speed_publisher.publish(Int32(data=speed))

        else:
            # Desactivar movimiento
            self.steering_publisher.publish(Float64(data=0.0))
            self.speed_publisher.publish(Int32(data=1500))

def main(args=None):
    print("Hi there")
    rclpy.init(args=args)
    node = SteeringError()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


