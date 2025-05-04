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
        self.max_speed = 1585
        self.subscription_enable = self.create_subscription(Bool, 'enable_Auto', self.listener_enable, 10)
        self.enable = 0
        self.left_point = 0
        self.right_point = 0
        self.left_pointR = 50
        self.right_pointR = 113
        self.previous_error = 0.0
        self.kp = 1.0  # Constante proporcional
        self.kd = 0.1  # Constante derivativa

    def listener_callback_points(self, msg):
        self.left_point = msg.data[0]
        self.right_point = msg.data[1]

    def listener_enable(self, msg):
        print("In listener_enable")
        self.enable = msg.data 
        if self.enable:
            self.correction_mov()
        else:
            # Desactivar movimiento
            self.steering_publisher.publish(Float64(data=0.0))
            self.speed_publisher.publish(Int32(data=1500))

    def correction_mov(self):
        print("In correction_mov")
        error = 0.0
        if self.left_point > 0 and self.right_point > 0:
            # Ambos puntos válidos, calcular error promedio
            error_left = self.left_point - self.left_pointR
            error_right = self.right_point - self.right_pointR
            error = (error_left + error_right) / 2
        elif self.left_point > 0:
            # Solo punto izquierdo válido
            error = self.left_point - self.left_pointR
        elif self.right_point > 0:
            # Solo punto derecho válido
            error = self.right_point - self.right_pointR
        else:
            self.get_logger().warn("No se recibieron puntos válidos.")
            return

        print(f"Raw Error: {error}")

        # Aplicar el controlador PD al error
        derivative = error - self.previous_error
        corrected_error = self.kp * error + self.kd * derivative
        self.previous_error = error

        print(f"Corrected Error (PD): {corrected_error}")

        # Determinar el ángulo de dirección basado en el error corregido
        # Si el error es positivo, steering_angle será positivo (girar a la derecha)
        # Si el error es negativo, steering_angle será negativo (girar a la izquierda)
        kp_steering = 0.03  # Constante para ajustar el rango del ángulo
        steering_angle = max(min(kp_steering * -corrected_error, 0.5), -0.5)
        print(f"Steering angle: {steering_angle}")

        # Publicar el ángulo de dirección
        self.steering_publisher.publish(Float64(data=steering_angle))

        # Publicar la velocidad (puedes ajustar esta lógica según el error corregido)
        speed = max(self.max_speed - abs(corrected_error) * 3, 1560)  # Reducir velocidad con error
        print(f"Speed: {speed}")
        self.speed_publisher.publish(Int32(data=int(speed)))

        
def main(args=None):
    print("Hi there")
    rclpy.init(args=args)
    node = SteeringError()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
