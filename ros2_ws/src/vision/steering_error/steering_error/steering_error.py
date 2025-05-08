import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, Bool, Int32
import time

class SteeringError(Node):
    def __init__(self):
        super().__init__('steering_error')

        # Subscripciones
        self.subscription_borders = self.create_subscription(Float64MultiArray, 'lane_borders', self.listener_callback_points, 10)
        self.subscription_enable = self.create_subscription(Bool, 'enable_Auto', self.listener_enable, 10)
        self.subscription_stop = self.create_subscription(Bool, '/stop', self.listener_stop, 10)
        self.go_sub = self.create_subscription(Bool, '/go_state', self.go_state_callback, 10)
        self.overtake_sub = self.create_subscription(Bool, '/overtake_state', self.overtake_state_callback, 10)

        # Publicadores
        self.steering_publisher = self.create_publisher(Float64, 'steering', 10)
        self.speed_publisher = self.create_publisher(Int32, '/target_speed', 10)

        # Variables de estado
        self.enable = False
        self.go_state_active = False
        self.overtake_state_active = False
        self.stop_signal = False
        self.left_point = 0
        self.right_point = 0
        self.left_pointR = 50
        self.right_pointR = 113
        self.previous_error = 0.0
        self.kp = 1.0
        self.kd = 0.1
        self.current_speed = 1500

    def go_state_callback(self, msg):
        self.go_state_active = msg.data
        estado = "activo" if msg.data else "inactivo"
        self.get_logger().info(f"Estado GO {estado} (/go_state = {msg.data})")

    def overtake_state_callback(self, msg):
        self.overtake_state_active = msg.data
        estado = "activo" if msg.data else "inactivo"
        self.get_logger().info(f"Estado OVERTAKE {estado} (/overtake_state = {msg.data})")

    def listener_callback_points(self, msg):
        self.left_point = msg.data[0]
        self.right_point = msg.data[1]
        if self.enable and (self.go_state_active or self.overtake_state_active):
            self.evaluate_stop_signal()

    def listener_enable(self, msg):
        self.enable = msg.data
        if not self.enable:
            self.steering_publisher.publish(Float64(data=0.0))
            self.speed_publisher.publish(Int32(data=1500))
            self.get_logger().info("Modo automático desactivado.")

    def listener_stop(self, msg):
        self.stop_signal = msg.data

    def evaluate_stop_signal(self):
        if self.stop_signal:
            self.get_logger().info("Señal de STOP detectada. Deteniendo el coche por 5 segundos.")
            self.steering_publisher.publish(Float64(data=0.0))
            self.speed_publisher.publish(Int32(data=1500))
            time.sleep(5)
            self.get_logger().info("Reanudando movimiento después de STOP.")
        else:
            self.correction_mov()

    def correction_mov(self):
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

        kp_steering = 0.03
        steering_angle = max(min(kp_steering * -corrected_error, 0.5), -0.5)

        self.steering_publisher.publish(Float64(data=steering_angle))
        self.speed_publisher.publish(Int32(data=1562))

def main(args=None):
    rclpy.init(args=args)
    node = SteeringError()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()