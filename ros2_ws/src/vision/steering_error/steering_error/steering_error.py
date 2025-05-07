import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, Bool, Int32
import time

class SteeringError(Node):
    def __init__(self):
        super().__init__('steering_error')
        self.subscription_borders = self.create_subscription(Float64MultiArray, 'lane_borders', self.listener_callback_points, 10)
        self.steering_publisher = self.create_publisher(Float64, 'steering', 10)
        self.speed_publisher = self.create_publisher(Int32, '/target_speed', 10)
        self.subscription_enable = self.create_subscription(Bool, 'enable_Auto', self.listener_enable, 10)
        self.subscription_stop = self.create_subscription(Bool, '/stop', self.listener_stop, 10)

        self.go_sub = self.create_subscription(Bool, '/go_state', self.avanzar_callback, 10)
        self.go_on_sub = self.create_subscription(Bool, '/go_on_state', self.continuar_callback, 10)

        # Variables de estado
        self.allow_processing = False  # <-- Bandera de control agregada
        self.enable = False
        self.stop_signal = False
        self.stop_detected = False
        self.left_point = 0
        self.right_point = 0
        self.left_pointR = 50
        self.right_pointR = 113
        self.previous_error = 0.0
        self.kp = 1.0
        self.kd = 0.1
        self.max_speed = 1585
        self.current_speed = 1500

    def avanzar_callback(self, msg):
        if msg.data:
            self.allow_processing = True
            self.get_logger().info("Procesamiento activado por /go_state")

    def continuar_callback(self, msg):
        if msg.data:
            self.allow_processing = True
            self.get_logger().info("Procesamiento activado por /go_on_state")

    def listener_callback_points(self, msg):
        self.left_point = msg.data[0]
        self.right_point = msg.data[1]

    def listener_enable(self, msg):
        self.enable = msg.data
        if not self.enable:
            self.steering_publisher.publish(Float64(data=0.0))
            self.speed_publisher.publish(Int32(data=1500))
            self.get_logger().info("Modo automático desactivado.")

    def listener_stop(self, msg):
        self.stop_signal = msg.data
        if self.enable and self.allow_processing:
            self.evaluate_stop_signal()

    def evaluate_stop_signal(self):
        if self.stop_signal:
            self.get_logger().info("Señal de STOP detectada. Deteniendo el coche por 5 segundos.")
            self.steering_publisher.publish(Float64(data=0.0))
            self.speed_publisher.publish(Int32(data=1500))
            time.sleep(5)
            self.get_logger().info("Reanudando movimiento después de STOP.")
            self.stop_detected = True
        else:
            self.correction_mov()

    def correction_mov(self):
        if not self.enable or not self.allow_processing:
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