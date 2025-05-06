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

        # Variables de estado
        self.enable = False
        self.stop_signal = False
        self.stop_detected = False  # Nueva bandera para rastrear si ya se procesó un True en /stop
        self.left_point = 0
        self.right_point = 0
        self.left_pointR = 45  # Referencia para el punto izquierdo, 50
        self.right_pointR = 116  # Referencia para el punto derecho, 113
        #140 para la interseccion
        self.previous_error = 0.0
        self.kp = 1.0  # Constante proporcional
        self.kd = 0.1  # Constante derivativa
        self.max_speed = 1585
        self.current_speed = 1500  # Velocidad actual del vehículo

    def listener_callback_points(self, msg):
        # Actualizar los puntos de los bordes del carril
        self.left_point = msg.data[0]
        self.right_point = msg.data[1]

    def listener_enable(self, msg):
        # Activar o desactivar el modo automático
        self.enable = msg.data
        if not self.enable:
            # Desactivar movimiento
            self.steering_publisher.publish(Float64(data=0.0))
            self.speed_publisher.publish(Int32(data=1500))
            self.get_logger().info("Modo automático desactivado.")

    def listener_stop(self, msg):
        # Actualizar el estado de la señal de stop
        self.stop_signal = msg.data
        if self.enable:
            if not self.stop_detected:  # Solo procesar si no se ha detectado antes
                self.evaluate_stop_signal()
            elif not self.stop_signal:  # Si ya se procesó y la señal es False, continuar movimiento
                self.correction_mov()

    def evaluate_stop_signal(self):
        if self.stop_signal:
            # Detener el coche por 5 segundos
            self.get_logger().info("Señal de STOP detectada. Deteniendo el coche por 5 segundos.")
            self.steering_publisher.publish(Float64(data=0.0))
            self.speed_publisher.publish(Int32(data=1500))
            time.sleep(5)  # Pausa de 5 segundos
            self.get_logger().info("Reanudando movimiento después de STOP.")
            self.stop_detected = True  # Marcar que ya se procesó el STOP
        else:
            # Continuar con el movimiento normal
            self.correction_mov()

    def correction_mov(self):
        # Calcular el error de dirección y ajustar el movimiento
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

        # Aplicar el controlador PD al error
        derivative = error - self.previous_error
        corrected_error = self.kp * error + self.kd * derivative
        self.previous_error = error

        # Calcular el ángulo de dirección
        kp_steering = 0.03  # Constante para ajustar el rango del ángulo
        steering_angle = max(min(kp_steering * -corrected_error, 0.5), -0.5)

        # Publicar el ángulo de dirección
        self.steering_publisher.publish(Float64(data=steering_angle))

        # Calcular y publicar la velocidad
        target_speed = max(self.max_speed - abs(corrected_error) * 3, 1560)  # Reducir velocidad con error

        # Si la velocidad actual es mayor a 1670 y se necesita reducir
        if self.current_speed > 1670 and target_speed < self.current_speed:
            self.get_logger().info("Reduciendo velocidad: pasando por 1500 antes de establecer 1600.")
            self.speed_publisher.publish(Int32(data=1500))  # Detener momentáneamente
            time.sleep(0.5)  # Pausa breve para simular la transición
            self.speed_publisher.publish(Int32(data=1600))  # Establecer velocidad a 1600
            self.current_speed = 1600
        else:
            # Publicar la velocidad calculada normalmente
            self.speed_publisher.publish(Int32(data=int(target_speed)))
            self.current_speed = target_speed


def main(args=None):
    rclpy.init(args=args)
    node = SteeringError()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()