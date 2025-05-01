import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from cv_bridge import CvBridge

bridge = CvBridge()

class SpeedController(Node):
    print("Entrando a SpeedController")
    def __init__(self):
        print("En init")
        super().__init__('speed_controller')
        self.subscription = self.create_subscription(Float64, 'curvature_radius', self.speed_callback, 10)
        print("Creando subscription a curvature_radius")
        self.signal_subscription = self.create_subscription(Int32, '/stop_detected', self.signal_callback, 10)
        print("Creando subscription a stop_detected")
        self.publisher_ = self.create_publisher(Float64, '/target_speed', 10)
        self.max_speed = 1570
        self.stop_signal_active = False  # Bandera para señal de alto

    def signal_callback(self, msg):
        print("Señal de alto")
        # Verificar si la señal de alto está activa
        if msg.data == 1500:
            self.stop_signal_active = True
        else:
            self.stop_signal_active = False
    print("Antes de la función speed_callback")
    def speed_callback(self, msg):
        print("Velocidad")
        radius = msg.data
        speed = Float64()
        
        # No enviar velocidad si la señal de alto está activa
        if self.stop_signal_active:
            speed.data = 1500
            self.publisher_.publish(speed)
            return
        elif radius < 2.5 and radius > 0:
            speed.data = self.max_speed * (radius / 2.5)
        else:
            speed.data = self.max_speed
        print(self.stop_signal_active)
        print(speed.data)
        self.publisher_.publish(speed)

def main(args=None):
    print("Hola")
    rclpy.init(args=args)
    driver_speed = SpeedController()
    rclpy.spin(driver_speed)
    driver_speed.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()