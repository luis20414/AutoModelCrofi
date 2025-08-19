import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64, Float64MultiArray, Bool, String
import numpy as np
import time

class RebaseNode(Node):
    def __init__(self):
        super().__init__('rebase_node')

        # Variables de estado
        self.rebasando = False
        self.fase_actual = 0
        self.tiempo_cambio_carril = 0
        self.colision_detectada = False
        self.distanciaColision = 0.65

        

        # Banderas de máquina de estados
        self.B1 = False
        self.B2 = False
        self.B3 = False
        self.B4 = False

        self.idealDistance = 0.20   
        self.steeringConstant = 7  

        # Timer de velocidad
        self.startSpeed = 1547
        self.max_speed = 1553
        self.currentSpeed = self.startSpeed

        tiempoTimer = 1
        self.timer = self.create_timer(tiempoTimer, self.timer_callback)

        # Crear publicadores
        self.driver_publisher = self.create_publisher(Int32, '/target_speed', 10)
        self.servo_publisher = self.create_publisher(Float64, 'steering', 10)
        self.lights_publisher = self.create_publisher(String, '/rebase', 10)
        self.rebase_publisher = self.create_publisher(Bool, '/overtake_detected', 10)
        self.finrebase_publisher = self.create_publisher(Bool, '/end_overtake', 10)

        # Crear suscripciones
        self.degrees_subscription = self.create_subscription(
            Float64MultiArray, 'degreesLiDar', self.degrees_callback, 10)
        self.ranges_subscription = self.create_subscription(
            Float64MultiArray, 'rangesLiDar', self.ranges_callback, 10)
        self.colision_subscription = self.create_subscription(
            Bool, '/colision', self.colision_callback, 10)

        # FSM state subscriptions
        self.go_state_sub = self.create_subscription(Bool, '/go_state', self.go_callback, 10)
        self.overtake_state_sub = self.create_subscription(Bool, '/overtake_state', self.overtake_callback, 10)

        # Activación FSM
        self.go_active = False
        self.overtake_active = False
        self.active = False

        # Datos de LiDAR
        self.angles = []
        self.ranges = []

    def go_callback(self, msg):
        self.go_active = msg.data
        self.update_activation()

    def overtake_callback(self, msg):
        self.overtake_active = msg.data
        self.update_activation()

    def update_activation(self):
        self.active = self.go_active or self.overtake_active
        estado = "activo" if self.active else "inactivo"
        self.get_logger().info(f"Rebase está {estado} (go={self.go_active}, overtake={self.overtake_active})")


    def timer_callback(self):
        if self.currentSpeed <= self.max_speed:
                self.currentSpeed += 1
        else:
            self.currentSpeed = self.startSpeed


    def colision_callback(self, msg):
        self.colision_detectada = msg.data
        if self.colision_detectada:
            msg = Int32()
            msg.data = 1500
            self.driver_publisher.publish(msg)

    def degrees_callback(self, msg):
        self.angles = np.array(msg.data)

    def ranges_callback(self, msg):
        self.ranges = np.array(msg.data)
        if len(self.angles) > 0 and len(self.ranges) > 0:
            if len(self.angles) == len(self.ranges):
                if self.active:
                    self.procesar_rebase()
            else:
                self.get_logger().warn("Los tamaños de angles y ranges no coinciden. Esperando sincronización...")

    def procesar_rebase(self):
        if self.colision_detectada:
            return

        if self.rebasando:
            if self.fase_actual == 1:
                if self.verificar_izquierda():
                    self.fase_actual = 2
                    self.get_logger().info("Carril izquierdo despejado, cambiando de carril")
            elif self.fase_actual == 2:
                if self.cambiar_carril():
                    self.fase_actual = 3
                    self.get_logger().info("Cambio de carril completado, iniciando reincorporación")
            elif self.fase_actual == 3:
                if self.reincorporarse():
                    self.fase_actual = 4
                    self.get_logger().info("Reincorporación completada, regresando al carril original")
            elif self.fase_actual == 4:
                if self.regresar_carril_original():
                    self.fase_actual = 5
                    self.get_logger().info("Regreso al carril original completado")
            elif self.fase_actual == 5:
                self.finalizar_rebase()
        else:
            if self.detectar_objeto_enfrente():
                self.rebasando = True
                self.fase_actual = 1
                self.get_logger().info("Iniciando maniobra de rebase")

    def detectar_objeto_enfrente(self):
        indices_frente = (self.angles > -15) & (self.angles < 15)
        if np.any((self.ranges[indices_frente] < self.distanciaColision) & (self.ranges[indices_frente] != 0)):
            self.get_logger().info("Objeto detectado enfrente, iniciando rebase")
            self.rebase_publisher.publish(Bool(data=True))
            return True
        return False

    def verificar_izquierda(self):
        indices_izquierda = (self.angles > 35) & (self.angles < 50)
        if np.any((self.ranges[indices_izquierda] < 0.35) & (self.ranges[indices_izquierda] != 0)):
            self.get_logger().info("Obstáculo detectado a la izquierda, deteniendo vehículo")
            self.driver_publisher.publish(Int32(data=1500))
            return False
        else:
            self.driver_publisher.publish(Int32(data=self.currentSpeed))
            return True

    def cambiar_carril(self):
        self.servo_publisher.publish(Float64(data=0.5))
        start_time = time.time()
        indices_cambio1 = (self.angles > -45) & (self.angles < 0)
        if np.all((self.ranges[indices_cambio1] > 0.65) | (self.ranges[indices_cambio1] == 0)):
            self.get_logger().info("Avanzando en el carril izquierdo")
            self.tiempo_cambio_carril = time.time() - start_time
            self.servo_publisher.publish(Float64(data=-0.5))
            self.driver_publisher.publish(Int32(data=self.currentSpeed))
            time.sleep(0.8)
            return True
        return False

    def reincorporarse(self):
        indices_derecha = (self.angles > -90) & (self.angles < -30)
        if np.any((self.ranges[indices_derecha] < 0.60) & (self.ranges[indices_derecha] != 0)):
            self.alineacion()
            return False
        else:
            return True

    def alineacion(self):
        indices_alineacion_costado = (self.angles > -95) & (self.angles < -45)
        indices_alineacion_esquina = (self.angles > -45) & (self.angles < -30)

        distancias_costado = self.ranges[indices_alineacion_costado & (self.ranges > 0)]
        distancias_esquina = self.ranges[indices_alineacion_esquina & (self.ranges > 0)]

        if len(distancias_costado) > 0 and len(distancias_esquina) > 0:
            d_costado = np.min(distancias_costado)
            d_esquina = np.min(distancias_esquina)

            if d_costado > (0.6 * d_esquina):
                ideal = 2.5 * self.idealDistance
                error = d_esquina - ideal
                self.get_logger().info(f"DISTANCIA: {d_esquina}")
            else:
                ideal = self.idealDistance
                error = d_costado - ideal
                self.get_logger().info(f"Distancia: {d_costado}")

            steer = -self.steeringConstant * error
            steer = self.constrain(steer, -0.5, 0.5)
            self.servo_publisher.publish(Float64(data=steer))
        else:
            self.servo_publisher.publish(Float64(data=0.0))

    def regresar_carril_original(self):
        self.servo_publisher.publish(Float64(data=-0.5))
        time.sleep(0.5)
        self.servo_publisher.publish(Float64(data=0.5))
        time.sleep(0.35)
        self.servo_publisher.publish(Float64(data=0.0))
        return True

    def finalizar_rebase(self):
        self.get_logger().info("Rebase completado, regresando al carril original")
        self.rebasando = False
        self.fase_actual = 0
        self.servo_publisher.publish(Float64(data=0.0))
        self.driver_publisher.publish(Int32(data=1500))
        self.finrebase_publisher.publish(Bool(data=True))
        time.sleep(3)

    def constrain(self, val, min_val, max_val):
        return min(max_val, max(min_val, val))

def main(args=None):
    rclpy.init(args=args)
    node = RebaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()