import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from enum import Enum

class Estado(Enum):
    IDLE = 0
    GO = 1
    OVERTAKE = 2

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')
        
        # Publicadores
        self.prueba_pub = self.create_publisher(String, '/tipo_prueba', 10)
        self.idle_pub = self.create_publisher(Bool, '/idle_state', 10)
        self.go_pub = self.create_publisher(Bool, '/go_state', 10)
        self.overtake_pub = self.create_publisher(Bool, '/overtake_state', 10)

        # Subscripciones a distintos tópicos para validar transiciones
        self.encender_sub = self.create_subscription(Bool, '/enable_Auto', self.encender_callback, 10)
        self.rebase_sub = self.create_subscription(Bool, '/overtake_detected', self.rebase_callback, 10)
        self.final_rebase_sub = self.create_subscription(Bool, '/end_overtake', self.fin_rebase_callback, 10)

        # Estado actual
        self.estado_actual = Estado.IDLE
        self.publicar_estado()  # Publica el estado inicial

        # Lista de pruebas
        """self.nom_prueba = ['prueba1', 'prueba23']
        self.prueba_index = 0

    def cambio_prueba(self):
        prueba = self.nom_prueba[self.prueba_index]
        self.prueba_pub.publish(String(data=prueba))
        self.prueba_index = (self.prueba_index + 1) % len(self.nom_prueba)"""

    def publicar_estado(self):
        msg = Bool()
        msg.data = True
        self.get_logger().info(f"Publicando estado actual: {self.estado_actual.name}")

        self.idle_pub.publish(Bool(data=self.estado_actual == Estado.IDLE))
        self.go_pub.publish(Bool(data=self.estado_actual == Estado.GO))
        self.overtake_pub.publish(Bool(data=self.estado_actual == Estado.OVERTAKE))

    def encender_callback(self, msg):
        if msg.data and self.estado_actual == Estado.IDLE:
            self.estado_actual = Estado.GO
            self.get_logger().info('Transición IDLE → GO')
            self.publicar_estado()

    def rebase_callback(self, msg):
        if msg.data and self.estado_actual == Estado.GO:
            self.estado_actual = Estado.OVERTAKE
            self.get_logger().info('Transición GO → OVERTAKE')
            self.publicar_estado()

    def fin_rebase_callback(self, msg):
        if msg.data and self.estado_actual == Estado.OVERTAKE:
            self.estado_actual = Estado.GO
            self.get_logger().info('Transición OVERTAKE → GO')
            self.publicar_estado()

def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
