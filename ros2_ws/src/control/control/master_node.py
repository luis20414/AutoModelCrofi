import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from enum import Enum

class Estado(Enum):
    IDLE = 0
    GO = 1
    STOP = 2
    GO_ON = 3
    OVERTAKE = 4

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')
        
        # Publicadores
        self.prueba_pub = self.create_publisher(String, '/tipo_prueba', 10)
        self.idle_pub = self.create_publisher(Bool, '/idle_state', 10)
        self.go_pub = self.create_publisher(Bool, '/go_state', 10)
        self.stop_pub = self.create_publisher(Bool, '/stop_state', 10)
        self.go_on_pub = self.create_publisher(Bool, '/go_on_state', 10)

        # Subscripciones a distintos tópicos para validar transiciones
        self.encender_sub = self.create_subscription(Bool, '/enable_Auto', self.encender_callback, 10)
        self.senal_sub = self.create_subscription(Bool, '/stop_detected', self.signal_callback, 10)
        self.finrebase_sub = self.create_subscription(Bool, '/final_rebase', self.continue_callback, 10)

        # Estado actual
        self.estado_actual = Estado.IDLE

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

        if self.estado_actual == Estado.IDLE:
            self.idle_pub.publish(msg)
        elif self.estado_actual == Estado.GO:
            self.go_pub.publish(msg)
        elif self.estado_actual == Estado.STOP:
            self.stop_pub.publish(msg)
        elif self.estado_actual == Estado.GO_ON:
            self.go_on_pub.publish(msg)

    def encender_callback(self, msg):
        if msg.data:
            self.estado_actual = Estado.GO
            self.get_logger().info('Transición a GO')
            self.publicar_estado()

    def signal_callback(self, msg):
        if msg.data:
            self.estado_actual = Estado.STOP
            self.get_logger().info('Transición a STOP')
            self.publicar_estado()

    def continue_callback(self, msg):
        if msg.data:
            self.estado_actual = Estado.GO_ON
            self.get_logger().info('Transición a GO_ON')
            self.publicar_estado()


def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()