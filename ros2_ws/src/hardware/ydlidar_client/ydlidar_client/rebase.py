import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Float64
import math


class YDLidarClient(Node):
    def __init__(self):
        super().__init__('ydlidar_ros2_driver_client')

        # Variables de estado
        self.rebasando = False
        self.primeraFase = False
        self.segundaFase = False
        self.terceraFase = False

        # Crear publicadores
        self.driver_publisher = self.create_publisher(Int32, '/detection_value', 10)
        self.servo_publisher = self.create_publisher(Float64, 'angle_servo', 10)

        # Crear perfil QoS
        self.qos_profile = qos.qos_profile_sensor_data

        # Crear suscripción
        self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            self.qos_profile
        )

    def scan_callback(self, scan):
        count = int(scan.scan_time / scan.time_increment)
        #print(f"[YDLIDAR INFO]: Escaneo recibido {scan.header.frame_id}[{count}]:")
        #print(f"[YDLIDAR INFO]: Rango angular : [{math.degrees(scan.angle_min):.2f}, {math.degrees(scan.angle_max):.2f}]")


        self.deteccionColisiones(scan, count)
        self.rebase(scan, count)

    def alineacion(self, scan, count):
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            if 85 < degree < 95:
                if scan.ranges[i] < 0.22 and scan.ranges[i] != 0:
                    self.servo_publisher.publish(Float64(data=0.2))
                if scan.ranges[i] > 0.22:
                    self.servo_publisher.publish(Float64(data=-0.2))

    def deteccionColisiones(self, scan, count):
        colision = False
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            if degree > -15 and degree < 15:
                if scan.ranges[i] < 0.35 and scan.ranges[i] != 0:
                    colision = True
                    print("Frente")
                    
                    # Publicar valor 1500 al nodo driver
                    msg = Int32()
                    msg.data = 1500
                    self.driver_publisher.publish(msg)
                    break
        return colision

    def detectarPosibleRebase(self, scan, count):
        objeto_detectado = False

        # Detectar objeto justo enfrente (-20 a 20 grados)
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            if -20 < degree < 20:
                if scan.ranges[i] < 0.35 and scan.ranges[i] != 0:
                    objeto_detectado = True
                    print("Objeto detectado justo enfrente")
                    break

        # Si se detectó un objeto enfrente, buscar obstáculos a la izquierda (-50 a -35 grados)
        if objeto_detectado:
            obstaculo_izquierda = False
            for i in range(count):
                degree = math.degrees(scan.angle_min + scan.angle_increment * i)
                if -50 < degree < -35:
                    if scan.ranges[i] < 0.35 and scan.ranges[i] != 0:
                        obstaculo_izquierda = True
                        print("Obstáculo detectado a la izquierda, no se puede rebasar")
                        self.rebasando = False

            if not obstaculo_izquierda:
                print("No hay obstáculos a la izquierda, se puede rebasar")
                self.rebasando = True
        else:
            self.rebasando = False

    def rebase(self, scan, count):
        if self.rebasando:
            # Mover el servo a la derecha
            self.servo_publisher.publish(Float64(data=0.5))
            if not self.primeraFase:
                if self.enderezarDireccion(scan, count):
                    self.primeraFase = True
            if not self.segundaFase and self.primeraFase:
                if self.incorporarse(scan, count):
                    self.segundaFase = True
                else:
                    self.alinearse(scan, count)
            if not self.terceraFase and self.segundaFase:
                print("Incorporándose")
                self.primeraFase = False
                self.segundaFaseFase = False
                self.terceraFase = False
                self.rebasando = False
        else:
            self.detectarPosibleRebase(scan, count)

    def enderezarDireccion(self, scan, count):
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            if 0 < degree < 30:
                if scan.ranges[i] > 0.30:
                    print("Enderezando dirección")
                    self.servo_publisher.publish(Float64(data=0.0))
                    return True
        return False

    def incorporarse(self, scan, count):
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            if 85 < degree < 95:
                if scan.ranges[i] > 0.25:
                    print("Incorporándose")
                    return True
        print("No se puede incorporar")
        return False
    


    def alinearse(self, scan, count):
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            if 85 < degree < 95:
                if scan.ranges[i] < 0.22 and scan.ranges[i] != 0:
                    self.alineacion(scan, count) # Se alinea en el segundo carril mientras está rebasando
                    print("Alineándose")



def main(args=None):
    rclpy.init(args=args)
    node = YDLidarClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()