import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Float64
import math
from time import sleep

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val,val))

class YDLidarClient(Node):
    def __init__(self):
        super().__init__('ydlidar_ros2_driver_client')

        # Variables de estado
        self.rebasando = False
        self.primeraFase = False
        self.segundaFase = False
        self.terceraFase = False

        # Crear publicadores
        self.driver_publisher = self.create_publisher(Int32, '/target_speed', 10)
        self.servo_publisher = self.create_publisher(Float64, 'steering', 10)

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
        # Verificar si los datos del escaneo son válidos
        if scan.scan_time == 0 or scan.time_increment == 0:
            self.get_logger().warn("Datos del LIDAR no válidos. Esperando datos correctos...")
            return

        count = int(scan.scan_time / scan.time_increment)
        #print(f"Número de medidas: {count}")
        #print(f"Medidas por grado: {count / 360:.2f}")
        #print(f"[YDLIDAR INFO]: Escaneo recibido {scan.header.frame_id}[{count}]:")
        #print(f"[YDLIDAR INFO]: Rango angular : [{math.degrees(scan.angle_min):.2f}, {math.degrees(scan.angle_max):.2f}]")


        while self.deteccionColisiones(scan, count):
            # Si hay una colisión, detener el vehículo
            self.driver_publisher.publish(Int32(data=1500))
        self.rebase(scan, count)

    def alineacion(self, scan, count):
        servo_steer = 0

        idealDistance = 0.20    # Ajustar estos dos valores para que
        steeringConstant = 5    #


        lastSmallestDistance = 10
        #print("Alineándose")
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            if 45 < degree < 95:
                if scan.ranges[i] > 0:
                    lastSmallestDistance = min(lastSmallestDistance,scan.ranges[i])
        fixedDistance = lastSmallestDistance - idealDistance
        servo_steer = -steeringConstant * fixedDistance
        print(f"Distancia: {lastSmallestDistance}")
        self.servo_publisher.publish(Float64(data=servo_steer))
        """
        for i in range(count):
            if 85 < degree < 95:
                if scan.ranges[i] <= 0.25 and scan.ranges[i] != 0:
                    servo_steer = 0.05 / scan.ranges[i]
                    self.servo_publisher.publish(Float64(data=servo_steer))
                if scan.ranges[i] > 0.25 and scan.ranges[i] < 0.30:
                    self.servo_publisher.publish(Float64(data=-0.1))
        """
                    
    def deteccionColisiones(self, scan, count):
        colision = False
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            if degree > -15 and degree < 15:
                if scan.ranges[i] < 0.25 and scan.ranges[i] != 0:
                    colision = True
                    #print("Frente")
                    
                    # Publicar valor 1500 al nodo driver
                    msg = Int32()
                    msg.data = 1500
                    self.driver_publisher.publish(msg)
                    #print("Colisión detectada, deteniendo el vehículo")
                    break
        return colision

    def detectarPosibleRebase(self, scan, count):
        objeto_detectado = False

        # Detectar objeto justo enfrente (-20 a 20 grados)
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            if -20 < degree < 20:
                if scan.ranges[i] < 0.45 and scan.ranges[i] != 0:
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
                        return False

            if not obstaculo_izquierda:
                print("No hay obstáculos a la izquierda, se puede rebasar")
                return True

        return False

    def rebase(self, scan, count):
        if self.rebasando:
            # Mover el servo a la derecha
            self.servo_publisher.publish(Float64(data=0.6))
            self.driver_publisher.publish(Int32(data=1560))  
            # Primera fase: Enderezar dirección
            if not self.primeraFase:
                if self.enderezarDireccion(scan, count):
                    self.primeraFase = True
                    print("Primera fase completada: Dirección enderezada")
                    print("Alineándose")

            # Segunda fase: Incorporarse
            if self.primeraFase and not self.segundaFase:
                if self.incorporarse(scan, count):
                    self.segundaFase = True
                    print("Segunda fase completada: Incorporación exitosa")
                else:
                    self.alineacion(scan, count)

            # Tercera fase: Finalizar rebase
            if self.segundaFase and not self.terceraFase:
                print("Tercera fase completada: Rebase finalizado")
                self.terceraFase = True
                self.rebasando = False  # Finalizar el proceso de rebase
                # Regresa a la la deteccion de carriles con la webcam
        else:
            # Detectar si es posible iniciar el rebase
            if self.detectarPosibleRebase(scan, count):
                # Reiniciar las fases si se detecta un nuevo rebase
                self.primeraFase = False
                self.segundaFase = False
                self.terceraFase = False
                self.rebasando = True

    def enderezarDireccion(self, scan, count):
        libreFrente = True
        libreCostado = True
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            if 0 < degree < 20:
                if scan.ranges[i] < 0.38 and scan.ranges[i] != 0:
                    libreFrente = False
        if libreFrente:
            for i in range(count):
                degree = math.degrees(scan.angle_min + scan.angle_increment * i)
                if 20 < degree < 40:
                    if scan.ranges[i] < 0.38 and scan.ranges[i] != 0:
                        libreCostado = False
                        print("Libre para 2do carril")
                        self.servo_publisher.publish(Float64(data=-0.5))
                        return True
        else:
            print("No libre para 2do carril")
            return False

    def incorporarse(self, scan, count):
        libre = True
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            if 25 < degree < 95:
                if scan.ranges[i] < 0.40 and scan.ranges[i] != 0:
                    libre = False
        if libre:
            print("Incorporándose")
            self.servo_publisher.publish(Float64(data=-0.6))
            sleep(1)
            self.servo_publisher.publish(Float64(data=0.5))
            sleep(0.5)
            self.servo_publisher.publish(Float64(data=0.0))
            self.driver_publisher.publish(Int32(data=1500))
            sleep(0.3)
            self.driver_publisher.publish(Int32(data=1500))
            sleep(0.3)
            self.driver_publisher.publish(Int32(data=1500))
            
            # Termina maniobra de incorporación
            return True
        else:
            return False
        """
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            if 45 < degree < 95:
                if scan.ranges[i] > 0.50: # Por ajustar
                    print("Incorporándose")
                    self.servo_publisher.publish(Float64(data=-0.6))
                    sleep(0.4)
                    self.servo_publisher.publish(Float64(data=0.5))
                    sleep(0.3)
                    self.servo_publisher.publish(Float64(data=0.0))
                    self.driver_publisher.publish(Int32(data=1500))
                    # Termina maniobra de incorporación
                    return True
        print("No se puede incorporar")
        return False
        """
    


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