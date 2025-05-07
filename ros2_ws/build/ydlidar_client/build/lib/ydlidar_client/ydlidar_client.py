import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Float64
import math




def deteccionColisiones(scan, count, driver_publisher):
    colision = False

    for i in range(count):
        degree = math.degrees(scan.angle_min + scan.angle_increment * i)
        if degree > -35 and degree < 35:
            if scan.ranges[i] < 0.35 and scan.ranges[i] != 0:
                colision = True
                print("Frente")
                
                # Publicar valor 1500 al nodo driver
                msg = Int32()
                msg.data = 1500
                driver_publisher.publish(msg)
                break
        elif degree > 85 and degree < 95:
            if scan.ranges[i] < 0.25 and scan.ranges[i] != 0:
                colision = True
                print("Derecha")
                break
        elif degree > -95 and degree < -85:
            if scan.ranges[i] < 0.25 and scan.ranges[i] != 0:
                colision = True
                print("Izquierda")
                break

    if not colision:
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            if -10 < degree < 10:
                print(f"[YDLIDAR INFO]: Angulo-distancia : [{degree:.2f}, {scan.ranges[i]:.2f}]")
    else:
        print("Te fuiste de hocico")
    
    return colision


def detectarPosibleRebase(scan, count):
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
                    rebasando = False

        if not obstaculo_izquierda:
            print("No hay obstáculos a la izquierda, se puede rebasar")
            rebasando = True
    else:
        rebasando = False

def enderezarDireccion(scan, count):
    for i in range(count):
        degree = math.degrees(scan.angle_min + scan.angle_increment * i)
        if 25 < degree < 30:
            if scan.ranges[i] > 0.30:
                print("Enderezando direccion")
                return True
    return False

def incorporarse(scan, count):
    for i in range(count):
        degree = math.degrees(scan.angle_min + scan.angle_increment * i)
        if 85 < degree < 95:
            if scan.ranges[i] > 0.25:
                print("Incorporandose")
                return True
    print("No se puede incorporar")
    return False

def alinearse(scan, count):
    for i in range(count):
        degree = math.degrees(scan.angle_min + scan.angle_increment * i)
        if 85 < degree < 95:
            if scan.ranges[i] < 0.17 and scan.ranges[i] != 0:
                print("Alineandose")

def rebase(scan, count):
    if rebasando:
        # Mover el servo a la derecha
        if not primeraFase:
            if enderezarDireccion(scan, count):
                primeraFase = True
                # Mover el servo a la derecha
        if not segundaFase and primeraFase:
            if incorporarse(scan, count):
                segundaFase = True
                # Mover el servo a la derecha
            else:
                alinearse(scan, count)
        if not terceraFase and segundaFase:
            print("Incor")
            terceraFase = True
            # Maniobra de incorporacion
    else:
        detectarPosibleRebase(scan, count)



def scan_callback(scan, driver_publisher, servo_publisher):
    colision = False
    count = int(scan.scan_time / scan.time_increment)
    print(f"[YDLIDAR INFO]: Escaneo recibido {scan.header.frame_id}[{count}]:")
    print(f"[YDLIDAR INFO]: Rango angular : [{math.degrees(scan.angle_min):.2f}, {math.degrees(scan.angle_max):.2f}]")

    deteccionColisiones(scan, count, driver_publisher)
    
    #rebase(scan, count)



def main(args=None):
    rclpy.init(args=args)
    node = Node("ydlidar_ros2_driver_client")
    
    global rebasando
    global primeraFase
    global segundaFase
    global terceraFase
    
    rebasando = False
    primeraFase = False
    segundaFase = False
    terceraFase = False

    # Crear publicadores
    driver_publisher = node.create_publisher(Int32, '/detection_value', 10)
    servo_publisher = node.create_publisher(Float64, 'angle_servo', 10)

    qos_profile = qos.qos_profile_sensor_data
    node.create_subscription(
        LaserScan,
        'scan',
        lambda scan: scan_callback(scan, driver_publisher, servo_publisher),
        qos_profile
    )
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()