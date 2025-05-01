#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

class LEDController(Node):
    def __init__(self):
        super().__init__('led_controller')
        
        # Configuración de los pines GPIO (BCM numbering)
        self.led_pins = {
            'D': [6, 19],  # LEDs para 'D'
            'I': [13, 26], # LEDs para 'I'
            'N': []        # Ningún LED para 'N'
        }
        
        GPIO.setmode(GPIO.BCM)
        
        # Configurar todos los pines como salida y apagarlos
        all_pins = set(self.led_pins['D'] + self.led_pins['I'])
        for pin in all_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        # Suscriptor al tópico 'rebase'
        self.subscription = self.create_subscription(
            String,
            'rebase',
            self.listener_callback,
            10
        )
        self.subscription  # Evita advertencia de variable no usada
        self.get_logger().info("LED Controller listo, esperando mensajes en /rebase...")

    def listener_callback(self, msg):
        print("Mensaje recibido")
        command = msg.data.strip().upper()  # Convertir a mayúsculas y eliminar espacios
        
        # Apagar todos los LEDs primero
        all_pins = set(self.led_pins['D'] + self.led_pins['I'])
        for pin in all_pins:
            GPIO.output(pin, GPIO.LOW)
        
        # Encender los LEDs según el comando
        if command in self.led_pins:
            for pin in self.led_pins[command]:
                GPIO.output(pin, GPIO.HIGH)
            self.get_logger().info(f"Comando '{command}' recibido. LEDs encendidos: {self.led_pins[command]}")
        else:
            self.get_logger().warn(f"Comando '{command}' no válido. Use 'D', 'I' o 'N'.")

    def __del__(self):
        GPIO.cleanup()  # Limpiar GPIO al cerrar

def main(args=None):
    print("Main")
    rclpy.init(args=args)
    node = LEDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
