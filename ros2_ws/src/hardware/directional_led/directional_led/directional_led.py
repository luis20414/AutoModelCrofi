#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO 
import time

class LEDController(Node):
    def __init__(self):
        super().__init__('led_controller')
        
        # Configuración de los pines GPIO (BCM numbering)
        self.led_pins = {
            'D': [6, 26],  # LEDs para 'D'
            'I': [13, 19], # LEDs para 'I'
            'N': [],       # Ningún LED para 'N'
            'T': [6, 13, 19, 26]   # Todos los LEDs 'T'
        }
        
        GPIO.setmode(GPIO.BCM)
        
        # Configurar todos los pines como salida y apagarlos
        all_pins = set(self.led_pins['D'] + self.led_pins['I'] + self.led_pins['T'])
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

    def blink_leds(self, pins, blink_count=1, delay=0.5):
        """
        Hace parpadear los LEDs en los pines especificados.
        :param pins: Lista de pines GPIO a parpadear.
        :param blink_count: Número de parpadeos.
        :param delay: Tiempo en segundos entre encendido y apagado.
        """
        for _ in range(blink_count):
            for pin in pins:
                GPIO.output(pin, GPIO.HIGH)  # Encender
            time.sleep(delay)  # Esperar
            for pin in pins:
                GPIO.output(pin, GPIO.LOW)  # Apagar
            time.sleep(delay)  # Esperar

    def listener_callback(self, msg):
        print("Mensaje recibido")
        command = msg.data.strip().upper()  # Convertir a mayúsculas y eliminar espacios
        
        # Apagar todos los LEDs primero
        all_pins = set(self.led_pins['T'])
        for pin in all_pins:
            GPIO.output(pin, GPIO.LOW)
        
        # Encender o parpadear LEDs según el comando
        if command in self.led_pins:
            if command == 'N':  # Si el comando es 'N', no hacer nada
                self.get_logger().info("Comando 'N' recibido. No se encienden LEDs.")
            else:
                self.blink_leds(self.led_pins[command])  # Parpadear LEDs
                self.get_logger().info(f"Comando '{command}' recibido. LEDs parpadeando: {self.led_pins[command]}")
        else:
            self.get_logger().warn(f"Comando '{command}' no válido. Use 'D', 'I', 'T' o 'N'.")

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