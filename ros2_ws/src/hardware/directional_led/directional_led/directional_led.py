#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import RPi.GPIO as GPIO 
import time

class LEDController(Node):
    def __init__(self):
        super().__init__('led_controller')
        
        self.led_pins = {
            'D': [6, 26],
            'I': [13, 19],
            'N': [],
            'T': [6, 13, 19, 26]
        }

        GPIO.setmode(GPIO.BCM)
        all_pins = set(self.led_pins['D'] + self.led_pins['I'] + self.led_pins['T'])
        for pin in all_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        self.allow_processing = False  # Solo se activa en stop_state=True

        self.subscription = self.create_subscription(String, 'rebase', self.listener_callback, 10)
        self.stop_sub = self.create_subscription(Bool, '/stop_state', self.stop_callback, 10)

        self.get_logger().info("LED Controller listo, esperando mensajes en /rebase...")

    def stop_callback(self, msg):
        self.allow_processing = msg.data
        estado = "activado" if msg.data else "desactivado"
        self.get_logger().info(f"Procesamiento {estado} por /stop_state = {msg.data}")

    def blink_leds(self, pins, blink_count=1, delay=0.5):
        for _ in range(blink_count):
            for pin in pins:
                GPIO.output(pin, GPIO.HIGH)
            time.sleep(delay)
            for pin in pins:
                GPIO.output(pin, GPIO.LOW)
            time.sleep(delay)

    def listener_callback(self, msg):
        if not self.allow_processing:
            self.get_logger().info("Ignorando comando /rebase: procesamiento no permitido (stop_state = False).")
            return

        command = msg.data.strip().upper()
        if command in self.led_pins:
            if command == 'N':
                self.get_logger().info("Comando 'N' recibido. No se encienden LEDs.")
            else:
                self.blink_leds(self.led_pins[command])
                self.get_logger().info(f"Comando '{command}' recibido. LEDs parpadeando: {self.led_pins[command]}")
        else:
            self.get_logger().warn(f"Comando '{command}' no válido. Use 'D', 'I', 'T' o 'N'.")

    def __del__(self):
        GPIO.cleanup()

def main(args=None):
    print("Luces listas")
    rclpy.init(args=args)
    node = LEDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
