#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
from time import sleep

class ButtonPublisher(Node):
    def __init__(self):
        super().__init__('button_publisher')
        self.publisher = self.create_publisher(Bool, 'enable_Auto', 10)
        
        # Configuración del GPIO
        self.BUTTON_PIN = 22
        self.button_state = False
        self.last_button_state = False
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        
        # Detección de cambios en el botón
        GPIO.add_event_detect(
            self.BUTTON_PIN,
            GPIO.BOTH,
            callback=self.button_callback,
            bouncetime=300
        )
        
        self.get_logger().info(f"Monitorizando botón en GPIO {self.BUTTON_PIN}...")

    def button_callback(self, channel):
        current_button_state = GPIO.input(self.BUTTON_PIN)
        
        if current_button_state != self.last_button_state:
            self.button_state = current_button_state
            self.publish_button_state()
            self.last_button_state = current_button_state

    def publish_button_state(self):
        msg = Bool()
        msg.data = bool(self.button_state)
        self.publisher.publish(msg)
        self.get_logger().info(f"Estado del botón publicado: {msg.data}")

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ButtonPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Deteniendo el nodo...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()