#!/usr/bin/env python3

import RPi.GPIO as GPIO
import subprocess
import os
from time import sleep

# Configuración
BUTTON_PIN = 22          # GPIO4 (Pin 7)
LAUNCH_PATH = "/home/crofi/AutoModelCrofi/ros2_ws/launch/AutoModelT.py"  # Ajusta la ruta

# Estado del botón y proceso
launch_process = None
button_state = False    # False = circuito abierto, True = cerrado
last_button_state = False

def toggle_launch():
    global launch_process, button_state
    
    if button_state:  # Botón activado (cerrado)
        if launch_process is None or launch_process.poll() is not None:
            print("\nIniciando el launch file...")
            launch_process = subprocess.Popen(
                ["ros2", "launch", LAUNCH_PATH],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
    else:  # Botón desactivado (abierto)
        if launch_process is not None and launch_process.poll() is None:
            print("\nDeteniendo el launch file...")
            launch_process.terminate()
            launch_process = None

def button_callback(channel):
    global button_state, last_button_state
    current_button_state = GPIO.input(BUTTON_PIN)
    
    # Solo detectamos un cambio si el estado actual es diferente del anterior
    if current_button_state != last_button_state:
        button_state = current_button_state  # Actualizamos el estado
        toggle_launch()
        last_button_state = current_button_state  # Actualizamos el estado previo del botón

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Configuración de evento de detección del cambio de estado
GPIO.add_event_detect(
    BUTTON_PIN,
    GPIO.BOTH,  # Detecta cambios (HIGH->LOW y LOW->HIGH)
    callback=button_callback,
    bouncetime=300  # Antirrebote (300 ms)
)

print(f"Control de Launch con Botón Latch (GPIO {BUTTON_PIN})...")

try:
    while True:
        # Verifica el estado del pin y lo imprime
        button_state_value = GPIO.input(BUTTON_PIN)
        print(f"Estado del botón: {button_state_value}")  # 1 es HIGH, 0 es LOW
        sleep(1)
except KeyboardInterrupt:
    print("\nDeteniendo el script...")
    if launch_process is not None:
        launch_process.terminate()
finally:
    GPIO.cleanup()
