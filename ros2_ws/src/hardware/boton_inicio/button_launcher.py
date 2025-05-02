#!/usr/bin/env python3

import RPi.GPIO as GPIO
import subprocess
import os
from time import sleep

# Configuración del GPIO
BUTTON_PIN = 4  # Usando GPIO4 (Pin 7 en la Raspberry Pi)
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Ruta absoluta del archivo launch (ajusta esto)
LAUNCH_PATH = "/home/crofi/AutoModelCrofi/ros2_ws/launch/AutoModelT.py"

# Variable para gestionar el proceso
launch_process = None
button_state = False

def button_callback(channel):
    global launch_process
    
    # Verifica si no hay un proceso activo o si terminó
    if launch_process is None or launch_process.poll() is not None:
        print("\nIniciando el launch file...")
        try:
            # Ejecuta ros2 launch (el entorno ya está cargado desde .bashrc)
            launch_process = subprocess.Popen(
                ["ros2", "launch", LAUNCH_PATH],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
        except Exception as e:
            print(f"Error al ejecutar el launch: {e}")
    else:
        print("El launch ya está en ejecución.")

# Configura la detección del botón (flanco de bajada con debounce)
GPIO.add_event_detect(
    BUTTON_PIN,
    GPIO.FALLING,
    callback=button_callback,
    bouncetime=500  # Anti-rebote (ms)
)

print(f"Esperando pulsación del botón (GPIO {BUTTON_PIN})...")

try:
    while True:
        sleep(1)
except KeyboardInterrupt:
    print("\nDeteniendo el script...")
    if launch_process is not None:
        launch_process.terminate()  # Termina el proceso al salir
finally:
    GPIO.cleanup()  # Limpia los pines GPIO