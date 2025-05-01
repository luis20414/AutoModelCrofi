#!/usr/bin/env python3
import RPi.GPIO as GPIO
import subprocess
from time import sleep

BUTTON_PIN = 21
launch_active = False

def button_callback(channel):
    global launch_active
    if not launch_active:
        launch_active = True
        subprocess.Popen(["ros2", "launch", "~/AutoModelCrofi/ros2_ws/launch/AutoModel.py"])
        # LED de confirmación opcional

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, 
                     callback=button_callback, bouncetime=300)

try:
    while True:
        sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()
