import os
import sys
import logging
import time
import config

script_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, script_dir + "/lib/")

import OPi.GPIO as GPIO

GPIO.setboard(GPIO.H616)
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO.setup(config.gpio_heat, GPIO.OUT)


def led_blink(time_step, pid):
    while True:
        # logging.info("Temperature: %0.2f°C", temp.temperature())
        print("on")
        GPIO.output(config.gpio_heat, GPIO.HIGH)
        time.sleep(time_step * pid)
        GPIO.output(config.gpio_heat, GPIO.LOW)
        print("off")
        time.sleep(1)
