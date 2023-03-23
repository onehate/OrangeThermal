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


def led_blink(time_step, pid):
    while True:
        GPIO.setup(config.gpio_heat, GPIO.OUT)
        GPIO.output(config.gpio_heat, GPIO.HIGH)
        print("on")
        time.sleep(time_step * pid)
        GPIO.setup(config.gpio_heat, GPIO.OUT)
        GPIO.output(config.gpio_heat, GPIO.LOW)
        print("off")
        # time.sleep(1)
