#!/usr/bin/python
# -*- coding: utf-8; python-indent-offset: 4; -*-


import logging
import time
import sys
from max31865 import MAX31865

sys.dont_write_bytecode = True
import config
sys.dont_write_bytecode = False

import OPi.GPIO as GPIO
GPIO.setboard(GPIO.H616)
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO.setup(config.gpio_heat, GPIO.OUT)

def main():
    """
    Example demonstrating continous read out of temperature of MAX31865.
    """
    # NOTE Pins use "gpio numbering", not "physical numbering"! https://www.raspberrypi.org/documentation/usage/gpio-plus-and-raspi2/README.md
    # e.g. 21 is GPIO21, not physical pin 21
    with MAX31865(config2.gpio_sensor_cs,
                config2.gpio_sensor_miso,
                config2.gpio_sensor_mosi,
                config2.gpio_sensor_clock
    ) as temp:
        print(temp.temperature())       
        while True:
            logging.info('Temperature: %0.2f°C', temp.temperature())
            GPIO.setup(config.gpio_heat, GPIO.OUT)
            GPIO.output(config.gpio_heat, GPIO.HIGH)
            time.sleep(3)
            GPIO.output(config.gpio_heat, GPIO.LOW)
            time.sleep(1)

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    main()
