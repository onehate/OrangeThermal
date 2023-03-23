#!/usr/bin/python
# -*- coding: utf-8; python-indent-offset: 4; -*-
import os
import sys
import logging
import time
import config

script_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, script_dir + "/lib/")

from max31865 import MAX31865

import OPi.GPIO as GPIO

GPIO.setboard(GPIO.H616)
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO.setup(config.gpio_heat, GPIO.OUT)


def main():
    """
    Example demonstrating continous read out of temperature of MAX31865.
    """
    with MAX31865(
        config.gpio_sensor_cs,
        config.gpio_sensor_miso,
        config.gpio_sensor_mosi,
        config.gpio_sensor_clock,
    ) as temp:
        print(temp.temperature())
        while True:
            logging.info("Temperature: %0.2f°C", temp.temperature())
            GPIO.output(config.gpio_heat, GPIO.HIGH)
            time.sleep(3)
            GPIO.setboard(GPIO.H616)
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            GPIO.setup(config.gpio_heat, GPIO.OUT)
            time.sleep(1)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    main()
