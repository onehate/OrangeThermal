#!/usr/bin/python
# -*- coding: utf-8; python-indent-offset: 4; -*-


import logging
import time
from max31865 import MAX31865


def main():
    """
    Example demonstrating continous read out of temperature of MAX31865.
    """
    # NOTE Pins use "gpio numbering", not "physical numbering"! https://www.raspberrypi.org/documentation/usage/gpio-plus-and-raspi2/README.md
    # e.g. 21 is GPIO21, not physical pin 21
    with MAX31865(
            cs_pin = 24,
            miso_pin = 21,
            mosi_pin = 19,
            clk_pin = 23,
    ) as temp:
        while True:
            logging.info('Temperature: %0.2f°C', temp.temperature())
            time.sleep(1)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    main()