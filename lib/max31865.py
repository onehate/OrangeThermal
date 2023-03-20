#!/usr/bin/python
# -*- coding: utf-8; python-indent-offset: 4; -*-

# The MIT License (MIT)
#
# Copyright (c) 2015 Stephen P. Smith
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import logging
import time
import math
import OPi.GPIO as GPIO


class TemperatureConverter(object):
    """
    Converter for raw temperature value read out from MAX31865 by using
    Callendar-Van-Dusen equation (https://en.wikipedia.org/wiki/Callendar%E2%80%93Van_Dusen_equation).

    NOTE: Implementation focuses on positive temperatures in °C.
          For negative temperatures equations and constants need adjustments.
    """

    pt100_data = {
        'resistor': 4300.0,
        'resistance_at_0': 1000.0,
        'A': 0.00390830,
        'B': -0.000000577500,
        'C': -0.00000000000418301,
    }
    """Constant data of PT100 and corresponding resistor.
    Items are:
     - resistor: Reference Resistor. Adafruit board uses 430ohm (https://www.adafruit.com/product/3328)
     - resistance_at_0: Resistance of resistor at 0°C.
     - A: Constant A for Callendar-Van-Dusen equation (Derived experimentally)
     - B: Constant B for Callendar-Van-Dusen equation (Derived experimentally)
     - C: Constant C for Callendar-Van-Dusen equation (Derived experimentally)
    """

    def __init__(self, method='quadratic'):
        """
        :param method: Either 'quadratic' or 'quartic', where 'quadratic' is faster but less accurate.
        """
        setattr(self, 'convert', getattr(self, 'convert_%s' % method))

    def convert(self, raw_temp):
        """
        Convert 0 <= raw_temp < 2^15 to temperature in °C.
        """
        pass

    def convert_quadratic(self, raw_temp):
        """
        Convert 0 <= raw_temp < 2^l5 to temperature in °C using quadratic polynom:

          R(T)=R(0)(1+A*T+B*T^{2})

        => T = (-A*R(0) + sqrt(A^2 R(0)^2 - 4*B*R(0)*(R(0) - R(T)))) / (2*B*R(0))
        """
        r0 = self.pt100_data['resistance_at_0']
        a = self.pt100_data['A']
        b = self.pt100_data['B']
        temp = raw_temp * self.pt100_data['resistor'] / (2**15 - 1)
        temp = ((-a * r0) + math.sqrt(a**2 * r0**2 - 4*b*r0*(r0 - temp))) / (2*b*r0)
        logging.getLogger(__name__).debug('convert_quadratic: %d -> %0.2f°C', raw_temp, temp)
        return temp

    def convert_quartic(self, raw_temp):
        """
        Convert 0 <= raw_temp < 2^15 to temperature in °C using quartic polynom:

              R(T) = R(0)[1+A*T+B*T^{2}+(T-100)C*T^{3}]

        => 0 = R(0)[1 + A*T + B*T^2 - 100*C*T^3 + C*T^4] - R(T)

        Solution is calculated numerically, thus numpy is needed.
        """
        import numpy
        r0 = self.pt100_data['resistance_at_0']
        a = self.pt100_data['A']
        b = self.pt100_data['B']
        c = self.pt100_data['C']
        temp = raw_temp * self.pt100_data['resistor'] / (2**15 - 1)
        roots = numpy.roots([c*r0, -100*r0*c, b*r0, a*r0, r0 - temp])
        temp = abs(roots[-1])
        logging.getLogger(__name__).debug('convert_quartic: %d -> %0.2f°C', raw_temp, temp)
        return temp


class MAX31865(object):
    """
    Reading Temperature from the MAX31865 with GPIO using the Raspberry Pi.
    Any 4 pins can be used to establish software based SPI to MAX31865.
    """

    REGISTERS = {
        'config': 0,
        'rtd_msb': 1,
        'rtd_lsb': 2,
        'high_fault_threshold_msb': 3,
        'high_fault_threshold_lsb': 4,
        'low_fault_threshold_msb': 5,
        'low_fault_threshold_lsb': 6,
        'fault_status': 7,
    }
    """
    Definition of register addresses. (https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf)

    Name                     ReadAddress WriteAddress PorState Access
    Configuration            00h         80h          00h      R/W
    RTD MSBs                 01h         —            00h      R
    RTD LSBs                 02h         —            00h      R
    High Fault Threshold MSB 03h         83h          FFh      R/W
    High Fault Threshold LSB 04h         84h          FFh      R/W
    Low Fault Threshold MSB  05h         85h          00h      R/W
    Low Fault Threshold LSB  06h         86h          00h      R/W
    Fault Status             07h         —            00h      R
    """

    REGISTERS_WRITE_MASK = 0x80
    """Mask to be ORed to register addresses when writing."""

    REGISTER_CONFIGURATION_ONE_SHOT_3_WIRE = 0b10110011
    """
    Configuration 0b10110010 == 0xB2:
    bit 7: Vbias -> 1 (ON)
    bit 6: Conversion Mode -> 0 (MANUAL)
    bit 5: 1-shot -> 1 (ON)
    bit 4: 3-wire select -> 1 (3 wire config)
    bit 3-2: fault detection cycle -> 0 (none)
    bit 1: fault status clear -> 1 (clear any fault)
    bit 0: 50/60 Hz filter select -> 0 (60Hz)
    """

    REGISTER_CONFIGURATION_ONE_SHOT = 0b10100011
    """
    Configuration 0b10110010 == 0xB2:
    bit 7: Vbias -> 1 (ON)
    bit 6: Conversion Mode -> 0 (MANUAL)
    bit 5: 1-shot -> 1 (ON)
    bit 4: 3-wire select -> 0 (2 or 4 wire config)
    bit 3-2: fault detection cycle -> 0 (none)
    bit 1: fault status clear -> 1 (clear any fault)
    bit 0: 50/60 Hz filter select -> 0 (60Hz)
    """

    def __init__(self, cs_pin, miso_pin, mosi_pin, clk_pin, number_of_wires=2):
        self.cs_pin = cs_pin
        self.miso_pin = miso_pin
        self.mosi_pin = mosi_pin
        self.clk_pin = clk_pin
        self.number_of_wires = number_of_wires
        self.temperature_converter = TemperatureConverter()
        """Number of RTD wires. One of 2,3,4."""

    def _setup_GPIO(self):
        """
        Setup GPIOs for SPI connection:
        CS: Chip Select (also called SS)
        CLK: Serial Clock
        MISO: Master In Slave Out (SDO at slave)
        MOSI: Master Out Slave In (SDI at slave)
        """
        GPIO.setwarnings(False)
        GPIO.setboard(GPIO.H616)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.cs_pin, GPIO.OUT)
        GPIO.setup(self.miso_pin, GPIO.IN)
        GPIO.setup(self.mosi_pin, GPIO.OUT)
        GPIO.setup(self.clk_pin, GPIO.OUT)

        GPIO.output(self.cs_pin, GPIO.HIGH)
        GPIO.output(self.clk_pin, GPIO.LOW)
        GPIO.output(self.mosi_pin, GPIO.LOW)

    def __enter__(self):
        self._setup_GPIO()
        return self

    def __exit__(self, *k):
        GPIO.cleanup()

    def write_register(self, register, data):
        """
        Write data to register.

        :param register: Either name or address of register.
        :param data: Single byte to be written.
        """
        GPIO.output(self.cs_pin, GPIO.LOW)

        if isinstance(register, str):
            register = self.REGISTERS[register]
        register |= self.REGISTERS_WRITE_MASK

        # Send address byte
        self.send(register)
        # Send data byte
        self.send(data)
        logging.getLogger(__name__).debug('write_register: register=0x%02x data=0x%02x', register, data)

        GPIO.output(self.cs_pin, GPIO.HIGH)

    def read_register(self, register):
        """
        Read data from register.

        :param register: Either name or address of register.
        :return: One byte of data.
        """
        GPIO.output(self.cs_pin, GPIO.LOW)

        if isinstance(register, str):
            register = self.REGISTERS[register]

        self.send(register)
        data = self.recv()
        logging.getLogger(__name__).debug(
            'read_register: register=0x%02x (%s) data=0x%02x',
            register,
            [k for k, v in self.REGISTERS.items() if v == register][0],
            data
        )

        GPIO.output(self.cs_pin, GPIO.HIGH)
        return data

    def read_registers(self):
        """
        Read all registers.

        :return: List of 8 bytes data.
        """
        # NOTE: Reusage of self.read_register is slower but more clean.
        data = [self.read_register(r) for r in range(len(self.REGISTERS))]
        logging.getLogger(__name__).debug('read_registers: %s', data)
        for reg_name, reg_address in self.REGISTERS.items():
            logging.getLogger(__name__).debug('read_registers: %s = 0x%02x', reg_name, data[reg_address])
        return data

    def temperature(self):
        """
        Read out temperature. Conversion to °C included.
        """
        self.write_register('config', MAX31865.REGISTER_CONFIGURATION_ONE_SHOT)

        # Sleep to wait for conversion (Conversion time is less than 100ms)
        time.sleep(0.1)

        temp = self.read_register('rtd_msb')
        temp = (temp << 8) | self.read_register('rtd_lsb')

        # Check if error bit was set
        if temp & 0x01:
            raise MAX31865Error(self)
        return self.temperature_converter.convert(temp >> 1)

    def send(self, byte):
        """
        Send one byte via configured SPI.
        """
        for bit in range(8):
            GPIO.output(self.clk_pin, GPIO.HIGH)
            if (byte & 0x80):
                GPIO.output(self.mosi_pin, GPIO.HIGH)
            else:
                GPIO.output(self.mosi_pin, GPIO.LOW)
            byte <<= 1
            GPIO.output(self.clk_pin, GPIO.LOW)

    def recv(self):
        """
        Receive one byte via configured SPI.
        """
        byte = 0x00
        for bit in range(8):
            GPIO.output(self.clk_pin, GPIO.HIGH)
            byte <<= 1
            if GPIO.input(self.miso_pin):
                byte |= 0x1
            GPIO.output(self.clk_pin, GPIO.LOW)
        return byte


class MAX31865Error(Exception):
    """
    Fault handling of MAX31865.
    MAX31865 includes onchip fault detection.

    TODO: Improve fault detection. Currently only status register is read.
    """

    def __init__(self, max31865):
        self.max31865 = max31865
        super(MAX31865Error, self).__init__(self.status_message())

    def status_message(self):
        """
        10 Mohm resistor is on breakout board to help
        detect cable faults
        bit 7: RTD High Threshold / cable fault open
        bit 6: RTD Low Threshold / cable fault short
        bit 5: REFIN- > 0.85 x VBias -> must be requested
        bit 4: REFIN- < 0.85 x VBias (FORCE- open) -> must be requested
        bit 3: RTDIN- < 0.85 x VBias (FORCE- open) -> must be requested
        bit 2: Overvoltage / undervoltage fault
        bits 1,0 don't care
        """
        status = self.max31865.read_register('fault_status')
        if status & 0x80:
            return "High threshold limit (Cable fault/open)"
        if status & 0x40:
            return "Low threshold limit (Cable fault/short)"
        if status & 0x04:
            return "Overvoltage or Undervoltage Error"