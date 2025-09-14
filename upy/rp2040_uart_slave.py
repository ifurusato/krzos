#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-12
# modified: 2025-06-24
#
# A UART slave for the RP2040.
#
# Note that the TX and RX pins may vary depending on the hardware implementation.
#

import uasyncio as asyncio
import time
from machine import Pin, UART
from colorama import Fore, Style

from logger import Logger, Level
from payload import Payload
from uart_slave_base import UartSlaveBase

class RP2040UartSlave(UartSlaveBase):
    def __init__(self, uart_id=1, baudrate=115200, status=None, rx_pin=5, tx_pin=4, led_pin=25):
        UartSlaveBase.__init__(self, 'rp2040-uart', uart_id=uart_id, baudrate=baudrate, status=status)
        self._log.info('pins: rx={}; tx={}.'.format(rx_pin, tx_pin))
        self.led_pin  = led_pin # Pin for the LED
        # set up LED pin
        self.led = Pin(self.led_pin, Pin.OUT)
        # set up UART connection with custom TX and RX pins
        self._uart = UART(uart_id, baudrate=baudrate, bits=8, parity=None, stop=1, tx=Pin(tx_pin), rx=Pin(rx_pin))
        # ready

#EOF
