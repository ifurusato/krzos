#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-12
# modified: 2025-07-01
#
# A UART slave for the STM32.
#
# UART 1 is used for communications when connecting between the Raspberry Pi.
# UART 4's pins conflict with use of the SD card so support for it was not
# built into MicroPython for the STM32H562. So only UART 2 or 3 are suitable.
#
# Example usage:
#
#   uart2 = UART(2, baudrate=115200)  # uses PA2 (TX), PA3 (RX)

import uasyncio as asyncio
import time
from pyb import LED, Pin, UART
from pyb import LED
from colorama import Fore, Style

from logger import Logger, Level
from payload import Payload
from uart_slave_base import UartSlaveBase

class Stm32UartSlave(UartSlaveBase):
    def __init__(self, uart_id=2, baudrate=115200, status=None):
        UartSlaveBase.__init__(self, 'stm32-uart', uart_id=uart_id, baudrate=baudrate, status=status)
        self._led   = LED(1)
        self._uart  = UART(uart_id)
        self._uart.init(baudrate=baudrate, bits=8, parity=None, stop=1)

#EOF
