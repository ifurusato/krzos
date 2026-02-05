#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-01-27
# modified: 2026-01-28

import machine

from colorama import Fore, Style
from logger import Logger, Level

class I2CScanner:
    '''
    Do not specify SDA or SCL for the STM32, only the bus number.
    You may need to modify this for your microcontroller.

    Args:
        id:    identifies a particular I2C peripheral. Allowed values depend on the particular port/board.
        scl:   should be a pin object specifying the pin to use for SCL.
        sda:   should be a pin object specifying the pin to use for SDA.
    '''
    def __init__(self, i2c_bus=1, scl=None, sda=None, level=Level.INFO):
        self._log = Logger('i2c-scanner', level=level)
        if scl and sda:
            self.i2c = machine.I2C(i2c_bus, scl=scl, sda=sda)
        else:
            self.i2c = machine.I2C(i2c_bus)
        self._devices = []
        self._log.info('ready.')

    @property
    def devices(self):
        '''
        Return the scanned list of devices.
        '''
        self._devices = []

    def scan(self):
        '''
        Scan all valid 7-bit I2C addresses, returning a list of found addresses.
        '''
        self._log.info(Style.DIM + "starting I2C scan…")
        self._devices = self.i2c.scan()
        if len(self._devices) == 0:
            self._log.warning("no I2C devices found.")
        else:
            self._log.info('I2C scan complete: ' + Fore.GREEN + "{} devices found.".format(len(self._devices)))
        return self._devices

    def has_hex_address(self, addr):
        '''
        Returns True if a given address is in the scanned device list.
        '''
        return addr in self._devices

    def i2cdetect(self, color=Fore.CYAN):
        '''
        Display I2C device addresses in i2cdetect format to stdout.
        Scans if not already done, then prints a formatted grid.
        '''
        if not self._devices:
            self.scan()
        print(color)
        # print header
        print("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f")
        # print each row (0x00-0x70 in steps of 0x10)
        for row in range(0x00, 0x80, 0x10):
            print("{:02x}:".format(row), end='')
            for col in range(0x10):
                addr = row + col
                if addr < 0x08 or addr > 0x77:
                    # invalid I2C address range
                    print("   ", end='')
                elif addr in self._devices:
                    print(" {:02x}".format(addr), end='')
                else:
                    print(" --", end='')
            print()  # newline at end of row
        print(Style.RESET_ALL)

#EOF
