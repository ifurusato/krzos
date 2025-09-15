#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   altheim
# created:  2020-02-14
# modified: 2025-09-09

from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from hardware.i2c_scanner import I2CScanner

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    level = Level.INFO
    log = Logger('main', level)
    i2c_bus_number = 0
    log.info('scanning for I²C devices on bus {}…'.format(i2c_bus_number))
    scanner = I2CScanner(i2c_bus_number=i2c_bus_number, level=Level.INFO)

    _addresses = scanner.get_int_addresses()
    _hex_addresses = scanner.get_hex_addresses()
    log.info('available I²C device(s):')
    if len(_addresses) == 0:
        log.warning('no devices found.')
        return
    else:
        for n in range(len(_addresses)):
            address = _addresses[n]
            hex_address = _hex_addresses[n]
            log.info('device: {0} ({1})'.format(address, hex_address))

        _has_0x29 = scanner.has_hex_address(['0x29'])
        log.info('has 0x29? {}'.format(_has_0x29))

if __name__== "__main__":
    main()

#EOF
