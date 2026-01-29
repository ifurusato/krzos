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

import sys
import time
from colorama import Fore, Style

from logger import Logger, Level
from i2c_scanner import I2CScanner
from radiozoa_config import RadiozoaConfig
from device import Device
from blinker import Blinker

# force module reload
for mod in ['main', 'configure', 'i2c_scanner', 'radiozoa_config']:
    if mod in sys.modules:
        del sys.modules[mod]

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

class Configure:
    def __init__(self):
        self._log = Logger('configure', level=Level.INFO)
        self._blinker = Blinker(50, 250)
        self._scanner = I2CScanner(i2c_bus=1)
        self._default_i2c_address = 0x29
        self._devices = []

    def get_devices(self):
        return self._devices

    def configure(self):
        self._log.info(Style.BRIGHT + "radiozoa sensor configuration…")
        for dev in Device.all():
            self._log.info("device '{}'; I2C address: 0x{:02X}; xshut: {}".format(dev.label, dev.i2c_address, dev.xshut))
        # scan for existing sensors
        self._devices = self._scanner.scan()
        for device in self._devices:
            self._log.info(Style.DIM + '  device: 0x{:02X}'.format(device))

        _sensor_addresses = [ dev.i2c_address for dev in Device.all() ]
        self._log.info("checking for default address and missing sensors…")
        _has_default = self._scanner.has_hex_address(self._default_i2c_address)
        _missing = [ addr for addr in _sensor_addresses if not self._scanner.has_hex_address(addr) ]

        if _has_default or _missing:
            self._scanner.i2cdetect(Fore.WHITE)
            if _has_default:
                self._log.info(Fore.YELLOW + "found default 0x{:02X} device; reassigning radiozoa addresses…".format(self._default_i2c_address))
            if _missing:
                self._log.info(Fore.YELLOW + "missing sensor addresses: {}".format([ "0x{:02X}".format(addr) for addr in _missing ]))
            try:
                radiozoa_config = RadiozoaConfig(i2c_bus=1, level=Level.INFO)
                radiozoa_config.configure()
                radiozoa_config.close()
            except Exception as e:
                self._log.error("{} raised during RadiozoaConfig configuration: {}".format(type(e), e))
                raise
            # re-scan after configuration
            self._log.info("re-scanning for radiozoa sensor addresses…")
            time.sleep_ms(1000)
            self._devices = self._scanner.scan()
            _has_default = self._scanner.has_hex_address(self._default_i2c_address)
            _missing = [ addr for addr in _sensor_addresses if not self._scanner.has_hex_address(addr) ]
            if _has_default:
                self._blinker.set_duty_cycle(250, 50)
                self._log.warning("default address 0x{:02X} is still present after configuration.".format(self._default_i2c_address))
                self._scanner.i2cdetect(Fore.RED)
            if _missing:
                self._blinker.set_duty_cycle(50, 100)
                self._log.error("missing sensor addresses after configuration: {}".format([ "0x{:02X}".format(addr) for addr in _missing ]))
                self._scanner.i2cdetect(Fore.RED)
            else:
                self._blinker.set_duty_cycle(50, 2950)
                self._log.info(Fore.GREEN + "radiozoa sensor addresses configured successfully.")
                self._scanner.i2cdetect(Fore.GREEN)
        else:
            self._log.info(Fore.GREEN + "radiozoa already configured.")
            self._blinker.set_duty_cycle(50, 2950)
            self._scanner.i2cdetect(Fore.CYAN)
        self._log.info('complete.')

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def main():
    print('configuring…')
    config = Configure()
    config.configure()

if __name__ == 'configure':
    main() 

#EOF
