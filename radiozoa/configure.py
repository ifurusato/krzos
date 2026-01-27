#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-01-27
# modified: 2026-01-27

import sys
import time
from colorama import Fore, Style

from logger import Logger, Level
from i2c_scanner import I2CScanner
from radiozoa_config import RadiozoaConfig
from device import Device

# force module reload
for mod in ['main', 'configure', 'radiozoa_config']:
    if mod in sys.modules:
        del sys.modules[mod]

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

log = Logger('configure', level=Level.INFO)

log.info(Style.BRIGHT + "radiozoa sensor configuration…")

for dev in Device.all():
    log.info("device '{}'; I2C address: 0x{:02X}; xshut: {}".format(dev.label, dev.i2c_address, dev.xshut))

# scan for existing sensors
scanner = I2CScanner(i2c_bus=1)
devices = scanner.scan()
for device in devices:
    log.info(Style.DIM + '  device: 0x{:02X}'.format(device))

_default_i2c_address = 0x29
_sensor_addresses = [ dev.i2c_address for dev in Device.all() ]

log.info("checking for default address and missing sensors…")
_has_default = scanner.has_hex_address(_default_i2c_address)
_missing = [ addr for addr in _sensor_addresses if not scanner.has_hex_address(addr) ]

if _has_default or _missing:
    if _has_default:
        log.info("found default 0x{:02X} device; reassigning radiozoa addresses…".format(_default_i2c_address))
    if _missing:
        log.info("missing sensor addresses: {}".format([ "0x{:02X}".format(addr) for addr in _missing ]))
    
    try:
        radiozoa_config = RadiozoaConfig(i2c_bus=1, level=Level.INFO)
        radiozoa_config.configure()
        radiozoa_config.close()
    except Exception as e:
        log.error("{} raised during RadiozoaConfig configuration: {}".format(type(e), e))
        raise

    # re-scan after configuration
    log.info("re-scanning for radiozoa sensor addresses…")
    time.sleep_ms(1000)
    devices = scanner.scan()
    _has_default = scanner.has_hex_address(_default_i2c_address)
    _missing = [ addr for addr in _sensor_addresses if not scanner.has_hex_address(addr) ]

    if _has_default:
        log.warning("default address 0x{:02X} is still present after configuration.".format(_default_i2c_address))
    if _missing:
        log.error("missing sensor addresses after configuration: {}".format([ "0x{:02X}".format(addr) for addr in _missing ]))
    else:
        log.info(Fore.GREEN + "radiozoa sensor addresses configured successfully.")
else:
    log.info(Fore.GREEN + "radiozoa already configured.")

log.info('complete.')

#EOF
