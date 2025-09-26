#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-08-30
# modified: 2025-08-31
#

import sys
import pytest
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.i2c_scanner import I2CScanner, DeviceNotFound

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

__log = Logger('test-devices', level=Level.INFO)
__config = ConfigLoader(level=Level.ERROR).configure()
__i2c_scanner = I2CScanner(__config, level=Level.ERROR)

@pytest.mark.unit
def test_required_devices():
    '''
    Tests for the required devices for the KRZ04.
    '''
    _show_optional = True
    __log.info(Style.BRIGHT + "testing existence of required and optional devices…")
    _devices = __config['kros'].get('hardware').get('devices')
    for _device in _devices:
        hex_address = "0x{:02X}".format(_device['address'])
        name        = _device['name']
        required    = _device['required']
        found = __i2c_scanner.has_hex_address([hex_address])
        if required:
            assert found, "{} not found at {}".format(name, hex_address)
            __log.info("{}: ".format(hex_address) + Fore.GREEN + "{}".format(name))
        elif _show_optional:
            if found:
                __log.info(Style.DIM + "{}: ".format(hex_address) + Fore.GREEN + "{}".format(name) + Fore.CYAN + " (optional)")
            else:
                __log.debug(Style.DIM + "{}: ".format(hex_address) + Fore.GREEN + "{}".format(name) + Fore.CYAN + " (not found, optional)")
    __log.info(Fore.GREEN + "required devices are available.")

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def main():
    global __log
    """
    Runs only the marked unit tests within this file when executed directly.
    """
    try:
        pytest.main([__file__, "-m", "unit", "-s"])
        __log.info("test execution complete.")
    except Exception as e:
        __log.error("an unexpected error occurred: {}".format(e))
    finally:
        __log         = None
        __config      = None
        __i2c_scanner = None

if __name__ == "__main__":
    main()

#EOF
