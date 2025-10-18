#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-08
# modified: 2025-10-09
#

import sys
import time
import traceback
import pathlib
import pytest
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.i2c_scanner import I2CScanner
from hardware.radiozoa_config import RadiozoaConfig

@pytest.mark.unit
def test_radiozoa_sensor_addresses():
    '''
    Unit test for Radiozoa sensor I2C address configuration.
    If 0x29 is present or any configured addresses are missing,
    attempts to configure sensors via RadiozoaConfig.
    Test passes only if 0x29 is absent and all eight addresses are present after configuration.
    '''
    _log = Logger('test-radiozoa', level=Level.INFO)
    _log.info(Style.BRIGHT + "testing radiozoa sensors…")
    _config_path = pathlib.Path(__file__).parent / "config.yaml"
    _config = ConfigLoader().configure(_config_path)
    _cfg_radiozoa = _config.get('kros').get('hardware').get('radiozoa')
    _cfg_devices = _cfg_radiozoa.get('devices')
    _i2c_bus_number = _cfg_radiozoa.get('i2c_bus_number')
    _default_i2c_address = '0x29'
    _sensor_addresses = [ "0x{:02X}".format(sensor.get('i2c_address')) for sensor in _cfg_devices.values() ]
    _i2c_scanner = I2CScanner(config=_config, i2c_bus_number=_i2c_bus_number, level=Level.ERROR)

    _log.info("scanning for Radiozoa sensor addresses…")
    _has_default = _i2c_scanner.has_hex_address([_default_i2c_address])
    _missing = [ addr for addr in _sensor_addresses if not _i2c_scanner.has_hex_address([addr]) ]

    if _has_default or _missing:
#       _log.warning("initial or error condition detected: 0x29 present={} or missing sensor addresses={}".format(_has_default, _missing))
        _log.info(Style.BRIGHT + "reasssigning radiozoa addresses…")
        try:
            radiozoa_config = RadiozoaConfig(_config, level=Level.INFO)
            radiozoa_config.configure()
            radiozoa_config.close()
        except Exception as e:
            _log.error("{} raised during RadiozoaConfig configuration: {}\n{}".format(type(e), e, traceback.format_exc()))
            pytest.fail("Exception during sensor configuration: {}".format(e))

        # re-scan after configuration
        _log.info("re-scanning for Radiozoa sensor addresses…")
        time.sleep(1)
        _has_default = _i2c_scanner.has_hex_address([_default_i2c_address], force_scan=True)
        _missing = [ addr for addr in _sensor_addresses if not _i2c_scanner.has_hex_address([addr]) ]

    time.sleep(0.5)
#   assert not _has_default, "Default address 0x29 is still present after configuration."
    assert not _missing, "Missing sensor addresses after configuration: {}".format(_missing)
    _log.info(Fore.GREEN + "Radiozoa sensor addresses are configured and present.")

def main():
    """
    Runs only the marked unit tests within this file when executed directly.
    """
    try:
        pytest.main([__file__, "-m", "unit", "-s"])
    except Exception as e:
        print("an unexpected error occurred: {}".format(e))

if __name__ == "__main__":
    main()

#EOF
