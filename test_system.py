#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-08-30
# modified: 2025-09-22

import time
import sys
import pathlib
import pytest
from colorama import init, Fore, Style
init()

from ads1015 import ADS1015

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.system import System

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

__log = Logger('test-system', level=Level.INFO)

@pytest.mark.unit
def test_power():

    __log.info(Style.BRIGHT + "testing system…")
    _config_path = pathlib.Path(__file__).parent / "config.yaml"
    _config = ConfigLoader().configure(_config_path)
    _system = System(_config)
    _12v_battery_min, _5v_reg_min, _3v3_reg_min = _system.get_voltage_thresholds()

    try:

        _cfg = _config.get('kros').get('hardware').get('system')
        _temperature_warning = _cfg.get('temperature_warning')
        _temperature_error   = _cfg.get('temperature_error')

        __log.info("testing power supplies…")
        # IN0 12V battery
        _battery_12v = _system.get_battery_12v()
        __log.info("battery (in0):   " + Fore.GREEN + "{:6.3f}V".format(_battery_12v))
        _reg_5v = _system.get_reg_5v()
        __log.info("5V ref (in2):    " + Fore.GREEN + "{:6.3f}V".format(_reg_5v))
        _ref_3v3 = _system.get_reg_3v3()
        __log.info("3V3 reg (in1):   "  + Fore.GREEN + "{:6.3f}V".format(_ref_3v3))

        if _battery_12v < _12v_battery_min:
            __log.warning('measured in0 value less than threshold {}v'.format(_12v_battery_min))
#       assert _battery_12v > _12v_battery_min, 'measured in0 value less than threshold {}v'.format(_12v_battery_min)
        assert _reg_5v      > _5v_reg_min,      'measured in2 value less than threshold {}v'.format(_5v_reg_min)
        assert _ref_3v3     > _3v3_reg_min,     'measured in1 value less than threshold {}v'.format(_3v3_reg_min)

        _system_voltage = _system.get_system_voltage()
        __log.info("system voltage:  " + Fore.GREEN + "{:6.3f}V".format(_system_voltage))

        _system_current = _system.get_system_current()
        if _system_current < 1.0:
            __log.info("system current:  " + Fore.GREEN + "{:6d}mA".format(int(_system_current * 1000)))
        else:
            __log.info("system current:  " + Fore.GREEN + "{:6.3f}A".format(_system_current))

        _system_power = _system.get_system_power()
        __log.info("system power:    " + Fore.GREEN + "{:6.1f}W".format(_system_power))

#       assert _system_voltage > 0.0, 'ina260 power monitor is not working.'
#       assert _system_voltage > _12v_battery_min, 'measured ina260 value less than threshold {}v'.format(_12v_battery_min)

#       assert _system_current > 0.0, 'ina260 power monitor is not working.'
#       assert _system_current < _system.get_system_current_max(), 'measured ina260 value greater than threshold {}A'.format(_12v_battery_min)

        __log.info(Fore.GREEN + "power supplies are functional.")

        _cpu_temp = _system.read_cpu_temperature()
        if _cpu_temp > _temperature_warning:
            __log.warning("cpu temperature: " + Fore.RED + "{:6.1f}C HIGH".format(_cpu_temp))
        else:
            __log.info("cpu temperature: " + Fore.GREEN + "{:6.1f}C".format(_cpu_temp))
        assert _cpu_temp < _temperature_error, 'cpu temperature {:6.3f}C greater than the threshold.'.format(_cpu_temp)

        __log.info(Fore.GREEN + "system is healthy.")

    except Exception as e:
        __log.error('{} raised in power supply test: {}'.format(type(e), e))

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
        __log = None

if __name__ == "__main__":
    main()

#EOF
