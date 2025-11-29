#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-09-21
# modified: 2025-09-21

import time
import pytest
import pathlib
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from core.orientation import Orientation
from hardware.fore_sensor import ForeSensor
from hardware.avoid_sensor import AvoidSensor

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

__log = Logger('test-proximity', level=Level.INFO)

def get_color_cm(distance):
    if distance > 100:
        return Fore.GREEN + Style.DIM
    elif distance > 50:
        return Fore.GREEN
    elif distance > 25:
        return Fore.YELLOW
    else:
        return Fore.RED

@pytest.mark.unit
def test_proximity():
    __log.info(Style.BRIGHT + "testing if proximity sensors are functional…")
    __log.info('starting test…')
    __log.info(Fore.YELLOW + 'Press Ctrl-C to exit.')
    _fore_sensor = None
    _port_sensor = None
    _stbd_sensor = None
    _aft_sensor  = None
    _count = 0
    try:
        _config_path = pathlib.Path(__file__).parent / "config.yaml"
        _config = ConfigLoader().configure(_config_path)
        _fore_sensor = ForeSensor(_config, level=Level.INFO)
        _port_sensor = AvoidSensor(_config, Orientation.PORT, level=Level.INFO)
        _stbd_sensor = AvoidSensor(_config, Orientation.STBD, level=Level.INFO)
        _aft_sensor  = AvoidSensor(_config, Orientation.AFT, level=Level.INFO)
        _fore_sensor.enable()
        _port_sensor.enable()
        _stbd_sensor.enable()
        _aft_sensor.enable()
        time.sleep(1)

        for _ in range(10):
            # fore sensor ..................................
            _raw_value = _fore_sensor.get_raw_value()
            _value = _fore_sensor.get_value()
            _fore_distance = _fore_sensor.get_distance_cm()
            if _fore_distance:
                _count += 1
                __log.info('raw: {:5.2f}; value: {:d}; '.format(_raw_value, _value) + get_color_cm(_fore_distance) + 'distance: {:d}cm'.format(_fore_distance))
            else:
                __log.warning('could not get fore sensor distance.')
            # port sensor ..................................
            _port_distance = _port_sensor.get_distance()
            if _port_distance:
                _count += 1
                __log.info(get_color_cm(_port_distance / 10) + 'port distance: {:d}cm'.format(_port_distance))
            else:
                __log.warning('could not get port sensor distance.')
            # stbd sensor ..................................
            _stbd_distance = _stbd_sensor.get_distance()
            if _stbd_distance:
                _count += 1
                __log.info(get_color_cm(_stbd_distance / 10) + 'stbd distance: {:d}cm'.format(_stbd_distance))
            else:
                __log.warning('could not get stbd sensor distance.')
            # aft sensor ...................................
            _aft_distance  = _aft_sensor.get_distance()
            if _aft_distance:
                _count += 1
                __log.info(get_color_cm(_aft_distance / 10) + 'aft distance: {:d}cm'.format(_aft_distance))
            else:
                __log.warning('could not get aft sensor distance.')
            time.sleep(0.1)

    except KeyboardInterrupt:
        __log.info('Ctrl-C caught; exiting…')
    except Exception as e:
        __log.error('error: {}'.format(e))
    finally:
        if _fore_sensor:
            _fore_sensor.close()
        if _port_sensor:
            _port_sensor.close()
        if _stbd_sensor:
            _stbd_sensor.close()
        if _aft_sensor:
            _aft_sensor.close()

        assert _count > 6
        _percent = int(_count / 4 * 100)
        __log.info('complete; success: {}%'.format(_percent))

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
