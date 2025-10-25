#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-05
# modified: 2025-09-22
#
# Tests the port and starboard motors for directly by setting their power, from
# a digital potentiometer, without the intermediaries of speed, slew, or PID
# controllers.
#

import sys, time, traceback
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from core.orientation import Orientation
from hardware.distance_sensors import DistanceSensors
from hardware.radiozoa_sensor import RadiozoaSensor
from hardware.side_sensor import SideSensor

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_start_time = dt.now()
_radiozoa_sensor = None
_distance_sensors = None
_log = Logger('test', Level.INFO)

try:

    _log.info('starting test…')
    time.sleep(2)

    # read YAML configuration
    _level = Level.INFO
    _config = ConfigLoader(Level.INFO).configure()

    _radiozoa_sensor = RadiozoaSensor(_config)
    _radiozoa_sensor.enable()
    _distance_sensors = DistanceSensors(_config)
    _distance_sensors.enable()

    _side_sensor = SideSensor(_config, Orientation.STBD)
#   _stbd_sensor = _distance_sensors.get_sensor(Orientation.STBD)

    _side_sensor.enable()

    while True:
        _stbd_distance = None #_stbd_sensor.get_distance()
        _side_distance = _side_sensor.get_distance()
        if _side_distance and _stbd_distance:
            _log.info('side distance: ' + Fore.YELLOW + '{:4.2f}; '.format(_side_distance) 
                    + Fore.CYAN + 'tof distance: ' + Fore.GREEN + '{:4.2f}'.format(_stbd_distance))
        elif _side_distance:
            _log.info('side distance: ' + Fore.YELLOW + '{:4.2f}; '.format(_side_distance))
        elif _stbd_distance:
            _log.info('stbd distance: ' + Fore.GREEN + '{:4.2f}'.format(_stbd_distance))
        else:
            _log.info('distances: ' + Fore.YELLOW + Style.DIM + 'none')
        time.sleep(0.05)

except KeyboardInterrupt:
    _log.info('Ctrl-C caught; exiting…')
except Exception as e:
    _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    if _radiozoa_sensor:
        _radiozoa_sensor.close()
    if _distance_sensors:
        _distance_sensors.close()
    _log.info('closed.')

_elapsed_ms = round((dt.now() - _start_time ).total_seconds() * 1000.0)
_log.info(Fore.YELLOW + 'complete: elapsed: {:d}ms'.format(_elapsed_ms))

#EOF
