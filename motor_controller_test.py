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
from math import isclose
from colorama import init, Fore, Style
init()

from core.orientation import Orientation
from core.rate import Rate
from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.irq_clock import IrqClock
from hardware.motor_controller import MotorController
from hardware.digital_pot import DigitalPotentiometer
from hardware.system import System

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_orientation = Orientation.SFWD # which motor to test?
_pot         = None
_start_time  = dt.now()

_log = Logger('test', Level.INFO)
_motor_controller = None
ENABLE_MOTORS = True

try:

    # read YAML configuration
    _level = Level.INFO
    _config = ConfigLoader(Level.INFO).configure()
    _system = System(_config)

    # external clock ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    _log.info('creating IRQ clock…')
    _irq_clock = IrqClock(_config, level=Level.INFO)
    _irq_clock.enable()

    _log.info('using digital potentiometer for speed…')
    _pot = DigitalPotentiometer(_config, level=_level)
    _pot.set_output_range(-1.0, 1.0)
#   _pot.set_output_range(-0.8, 0.8)

    _log.info('starting motor controller…')
    _motor_controller = MotorController(_config, external_clock=_irq_clock, level=_level)
    _motor_controller.set_closed_loop(False)
    _motor_controller.enable()

    _log.info('starting test…')
    _hz = 2
    _rate = Rate(_hz, Level.ERROR)

#   _test_orientation = Orientation.ALL
    _test_orientation = Orientation.PAFT

    while True:

        _current = _system.get_system_current()
        _target_speed = _pot.get_scaled_value(False) # values 0.0-1.0
        if isclose(_target_speed, 0.0, abs_tol=0.08):
            _pot.set_black() # only on digital pot
#           if ENABLE_MOTORS:
            if _test_orientation is Orientation.ALL:
                _motor_controller.set_speed(Orientation.PORT, 0.0)
                _motor_controller.set_speed(Orientation.STBD, 0.0)
            else:
                _motor_controller.set_speed(_test_orientation, 0.0)
            _log.info(Style.DIM + 'target speed: {:.2f}; current: {:4.2f}A'.format(_target_speed, _current))
        else:
#           _target_speed = 0.1
            _pot.set_rgb(_pot.value) # only on digital pot
            if ENABLE_MOTORS:
                if _test_orientation is Orientation.ALL:
                    _motor_controller.set_speed(Orientation.PORT, _target_speed)
                    _motor_controller.set_speed(Orientation.STBD, _target_speed)
                else:
                    _motor_controller.set_speed(_test_orientation, _target_speed)
            _log.info('target speed: {:.2f}; current: {:4.2f}A'.format(_target_speed, _current))

        _rate.wait()

except KeyboardInterrupt:
    _log.info('Ctrl-C caught; exiting…')
except Exception as e:
    _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    if _motor_controller:
        _motor_controller.emergency_stop()
    if _pot:
        _pot.close()

_elapsed_ms = round(( dt.now() - _start_time ).total_seconds() * 1000.0)
_log.info(Fore.YELLOW + 'complete: elapsed: {:d}ms'.format(_elapsed_ms))

#EOF
