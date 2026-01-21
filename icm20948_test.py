#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2024-05-20
# modified: 2024-10-24
#

import sys, time, traceback
from math import pi as π
from colorama import init, Fore, Style
init()

from core.cardinal import Cardinal
from core.convert import Convert
from core.orientation import Orientation
from core.rdof import RDoF
from core.logger import Logger, Level
from core.config_loader import ConfigLoader
#from hardware.icm20948 import Icm20948
from hardware.icm20948_s import Icm20948
#from hardware.icm20948_o import Icm20948
from hardware.digital_pot import DigitalPotentiometer # for calibration only

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

CALIBRATE       = True
HEADING_TEST    = False
ACCEL_GYRO_TEST = True

HALF_PI = π / 2.0

_trim_axis = None #RDoF.YAW
#_trim_axis = RDoF.ROLL
_icm20948 = None
_log = Logger('test', Level.INFO)
_cardinal = Cardinal.NORTH
_threshold = 4
_pot = None
_i2c_address = 0x0D

try:
    # read YAML configuration
    _config = ConfigLoader(Level.INFO).configure()

    _log.info('using digital pot.')
    _pot = DigitalPotentiometer(_config, i2c_address=_i2c_address, level=Level.INFO)

    if _pot:
        if _trim_axis:
            if _trim_axis == RDoF.YAW:
                _pot.set_output_range(-π, π) # ±180° adjustment range
#               _pot.set_output_range(-π/2.0, π/2.0) # ±90° adjustment range
#               _pot.set_output_range(-π/4.0, π/4.0) # ±45° adjustment range
            else:
#               _pot.set_output_range(-0.5 * π, 0.5 * π)  # ±90° adjustment range
#               _pot.set_output_range(-0.17453293, 0.17453293) # ±10° adjustment range
#               _pot.set_output_range(-0.08726646, 0.08726646) # ±5° adjustment range
#               _pot.set_output_range(-0.05235988, 0.05235988) # ±3° adjustment range
                _pot.set_output_range(-0.03490659, 0.03490659) # ±2° adjustment range

    _icm20948 = Icm20948(_config, level=Level.INFO)
    time.sleep(1)
    _icm20948._show_console = True
    if _trim_axis:
        _icm20948.adjust_trim(_trim_axis)
    _icm20948.enable()
    if CALIBRATE:
        if not _icm20948.is_calibrated:
            _icm20948.bench_calibrate()
    # just scan continually
    _icm20948.scan(enabled=True, callback=None)

except KeyboardInterrupt:
    _log.info('Ctrl-C caught; exiting…')
except Exception as e:
    _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    if _pot:
        _pot.close()

#EOF
