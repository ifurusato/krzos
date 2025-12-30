#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2020-10-05
# modified: 2025-09-22
#
# Stops all motors, immediately.
#

from core.orientation import Orientation
from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.motor_controller import MotorController

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('stop-motors', Level.INFO)
_motor_controller = None

try:
    _config = ConfigLoader(Level.INFO).configure()
    _log.info('starting motor controller…')
    _motor_controller = MotorController(_config, external_clock=None, level=Level.INFO)
    _motor_controller.emergency_stop()
except KeyboardInterrupt:
    _log.info('Ctrl-C caught; exiting…')
except Exception as e:
    _log.error('{} encountered, exiting: {}'.format(type(e), e))
finally:
    if _motor_controller:
        _motor_controller.emergency_stop()

_log.info('complete.')

#EOF
