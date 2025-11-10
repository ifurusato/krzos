#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-02
# modified: 2025-10-03

import sys
import time
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.vl53l5cx_sensor import Vl53l5cxSensor
from hardware.scout_sensor import ScoutSensor
from hardware.scout_visualiser import ScoutVisualiser

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def main():
    _log = Logger('main', level=Level.INFO)
    _config = ConfigLoader(Level.INFO).configure()

    _vl53_config     = _config['kros']['hardware']['vl53l5cx_sensor']
#   _flip_horizontal = _vl53_config.get('flip_horizontal', False)
#   _flip_vertical   = _vl53_config.get('flip_vertical', False)

    _vl53_sensor = Vl53l5cxSensor(_config, skip=('skip' in sys.argv or True in sys.argv), level=Level.INFO)
    _visualiser = None
#   _visualiser = ScoutVisualiser()
#   _visualiser = ScoutVisualiser(cols=8, rows=8, flip_horizontal=_flip_horizontal, flip_vertical=_flip_vertical)
#   _visualiser = None
    _scout_sensor = ScoutSensor(_config, vl53l5cx=_vl53_sensor, visualiser=_visualiser, level=Level.INFO)
    _scout_sensor.enable()

    try:
        while True:
            time.sleep(0.5)  # Let the sensor/visualiser thread run
    except KeyboardInterrupt:
        _log.info('caught Ctrl-C; exiting…')
    finally:
        _scout_sensor.close()
        _log.info('complete.')

if __name__== "__main__":
    main()

#EOF
