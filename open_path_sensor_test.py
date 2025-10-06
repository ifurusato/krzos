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
from hardware.open_path_sensor import OpenPathSensor
from hardware.open_path_visualiser import OpenPathVisualiser

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def main():
    _log = Logger('main', level=Level.INFO)
    _config = ConfigLoader(Level.INFO).configure()

    _vl53_sensor = Vl53l5cxSensor(_config, skip=('skip' in sys.argv or True in sys.argv), level=Level.INFO)
    _visualiser = OpenPathVisualiser(cols=8, rows=8)
#   _visualiser = None
    _open_path_sensor = OpenPathSensor(_config, vl53l5cx=_vl53_sensor, visualiser=_visualiser, level=Level.INFO)
    _open_path_sensor.enable()

    try:
        while True:
            time.sleep(0.5)  # Let the sensor/visualiser thread run
    except KeyboardInterrupt:
        _log.info('caught Ctrl-C; exiting…')
    finally:
        _open_path_sensor.close()
        _log.info('complete.')

if __name__== "__main__":
    main()

#EOF
