#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2020-10-26
# modified: 2026-02-05
#
# 10 runs, average 1228 steps over 200mm, so 614 steps per 100mm or 6.14 steps/mm

import sys
import time
import traceback
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.ofs import OpticalFlowSensor

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('test', Level.INFO)

def main():

    ofs = None

    try:
        _log.info('initialising…')

        _config = ConfigLoader(Level.INFO).configure()
        ofs = OpticalFlowSensor(_config, level=Level.INFO)
  
        _log.info("""Detect flow/motion in front of the optical flow sensor.

    Press Ctrl+C to exit!
    """)

        x_max_mm = 0.0
        y_max_mm = 0.0

        _log.info(Fore.GREEN + 'starting ofs polling thread…')
        ofs.start()

        while True:
            try:

                x, y = ofs.absolute()
                x_mm, y_mm = ofs.millimeters()

                x_max_mm = max(abs(x_mm), x_max_mm)
                y_max_mm = max(abs(y_mm), y_max_mm)
                perc = ofs.y_variance()
                _log.info("absolute: x {:03d} y {:03d}; ".format(x, y) 
                        + Fore.WHITE + ' {:d}%; '.format(perc)
                        + Fore.GREEN + "dist: x {:03d}mm, y {:03d}mm; ".format(x_mm, y_mm)
                        + Style.DIM + "max: x {:d}mm, y {:d}mm; ".format(int(x_max_mm), int(y_max_mm)))

                time.sleep(1.0)

            except RuntimeError as re:
                _log.error('runtime error in loop: {}\n{}'.format(re, traceback.format_exc()))
                continue

    except KeyboardInterrupt:
        print('\n')
        _log.info('caught Ctrl-C: exiting…')
    except Exception as e:
        _log.error('{} thrown executing ofs test: {}\n{}'.format(type(e), e, traceback.format_exc()))
    finally:
        if ofs:
            ofs.close()

if __name__ == "__main__":
    main()

#EOF
