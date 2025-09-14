#!/usr/bin/env python3

# -*- coding: utf-8 -*-
#
# Portions copyright 2020-2024 by Murray Altheim. All rights reserved. This file
# is part of the Robot Operating System project, released under the MIT License.
# Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-09-03
# modified: 2024-09-04
#

import sys, traceback
import time
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from matrix11x7 import Matrix11x7
from hardware.digital_pot import DigitalPotentiometer
from hardware.em7180 import Em7180

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    _em7180 = None
    _log = Logger('test', Level.INFO)
    _start_time = dt.now()
    I2C_ADDRESS = 0x0C # was 0x0E

    try:

        # read YAML configuration
        _config = ConfigLoader(Level.INFO).configure()
        _trim_pot = DigitalPotentiometer(_config, I2C_ADDRESS, level=Level.INFO)
        _trim_pot.set_output_range(-180.0, 180.0)

        _low_brightness    = 0.15
        _medium_brightness = 0.25
        _high_brightness   = 0.45

        _matrix11x7 = Matrix11x7()
        _matrix11x7.set_brightness(_medium_brightness)

        # set up ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        _em7180 = Em7180(_config, matrix11x7=_matrix11x7, trim_pot=_trim_pot, level=Level.INFO)
        # fixed trim for Pukerua Bay, NZ, determined via observation
#       _em7180.set_fixed_yaw_trim(148.0) 
        _em7180.set_fixed_yaw_trim(136.5) 
        _em7180.set_verbose(False)

        while True:
            _em7180.poll()
            _yaw = _em7180.yaw
            _corrected_yaw = _em7180.corrected_yaw
            _yaw_trim = _em7180.yaw_trim
            _log.info('Yaw: {:+2.2f} '.format(_yaw)
                    + Fore.WHITE + 'Corrected Yaw: {:+2.2f} '.format(_corrected_yaw)
                    + Style.DIM + 'with trim: {:+2.2f}'.format(_yaw_trim))
            time.sleep(.05)

    except KeyboardInterrupt:
        _log.info('Ctrl-C caught; exiting…')
    except Exception as e:
        _log.error('error in test: {}'.format(e))
        traceback.print_exc(file=sys.stdout)
    finally:
        if _em7180:
            _em7180.close()
        _elapsed_ms = round(( dt.now() - _start_time).total_seconds() * 1000.0)
        _log.info(Fore.YELLOW + 'complete: elapsed: {:d}ms'.format(_elapsed_ms))

if __name__== "__main__":
    main()

#EOF
