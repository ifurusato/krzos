#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-26
# modified: 2024-10-31
#
# 10 runs, average 1228 steps over 200mm, so 614 steps per 100mm or 6.14 steps/mm

import sys
import traceback
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.orientation import Orientation
from core.config_loader import ConfigLoader
from hardware.i2c_scanner import I2CScanner, DeviceNotFound
from hardware.ofs import OpticalFlowSensor

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('test', Level.INFO)

DECAY_RATE = 5  # rate at which values decay towards zero (units per loop)
DECAY_INTERVAL = 0.1  # Time interval (seconds) to wait before applying decay
MAX_SENSOR_VALUE = 100  # maximum sensor value range

def decay_to_zero(value, decay_rate):
    '''
    Gradually reduce a value to zero by the decay_rate.
    '''
    if value > 0:
        return max(0, value - decay_rate)
    elif value < 0:
        return min(0, value + decay_rate)
    return 0

def accumulate_and_clamp(value, delta, max_value):
    '''
    Accumulate the value and clamp it to the range of -max_value to max_value.
    '''
    value += delta
    return max(-max_value, min(value, max_value))

def main():

    ofs = None

    try:
        print('🦊 a.')

        _config = ConfigLoader(Level.INFO).configure()
        ofs = OpticalFlowSensor(_config, level=Level.INFO)
        _matrix = None
        print('🦊 b.')
        _i2c_scanner = I2CScanner(_config, level=Level.INFO)
        _brightness = 0.7
        print('🦊 c.')
  
        _log.info("""Detect flow/motion in front of the PAA5100JE sensor.

    Press Ctrl+C to exit!
    """)

        print('🦊 e.')
        _swap_axes = True
        x_max_mm = 0.0
        y_max_mm = 0.0
        xd, yd = 0, 0  # Initialize display values
        last_move_time = dt.now().timestamp()  # Track the last time the robot moved

        print('🦊 f.')
        _log.info(Fore.GREEN + 'starting nofs loop…')
        ofs.start()

        print('🦊 g.')
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
                        + Style.DIM + "max: x {:d}mm, y {:d}mm; ".format(x_max_mm, y_max_mm))

                current_time = dt.now().timestamp()
     
                if x_mm != 0 or y_mm != 0:
                    # accumulate and clamp the sensor data
                    xd = accumulate_and_clamp(xd, x_mm, MAX_SENSOR_VALUE)
                    yd = accumulate_and_clamp(yd, y_mm, MAX_SENSOR_VALUE)
                    
                    # update the last move time when movement is detected
                    last_move_time = current_time
                else:
                    # apply decay if no movement has been detected for a while
                    if current_time - last_move_time > DECAY_INTERVAL:
                        xd = decay_to_zero(xd, DECAY_RATE)
                        yd = decay_to_zero(yd, DECAY_RATE)

            except RuntimeError as re:
                _log.error('runtime error in nofs loop: {}\n{}'.format(e, traceback.format_exc()))
                continue
            finally:
                time.sleep(1)

        print('🦊 h.')
    except KeyboardInterrupt:
        print('\n')
        _log.info('caught Ctrl-C: exiting…')
    except Exception as e:
        _log.error('{} thrown executing nofs: {}\n{}'.format(type(e), e, traceback.format_exc()))
    finally:
        if ofs:
            ofs.close()
            ofs.stop()
        pass

#if __name__ == "__main__":
main()

#EOF
