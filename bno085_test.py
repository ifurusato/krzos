#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato.  All rights reserved.  This file is part
# of the Robot Operating System project, released under the MIT License.  Please
# see the LICENSE file included as part of this package. 
#
# author:    Ichiro Furusato
# created:  2026-01-20

import time
from colorama import init, Fore, Style
init()

from hardware.bno085 import (
    BNO085,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.bno085 import BNO085

_log = Logger('bno085-test', level=Level.INFO)


bno = None

try:
    # read YAML configuration
    _config = ConfigLoader(Level.INFO).configure()

    _log.info('initializing BNO085…')
    bno = BNO085(_config, level=Level.INFO)
    bno.enable()
    
    _log.info('reading sensor data…')
    while True:

        # accelerometer:
        accel_x, accel_y, accel_z = bno.acceleration
        _log.info(Fore.YELLOW + 'accel:  X: {:4.2f}  Y: {:4.2f}  Z: {:4.2f} m/s^2'.format(accel_x, accel_y, accel_z))
        
        # gyroscope:
        gyro_x, gyro_y, gyro_z = bno.gyro
        _log. info(Fore.MAGENTA + 'gyro:  X: {:4.2f}  Y: {:4.2f}  Z: {:4.2f} rad/s'.format(gyro_x, gyro_y, gyro_z))
        
        # magnetometer:
        mag_x, mag_y, mag_z = bno.magnetic
        _log.info(Fore.CYAN + 'mag:  X: {:4.2f}  Y: {:4.2f}  Z: {:4.2f} uT'.format(mag_x, mag_y, mag_z))
        
        # rotation vector quaternion:
        quat_i, quat_j, quat_k, quat_real = bno.quaternion
        _log.info(Fore.GREEN + 'rvq:  I: {:4.2f}  J: {:4.2f}  K: {:4.2f}  Real: {:4.2f}\n'.format(quat_i, quat_j, quat_k, quat_real))
#       _log.info('')

        time.sleep(0.5)

except KeyboardInterrupt:
    _log.info('Ctrl-C caught; exiting…')
except Exception as e: 
    _log.error('error during test: {}'.format(e))
    import traceback
    traceback.print_exc()
finally:
    _log.info('closing BNO085…')
    if bno:
        bno.close()
    _log.info('test complete.')

#EOF
