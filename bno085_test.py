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

_log = Logger('bno085-test', level=Level.INFO)

try:
    _log.info('initializing BNO085 on I2C bus 1 at default address 0x4A…')
    bno = BNO085(i2c_bus_number=1, level=Level.INFO)
    
    _log.info('enabling sensor reports…')
    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    bno.enable_feature(BNO_REPORT_MAGNETOMETER)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    
    _log.info('reading sensor data…')
    while True:

        # accelerometer:
        accel_x, accel_y, accel_z = bno.acceleration
        _log.info(Fore.YELLOW + 'accel:  X: {:.6f}  Y: {:.6f}  Z: {:.6f} m/s^2'.format(accel_x, accel_y, accel_z))
        
        # gyroscope:
        gyro_x, gyro_y, gyro_z = bno.gyro
        _log. info(Fore.MAGENTA + 'gyro:  X: {:.6f}  Y: {:.6f}  Z: {:.6f} rad/s'.format(gyro_x, gyro_y, gyro_z))
        
        # magnetometer:
        mag_x, mag_y, mag_z = bno.magnetic
        _log.info(Fore.CYAN + 'mag:  X: {:.6f}  Y: {:.6f}  Z: {:.6f} uT'.format(mag_x, mag_y, mag_z))
        
        # rotation vector quaternion:
        quat_i, quat_j, quat_k, quat_real = bno.quaternion
        _log.info(Fore.GREEN + 'rvq:  I: {:.6f}  J: {:.6f}  K: {:.6f}  Real: {:.6f}\n'.format(quat_i, quat_j, quat_k, quat_real))
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
    bno.close()
    _log.info('test complete.')

#EOF
