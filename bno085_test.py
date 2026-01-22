#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato.  All rights reserved.  This file is part
# of the Robot Operating System project, released under the MIT License.  Please
# see the LICENSE file included as part of this package. 
#
# author:    Ichiro Furusato
# created:  2026-01-20
# modified: 2026-01-21

import time
from math import pi as π
import itertools
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
from core.rate import Rate
from core.rdof import RDoF
from hardware.bno085_imu import Bno085
from hardware.digital_pot import DigitalPotentiometer # for calibration only

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈


trim_axis = None
#trim_axis = RDoF.PITCH
#trim_axis = RDoF.ROLL
trim_axis = RDoF.YAW

log = Logger('bno085-test', level=Level.INFO)
bno = None

try:

    # read YAML configuration
    _config = ConfigLoader(Level.INFO).configure()

    _pot = DigitalPotentiometer(_config, level=Level.INFO)
    if _pot:
        log.info('configuring digital pot…')
        if trim_axis:
            if trim_axis == RDoF.YAW:
#               _pot.set_output_range(-π, π) # ±180° adjustment range
                _pot.set_output_range(-π/2.0, π/2.0) # ±90° adjustment range
#               _pot.set_output_range(-π/4.0, π/4.0) # ±45° adjustment range
            else:
#               _pot.set_output_range(-0.5 * π, 0.5 * π)  # ±90° adjustment range
#               _pot.set_output_range(-0.17453293, 0.17453293) # ±10° adjustment range
                _pot.set_output_range(-0.08726646, 0.08726646) # ±5° adjustment range
#               _pot.set_output_range(-0.05235988, 0.05235988) # ±3° adjustment range
#               _pot.set_output_range(-0.03490659, 0.03490659) # ±2° adjustment range

    log.info('initializing BNO085…')
    bno = Bno085(_config, level=Level.INFO)
    if trim_axis:
        bno.adjust_trim(trim_axis)
    bno.enable()
    
    poll_rate_hz = 2
    rate = Rate(poll_rate_hz, Level.ERROR)

    log.info('reading sensor data…')
    while True:

        bno.poll()

        # accelerometer:
#       accel_x, accel_y, accel_z = bno.acceleration
#       log.info(Fore.YELLOW + 'accel:  X: {:4.2f}  Y: {:4.2f}  Z: {:4.2f} m/s^2'.format(accel_x, accel_y, accel_z))
        
        # gyroscope:
#       gyro_x, gyro_y, gyro_z = bno.gyro
#       log. info(Fore.MAGENTA + 'gyro:  X: {:4.2f}  Y: {:4.2f}  Z: {:4.2f} rad/s'.format(gyro_x, gyro_y, gyro_z))
        
        # magnetometer:
#       mag_x, mag_y, mag_z = bno.magnetic
#       log.info(Fore.CYAN + 'mag:  X: {:4.2f}  Y: {:4.2f}  Z: {:4.2f} uT'.format(mag_x, mag_y, mag_z))
        
        # rotation vector quaternion:
#       quat_i, quat_j, quat_k, quat_real = bno.quaternion
#       log.info(Fore.GREEN + 'rvq:  I: {:4.2f}  J: {:4.2f}  K: {:4.2f}  Real: {:4.2f}'.format(quat_i, quat_j, quat_k, quat_real))
#       log.info('')

        # euler angles in degrees:
#       pitch = bno.pitch
#       roll  = bno.roll
#       yaw   = bno.yaw
#       log.info(Fore.WHITE + 'euler: pitch: {:8.4f}°; roll: {:8.4f}°; yaw: {:8.4f}°\n'.format(pitch, roll, yaw))

        counter = itertools.count()
        if next(counter) % 10 == 0:
            bno.show_info()

        rate.wait()

except KeyboardInterrupt:
    log.info('Ctrl-C caught; exiting…')
except Exception as e: 
    log.error('error during test: {}'.format(e))
    import traceback
    traceback.print_exc()
finally:
    log.info('closing BNO085…')
    if bno:
        bno.close()
    if _pot:
        _pot.close()
    log.info('test complete.')

#EOF
