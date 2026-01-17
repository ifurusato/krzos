#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2024-05-20
# modified: 2025-12-19

import traceback
from math import pi as π
import itertools
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from core.rate import Rate
from core.rdof import RDoF
from hardware.digital_pot import DigitalPotentiometer

from hardware.icm20948 import Icm20948
from hardware.usfs import Usfs
from hardware.imu import IMU
from hardware.button import Button

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

CALIBRATE = True
icm20948  = None
pot       = None
log = Logger('test', Level.INFO)

try:
    # read YAML configuration
    config = ConfigLoader(Level.INFO).configure()
    counter = itertools.count()

    button = Button(config=config, name='button', level=Level.INFO)

    log.info('using digital pot.')
    pot = DigitalPotentiometer(config, level=Level.INFO)
    pot.set_output_range(-π, π) # ±180° adjustment range

    # ICM20948 ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    icm20948 = Icm20948(config, level=Level.INFO)
    icm20948._show_console = True
    icm20948.include_accel_gyro(True)
    icm20948.enable()
    if CALIBRATE:
        if not icm20948.is_calibrated:
            icm20948.bench_calibrate()

    # USFS ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    _usfs = Usfs(config, level=Level.INFO)
#   _usfs.set_fixed_yaw_trim(-72.5) # TODO config
#   _usfs.adjust_trim(RDoF.YAW)
    _usfs.set_verbose(False)

    # IMU ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    imu = IMU(config, level=Level.INFO)
    button.add_callback(lambda: imu.align(RDoF.YAW))
    imu.enable()
    log.info(Fore.GREEN + 'ready.')

    poll_rate_hz = 10
    rate = Rate(poll_rate_hz, Level.ERROR)

#   for i in range(10):
    while True:
        imu.poll()
        if next(counter) % 10 == 0:
            imu.show_info()
        rate.wait()

except KeyboardInterrupt:
    log.info('Ctrl-C caught; exiting…')
except Exception as e:
    log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    if pot:
        pot.close()

#EOF
