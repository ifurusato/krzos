#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-12-19
# modified: 2025-12-19

import math
from math import isclose
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level
from hardware.usfs import Usfs
from hardware.icm20948 import Icm20948
from hardware.usfs import Usfs

class IMU(Component):
    NAME = 'imu'
    '''
    A fusion of two IMUs. Instances of the Icm20948 and Usfs classes can be provided,
    obtained from the ComponentRegistry, or they will be created on the fly, though
    in the latter case there is no calibration management.

    Args:
        config:    the application configuration
        icm20948:  optional instance of Icm20948
        usfs:      optional instance of Usfs
        level:     the log level
    '''
    def __init__(self, config, icm20948=None, usfs=None, level=Level.INFO):
        self._log = Logger(IMU.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._log.info('initialising imu…')
        if not isinstance(config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(name)))
        # configuration
        _cfg = config['kros'].get('hardware').get('imu')
        self._verbose             = _cfg.get('verbose')
        self._yaw_match_threshold = _cfg.get('yaw_match_threshold')
        self._prefer_usfs         = _cfg.get('prefer_usfs') # prefer USFS for accel/gyro
        # components
        _component_registry = Component.get_registry()
        # ICM20948 ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._icm20948 = icm20948 if icm20948 else _component_registry.get(Icm20948.NAME)
        if not self._icm20948:
#           raise MissingComponentError('icm20948 required for IMU.')
            self._icm20948 = Icm20948(config, level=Level.INFO)
            self._icm20948.include_accel_gyro(True)
        # USFS ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._usfs = usfs if usfs else _component_registry.get(Usfs.NAME)
        if not self._usfs:
#           raise MissingComponentError('USFS required for IMU.')
            from matrix11x7 import Matrix11x7

            _low_brightness    = 0.15
            _medium_brightness = 0.25
            _high_brightness   = 0.45
            _matrix11x7 = Matrix11x7()
            _matrix11x7.set_brightness(_medium_brightness)
            _pot = None
            self._usfs = Usfs(config, matrix11x7=_matrix11x7, trim_pot=_pot, level=Level.INFO)
            # fixed trim for Pukerua Bay, NZ, determined via observation
            self._usfs.set_fixed_yaw_trim(-77.41)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def pitch(self):
        '''
        Return the last-polled pitch value.
        '''
        if self._prefer_usfs:
            return self._usfs.pitch
        else:
            return self._icm20948.pitch

    @property
    def roll(self):
        '''
        Return the last-polled roll value.
        '''
        if self._prefer_usfs:
            return self._usfs.roll
        else:
            return self._icm20948.roll

    @property
    def heading(self):
        '''
        Return the fused compass heading in degrees (as an int).
        '''
        return self._heading

    def heading_radians(self):
        '''
        Return the fused compass heading in radians.
        '''
        return self._radians

    def poll(self):
#       self._log.info(Style.DIM + 'polling…')
        self._usfs.poll()
        self._icm20948.poll()
#       self._log.info(Style.DIM + 'polled.')

    def show_info(self):
        # pitch ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        usfs_pitch       = self._usfs.pitch
        icm20948_pitch   = self._icm20948.pitch
        if isclose(usfs_pitch, icm20948_pitch, abs_tol=1.00):
            pitch_display = (usfs_pitch + icm20948_pitch) / 2
            pitch_str = '{:4.2f}'.format(pitch_display)
        else:
            pitch_str = '{:4.2f} | {:4.2f}'.format(usfs_pitch, icm20948_pitch)
        # roll ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        usfs_roll        = self._usfs.roll
        icm20948_roll    = self._icm20948.roll
        if isclose(usfs_roll, icm20948_roll, abs_tol=1.00):
            roll_display = (usfs_roll + icm20948_roll) / 2
            roll_str = '{:4.2f}'.format(roll_display)
        else:
            roll_str = '{:4.2f} | {:4.2f}'.format(usfs_roll, icm20948_roll)
        # heading ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        usfs_heading     = self._usfs.yaw
        icm20948_heading = self._icm20948.heading
        if isclose(usfs_heading, icm20948_heading, abs_tol=5.0):
            heading_display = (usfs_heading + icm20948_heading) / 2
            heading_str = '{:4.2f}'.format(heading_display)
        else:
            heading_str = '{:4.2f} | {:4.2f}'.format(usfs_heading, icm20948_heading)
        # log output
        self._log.info(
              Fore.YELLOW + 'pitch: ' + pitch_str + '; '
            + Fore.WHITE  + 'roll: ' + roll_str + '; '
            + Fore.GREEN  + 'heading: ' + heading_str
        )

    def x_show_info(self):
        # pitch ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        usfs_pitch       = self._usfs.pitch
        icm20948_pitch   = self._icm20948.pitch
        if isclose(usfs_pitch, icm20948_pitch, abs_tol=0.20):
            pass
        # roll ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        usfs_roll        = self._usfs.roll
        icm20948_roll    = self._icm20948.roll # in degrees
        if isclose(usfs_roll, icm20948_roll, abs_tol=0.20):
            pass
        # heading ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        usfs_heading     = self._usfs.yaw
        icm20948_heading = self._icm20948.heading
        if isclose(usfs_heading, icm20948_heading, abs_tol=3.0):
            pass
        self._log.info(
                Fore.YELLOW + 'pitch: {:4.2f} | {:4.2f}; '.format(usfs_pitch, icm20948_pitch)
              + Fore.WHITE  + 'roll: {:4.2f} | {:4.2f}; '.format(usfs_roll, icm20948_roll)     
              + Fore.GREEN  + 'heading: {:4.2f} | {:4.2f}'.format(usfs_heading, icm20948_heading)     
        )

    def enable(self):
        if not self.enabled:
            Component.enable(self)
            if not self._icm20948.enabled:
                self._icm20948.enable()
            if not self._usfs.enabled:
                self._usfs.enable()
            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def disable(self):
        if self.enabled:
            Component.disable(self)
            if self._icm20948.enabled:
                self._icm20948.disable()
            if self._usfs.enabled:
                self._usfs.disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

#EOF
