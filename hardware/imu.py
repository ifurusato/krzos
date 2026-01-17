#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-12-19
# modified: 2026-01-18

import math
from math import pi as π
from math import isclose
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level
from core.rate_limited import rate_limited
from hardware.numeric_display import NumericDisplay
from hardware.usfs import Usfs
from hardware.icm20948 import Icm20948

class IMU(Component):
    NAME = 'imu'
    USFS     = 0
    ICM20948 = 1
    '''
    A 'fusion' IMU class that uses USFS as primary heading source with an
    ICM20948 as a validator for detecting anomalies via rate-of-change comparison.

    Usage:

        # general navigation while moving (roam, avoid, etc.)
        heading = imu.yaw # always available when calibrated

        # precision rotation (rotate exactly a specified amount)
        stable_heading = imu.yaw_stable # only when stable
        if stable_heading is not None:
            # proceed with precise rotation
        else:
            # wait for stability or use approximate heading

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
        self._verbose                      = _cfg.get('verbose')
        self._rate_divergence_threshold    = math.radians(_cfg.get('rate_divergence_threshold_deg', 15.0))
        self._min_stability_threshold      = _cfg.get('min_stability_threshold')
        self._rotation_threshold           = math.radians(_cfg.get('rotation_threshold_deg', 1.0))
        # state
        self._usfs_prev_yaw                = None
        self._icm_prev_yaw                 = None
        self._usfs_yaw_rate                = 0.0
        self._icm_yaw_rate                 = 0.0
        self._was_rotating                 = False
        self._is_stable                    = False
        self._yaw                          = None
        self._yaw_radians                  = None
        # components
        _component_registry = Component.get_registry()
        # ICM20948 ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._icm20948 = icm20948 if icm20948 else _component_registry.get(Icm20948.NAME)
        if not self._icm20948:
            self._icm20948 = Icm20948(config, level=Level.INFO)
            self._icm20948.include_accel_gyro(True)
        # USFS ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._usfs = usfs if usfs else _component_registry.get(Usfs.NAME)
        if not self._usfs:
            self._usfs = Usfs(config, level=Level.INFO)
        # always use USFS as primary
        self._usfs.enable_matrix11x7(True)
        self._icm20948.enable_matrix11x7(False)
        self._numeric_display = self._usfs.numeric_display
        self._log.info('ready.')

    @rate_limited(500) # 500 ms between calls
    def _change_callback(self, value):
        if value:
            self._log.info(Fore.WHITE + '🍏 USFS IS STABLE transition callback: {}'.format(value))
        else:
            self._log.info(Fore.WHITE + '🍎 USFS IS NOT STABLE transition callback: {}'.format(value))

    # properties ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def pitch(self):
        '''
        return the last-polled pitch value from USFS.
        '''
        return self._usfs.pitch

    @property
    def roll(self):
        '''
        return the last-polled roll value from USFS.
        '''
        return self._usfs.roll

    @property
    def yaw(self):
        '''
        return the last-polled yaw (compass heading) in degrees from USFS.
        always returns a value if USFS is calibrated, regardless of stability.
        use this for general navigation while moving.
        returns None if USFS is not calibrated.
        '''
        return self._yaw

    @property
    def yaw_radians(self):
        '''
        return the last-polled yaw (compass heading) in radians from USFS.
        always returns a value if USFS is calibrated, regardless of stability.
        returns None if USFS is not calibrated.
        '''
        return self._yaw_radians

    @property
    def yaw_stable(self):
        '''
        return yaw only if USFS is both calibrated AND stable.
        use this for precision maneuvers like exact rotations.
        returns None if unstable or not calibrated.
        '''
        if self._usfs.is_calibrated and self._is_stable:
            return self._yaw
        return None

    @property
    def yaw_stable_radians(self):
        '''
        return yaw in radians only if USFS is both calibrated AND stable.
        returns None if unstable or not calibrated.
        '''
        if self._usfs.is_calibrated and self._is_stable:
            return self._yaw_radians
        return None

    @property
    def is_calibrated(self):
        '''
        returns a tuple containing calibration state for each IMU: (USFS, ICM20948)
        '''
        return self._usfs.is_calibrated, self._icm20948.is_calibrated

    @property
    def is_stable(self):
        '''
        returns True if USFS standard deviation is below stability threshold.
        '''
        return self._is_stable

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def poll(self):
        '''
        poll both IMUs, use USFS for heading, validate with ICM20948 rate-of-change.
        '''
        # 1. poll both sensors
        self._usfs.poll()
        self._icm20948.poll()

        # 2. get current yaw values
        _usfs_yaw_rad = self._usfs.yaw_radians
        _icm_yaw_rad = self._icm20948.yaw_radians

        # 3. rate-of-change validation and rotation detection
        if self._usfs_prev_yaw is not None and self._icm_prev_yaw is not None:
            # calculate angular change since last poll
            _usfs_delta = (_usfs_yaw_rad - self._usfs_prev_yaw + π) % (2 * π) - π
            _icm_delta = (_icm_yaw_rad - self._icm_prev_yaw + π) % (2 * π) - π

            self._usfs_yaw_rate = _usfs_delta
            self._icm_yaw_rate = _icm_delta

            # detect when rotation stops - clear queue for fast stability recovery
            if abs(_usfs_delta) < self._rotation_threshold and self._was_rotating:
                self._usfs.clear_queue()
                if self._verbose:
                    self._log.debug('rotation stopped, queue cleared for fast stability recovery')
                self._was_rotating = False
            elif abs(_usfs_delta) >= self._rotation_threshold:
                self._was_rotating = True

            # check if rotation rates diverge significantly
            _rate_divergence = abs(_usfs_delta - _icm_delta)
            if _rate_divergence > self._rate_divergence_threshold:
                self._log.warning('IMU rate divergence: USFS Δ={:+.2f}°, ICM Δ={:+.2f}°, diff={:.2f}°'.format(
                    math.degrees(_usfs_delta), math.degrees(_icm_delta), math.degrees(_rate_divergence)))

        # 4. store current values for next rate calculation
        self._usfs_prev_yaw = _usfs_yaw_rad
        self._icm_prev_yaw = _icm_yaw_rad

        # 5. use USFS yaw if calibrated (always available for general use)
        if self._usfs.is_calibrated:
            self._yaw_radians = _usfs_yaw_rad
            self._yaw = int(round(math.degrees(self._yaw_radians)))

            # check stability (for precision operations)
            _usfs_stdev = self._usfs.standard_deviation
            _is_stable = _usfs_stdev < self._min_stability_threshold

            if _is_stable != self._is_stable:
                self._is_stable = _is_stable
                self._change_callback(self._is_stable)
        else:
            self._yaw_radians = None
            self._yaw = None

    def show_info(self):
        '''
        display pitch, roll, yaw, stability and calibration status for both IMUs.
        '''
        # pitch ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        usfs_pitch = self._usfs.pitch
        icm20948_pitch = self._icm20948.pitch
        if isclose(usfs_pitch, icm20948_pitch, abs_tol=2.0):
            pitch_display = (usfs_pitch + icm20948_pitch) / 2
            pitch_str = '{:7.2f}          '.format(pitch_display)
        else:
            pitch_str = Style.DIM + '{:7.2f} | {:7.2f}'.format(usfs_pitch, icm20948_pitch)

        # roll ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        usfs_roll = self._usfs.roll
        icm20948_roll = self._icm20948.roll
        if isclose(usfs_roll, icm20948_roll, abs_tol=2.0):
            roll_display = (usfs_roll + icm20948_roll) / 2
            roll_str = '{:7.2f}          '.format(roll_display)
        else:
            roll_str = Style.DIM + '{:7.2f} | {:7.2f}'.format(usfs_roll, icm20948_roll)

        # yaw ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        usfs_yaw = self._usfs.yaw
        icm20948_yaw = self._icm20948.yaw
        # always show USFS brightly (it's the primary), ICM dimmed
        yaw_str = Style.BRIGHT + '{:7.2f}'.format(usfs_yaw) + Style.DIM + ' | {:7.2f}'.format(icm20948_yaw)

        # stability ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _usfs_stdev = self._usfs.standard_deviation
        _icm_stdev = self._icm20948.standard_deviation
        if self._is_stable:
            _stability_str = Style.BRIGHT + 'stdev: {:4.2f} | {:4.2f}; '.format(_usfs_stdev, _icm_stdev) + Style.NORMAL
            if self._numeric_display:
                self._numeric_display.set_brightness(NumericDisplay.HIGH_BRIGHTNESS)
        else:
            _stability_str = 'stdev: {:4.2f} | {:4.2f}; '.format(_usfs_stdev, _icm_stdev)
            if self._numeric_display:
                self._numeric_display.set_brightness(NumericDisplay.LOW_BRIGHTNESS)

        # calibrated?   ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _usfs_is_calibrated, _icm20948_is_calibrated = self.is_calibrated
        _calib_str = ' calib?   {} {}'.format(
            '+' if _usfs_is_calibrated else '-',
            '+' if _icm20948_is_calibrated else '-')

        # log output
        self._log.info(
              Fore.RED     + 'pitch: ' + pitch_str + '; '
            + Fore.GREEN   + Style.NORMAL + 'roll: ' + roll_str + '; '
            + Fore.BLUE    + Style.NORMAL + 'yaw (u|i): ' + yaw_str + '; '
            + Fore.MAGENTA + Style.NORMAL + _stability_str
            + Fore.CYAN    + _calib_str
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

#EOF
