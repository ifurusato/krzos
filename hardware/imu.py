#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-12-19
# modified: 2025-12-19

import math
from math import pi as π
from math import isclose
from collections import deque
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
        self._verbose                 = _cfg.get('verbose')
        self._yaw_match_threshold     = _cfg.get('yaw_match_threshold')
        self._prefer_usfs             = _cfg.get('prefer_usfs') # prefer USFS for accel/gyro
        self._heading_matches         = False
        # dynamic trim configuration
        self._enable_dynamic_trim     = _cfg.get('enable_dynamic_trim')
        self._min_stability_threshold = _cfg.get('min_stability_threshold')
        self._trim_adjustment_gain    = _cfg.get('trim_adjustment_gain')
        self._min_error_threshold     = _cfg.get('min_error_threshold')
        # dynamic trim state
        self._dynamic_yaw_offset = 0.0
        self._heading = 0
        self._radians = 0.0
        self._heading_stability_score = 0.0
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
            self._usfs = Usfs(config, matrix11x7=_matrix11x7, level=Level.INFO)
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
    def heading_matches(self):
        return self._heading_matches

    @property
    def dynamic_yaw_offset(self):
        '''
        return the current dynamic yaw offset in radians. 
        '''
        return self._dynamic_yaw_offset

    @property
    def heading_stability_score(self):
        '''
        return normalized stability score (0.0-1.0) based on individual stdevs
        and heading agreement.  1.0 = perfect (both stable, perfect agreement),
        0.0 = terrible (both unstable, large disagreement).
        '''
        return self._heading_stability_score

    @property
    def heading(self):
        '''
        Return the fused compass heading in degrees (as an int).
        '''
        return self._heading

    @property
    def heading_radians(self):
        '''
        Return the fused compass heading in radians.
        '''
        return self._radians

    def x_poll(self):
#       self._log.info(Style.DIM + 'polling…')
        self._usfs.poll()
        self._icm20948.poll()
#       self._log.info(Style.DIM + 'polled.')

    def poll(self):
        self._usfs.poll()
        self._icm20948.poll()
        # calculate heading error with circular wrapping
        _usfs_yaw_rad = self._usfs.yaw_radians
        _icm_yaw_rad = self._icm20948.heading_radians
        _error = (_usfs_yaw_rad - _icm_yaw_rad + π) % (2 * π) - π
        # calculate stability score
        _usfs_stdev = self._usfs.standard_deviation
        _icm_stdev = self._icm20948.standard_deviation
        # individual stability scores (inverse of stdev, clamped)
        _max_stdev = 0.5  # radians, ~28° - beyond this is considered completely unstable
        _usfs_stability = max(0.0, 1.0 - (_usfs_stdev / _max_stdev))
        _icm_stability = max(0.0, 1.0 - (_icm_stdev / _max_stdev))
        # agreement score based on heading error
        _max_error = π / 4  # 45° - beyond this is considered complete disagreement
        _agreement = max(0.0, 1.0 - (abs(_error) / _max_error))
        # combined score: geometric mean of all three factors
        self._heading_stability_score = (_usfs_stability * _icm_stability * _agreement) ** (1/3)

        # debug logging
        self._log. info(Style.DIM + 
            'enable_dynamic_trim: {}; '. format(self._enable_dynamic_trim) +
            'usfs_cal: {}; icm_cal:  {}; '.format(self._usfs.is_calibrated, self._icm20948.is_calibrated) +
            'error: {:.4f} ({:.2f}°) > min:  {:.4f}? ; '.format(abs(_error), math.degrees(abs(_error)), self._min_error_threshold) +
            'usfs_stdev: {:.4f}; icm_stdev: {:.4f}; diff: {:.4f} > 0.01?; '.format(_usfs_stdev, _icm_stdev, abs(_usfs_stdev - _icm_stdev)) +
            'usfs < thresh: {}; icm < thresh: {}'.format(_usfs_stdev < self._min_stability_threshold, _icm_stdev < self._min_stability_threshold)
        )

        # dynamic trim adjustment
        if self._enable_dynamic_trim:
            # check guard conditions
            if (self._usfs.is_calibrated and self._icm20948.is_calibrated
                    and abs(_error) > self._min_error_threshold):
                # determine which IMU is more stable
                _stdev_diff = abs(_usfs_stdev - _icm_stdev)
                # only adjust if there's a clear stability difference
                if _stdev_diff > 0.01: # ~0.6° difference threshold
                    if _usfs_stdev < _icm_stdev and _usfs_stdev < self._min_stability_threshold:
                        # usfs is more stable, adjust icm20948 toward it
                        _adjustment = _error * self._trim_adjustment_gain
                        self._dynamic_yaw_offset += _adjustment
                        if self._verbose:
                            self._log.info('adjusting ICM20948 trim by {:+.4f} rad ({:+.2f}°)'.format(
                                _adjustment, math.degrees(_adjustment)))
                    elif _icm_stdev < _usfs_stdev and _icm_stdev < self._min_stability_threshold:
                        # icm20948 is more stable, adjust usfs toward it (reverse error sign)
                        _adjustment = -_error * self._trim_adjustment_gain
                        _usfs_trim_rad = self._usfs.yaw_trim_radians + _adjustment
                        self._usfs.set_fixed_yaw_trim(math.degrees(_usfs_trim_rad))
                        if self._verbose:
                            self._log.info('adjusting USFS trim by {:+.4f} rad ({:+.2f}°)'.format(
                                _adjustment, math.degrees(_adjustment)))
        # calculate fused heading
        _icm_adjusted = _icm_yaw_rad + self._dynamic_yaw_offset
        # normalize to 0-2π
        if _icm_adjusted < 0:
            _icm_adjusted += 2 * π
        elif _icm_adjusted >= 2 * π:
            _icm_adjusted -= 2 * π
        # simple average for now (could weight by stability in future)
        self._radians = (_usfs_yaw_rad + _icm_adjusted) / 2.0
        self._heading = int(round(math.degrees(self._radians)))

    def show_info(self):
        # pitch ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        usfs_pitch       = self._usfs.pitch
        icm20948_pitch   = self._icm20948.pitch
        if isclose(usfs_pitch, icm20948_pitch, abs_tol=2.00):
            pitch_display = (usfs_pitch + icm20948_pitch) / 2
            pitch_str = '{:7.2f}          '.format(pitch_display)
        else:
            pitch_str = Style.DIM + '{:7.2f} | {:7.2f}'.format(usfs_pitch, icm20948_pitch)
        # roll ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        usfs_roll        = self._usfs.roll
        icm20948_roll    = self._icm20948.roll
        if isclose(usfs_roll, icm20948_roll, abs_tol=2.00):
            roll_display = (usfs_roll + icm20948_roll) / 2
            roll_str = '{:7.2f}          '.format(roll_display)
        else:
            roll_str = Style.DIM + '{:7.2f} | {:7.2f}'.format(usfs_roll, icm20948_roll)
        # heading ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        usfs_heading     = self._usfs.yaw
        icm20948_heading = self._icm20948.heading
        self._heading_matches = isclose(usfs_heading, icm20948_heading, abs_tol=5.0)
        if self._heading_matches:
            heading_display = (usfs_heading + icm20948_heading) / 2
            heading_str = '{:7.2f}          '.format(heading_display)
        else:
            heading_str = Style.DIM + '{:7.2f} | {:7.2f}'.format(usfs_heading, icm20948_heading)
        # trim ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        if self._usfs.is_adjusting_trim():
            trim_str = '; trim: {:7.2f}; '.format(self._usfs.trim_adjust)
        else:
            trim_str = ';              ; '
        # stability ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _stability_str = 'stability: {:3.2f}; '.format(self.heading_stability_score)
        # calibrated? ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _usfs_is_calibrated     = self._usfs.is_calibrated
        _icm20948_is_calibrated = self._icm20948.is_calibrated
        _calib_str = ' calib? {} {}'.format('+' if _usfs_is_calibrated else '-', '+' if _icm20948_is_calibrated else '-')
        # log output
        self._log.info(
              Fore.RED     + 'pitch: ' + pitch_str + '; '
            + Fore.GREEN   + Style.NORMAL + 'roll: ' + roll_str + '; '
            + Fore.BLUE    + Style.NORMAL + 'heading: ' + heading_str
            + Fore.YELLOW  + Style.NORMAL + trim_str
            + Fore.MAGENTA + Style.NORMAL + _stability_str
            + Fore.CYAN    + _calib_str
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
