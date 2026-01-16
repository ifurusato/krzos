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
from core.rate_limited import rate_limited
from hardware.numeric_display import NumericDisplay
from hardware.usfs import Usfs
from hardware.icm20948 import Icm20948
from hardware.usfs import Usfs

class IMU(Component):
    NAME = 'imu'
    USFS     = 0
    ICM20948 = 1
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
        self._yaw_matches             = False
        self._has_matched_heading     = False # True if headings have ever stably-matched
        self._is_stable               = False
        # dynamic trim configuration
        self._enable_dynamic_trim     = _cfg.get('enable_dynamic_trim')
        self._min_stability_threshold = _cfg.get('min_stability_threshold')
        self._stability_threshold     = _cfg.get('stability_threshold')
        self._trim_adjustment_gain    = _cfg.get('trim_adjustment_gain')
        self._min_error_threshold     = _cfg.get('min_error_threshold')
        self._require_absolute_stability = _cfg.get('require_absolute_stability')
        self._min_stability_difference = _cfg.get('min_stability_difference')
        # dynamic trim state
        self._dynamic_yaw_offset = 0.0
        self._yaw                     = None
        self._yaw_radians             = None
        self._heading_stability_score = 0.0
        # components
        _component_registry = Component.get_registry()
        # ICM20948 ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._icm20948 = icm20948 if icm20948 else _component_registry.get(Icm20948.NAME)
        if not self._icm20948:
#           raise MissingComponentError('ICM20948 required for IMU.')
            self._icm20948 = Icm20948(config, level=Level.INFO)
            self._icm20948.include_accel_gyro(True)
        # USFS ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._usfs = usfs if usfs else _component_registry.get(Usfs.NAME)
        if not self._usfs:
#           raise MissingComponentError('USFS required for IMU.')
            self._usfs = Usfs(config, level=Level.INFO)
        self.set_prefer_imu(IMU.USFS if self._prefer_usfs else IMU.ICM20948)
        self._numeric_display = self._usfs.numeric_display
        self._log.info('ready.')

    @rate_limited(500) # 500 ms between calls
    def _change_callback(self, value):
        if value:
            self._log.info(Fore.WHITE + '🍏 IS STABLE transition callback: {}'.format(value))
        else:
            self._log.info(Fore.WHITE + '🍎 IS NOT STABLE transition callback: {}'.format(value))

    # properties ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

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
    def yaw_matches(self):
        return self._yaw_matches

    @property
    def dynamic_yaw_offset(self):
        '''
        return the current dynamic yaw offset in radians.
        '''
        return self._dynamic_yaw_offset

    @property
    def yaw(self):
        '''
        If the pair of IMUs are both calibrated and their yaw values roughly matched,
        this returns the fused yaw (compass heading) in degrees (as an int). If not,
        this returns the yaw value from the preferred IMU if a stale match has ever
        occurred, otherwise None.
        '''
        return self._yaw

    @property
    def yaw_radians(self):
        '''
        If the pair of IMUs are both calibrated and their headings roughly matched,
        this returns the fused compass heading in radians (as a float). If not, this
        returns the heading from the preferred IMU if a stale match has ever occurred,
        otherwise None.
        '''
        return self._yaw_radians

    @property
    def is_calibrated(self):
        '''
        Returns a tuple containing a flag for each IMU indicating its calibrated
        state, the USFS then ICM20948.
        '''
        return self._usfs.is_calibrated, self._icm20948.is_calibrated

    @property
    def yaw_stability_score(self):
        '''
        Return the normalized stability score (0.0-1.0) based on individual stdevs
        and yaw agreement. 1.0 = perfect (both stable, perfect agreement),
        0.0 = uncoordinated (both unstable, large disagreement).
        '''
        return self._heading_stability_score

    @property
    def is_stable(self):
        '''
        Returns true if the stability score is greater than or equal to the
        configured threshold, indicating the two IMUs are reasonably close
        to each other.
        '''
        return self._heading_stability_score >= self._stability_threshold

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def set_prefer_imu(self, identifier):
        '''
        This overrides the configuration setting to indicate which IMU is
        preferred, IMU.USFS or IMU.ICM20948.
        '''
        if identifier != IMU.USFS and identifier != IMU.ICM20948:
            raise ValueError('unrecognised IMU identifier.')
        self._prefer_usfs = identifier == IMU.USFS
        self._usfs.enable_matrix11x7(self._prefer_usfs)
        self._icm20948.enable_matrix11x7(not self._prefer_usfs)

    def poll(self):
        self._usfs.poll()
        self._icm20948.poll()
        # calculate heading error with circular wrapping
        _usfs_yaw_rad = self._usfs.yaw_radians
        _icm_yaw_rad = self._icm20948.yaw_radians
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
#       self._log.debug(Style.DIM + 
#           'enable_dynamic_trim: {}; '.format(self._enable_dynamic_trim) +
#           'usfs_cal: {}; icm_cal:  {}; '.format(self._usfs.is_calibrated, self._icm20948.is_calibrated) +
#           'error: {:.4f} ({:.2f}°) > min:  {:.4f}? ; '.format(abs(_error), math.degrees(abs(_error)), self._min_error_threshold) +
#           'usfs_stdev: {:.4f}; icm_stdev: {:.4f}; diff: {:.4f} > 0.01?; '.format(_usfs_stdev, _icm_stdev, abs(_usfs_stdev - _icm_stdev)) +
#           'usfs < thresh: {}; icm < thresh: {}'.format(_usfs_stdev < self._min_stability_threshold, _icm_stdev < self._min_stability_threshold)
#       )
#       self._log.debug('USFS queue len:  {}; stdev: {:.6f}'.format(len(self._usfs._queue), _usfs_stdev))

        # dynamic trim adjustment
        if self._enable_dynamic_trim:
            # check guard conditions
            if (self._usfs.is_calibrated and self._icm20948.is_calibrated
                    and abs(_error) > self._min_error_threshold):
                # determine which IMU is more stable
                _stdev_diff = abs(_usfs_stdev - _icm_stdev)
                # only adjust if there's a clear stability difference
                if _stdev_diff > self._min_stability_difference: # 0.005/~0.3° difference threshold
                    if _usfs_stdev < _icm_stdev:
                        # usfs is more stable, check absolute threshold if required
                        if not self._require_absolute_stability or _usfs_stdev < self._min_stability_threshold:
                            # adjust icm20948 toward it
                            _adjustment = _error * self._trim_adjustment_gain
                            self._dynamic_yaw_offset += _adjustment
                            if self._verbose:
                                self._log.info('adjusting ICM20948 trim by {:+.4f} rad ({:+.2f}°)'.format(
                                    _adjustment, math.degrees(_adjustment)))
                    elif _icm_stdev < _usfs_stdev:
                        # icm20948 is more stable, check absolute threshold if required
                        if not self._require_absolute_stability or _icm_stdev < self._min_stability_threshold:
                            # adjust usfs toward it (reverse error sign)
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
        # if both are calibrated and their resp. yaw matches we alter state of is_stable
        if self._usfs.is_calibrated and self._icm20948.is_calibrated:
            _is_stable = self.is_stable and self._yaw_matches
            if _is_stable != self._is_stable:
                self._change_callback(self._is_stable)
                self._is_stable = _is_stable
                # simple average for now (could weight by stability in future)
                self._yaw_radians = (_usfs_yaw_rad + _icm_adjusted) / 2.0
                self._yaw = int(round(math.degrees(self._yaw_radians)))
            else:
                self._yaw_radians = None
                self._yaw = None
        elif self._has_matched_heading: # we've somehow matched in the past
            if self._prefer_usfs:
                self._yaw_radians = self._usfs.yaw_radians
                self._yaw = self._usfs.yaw
            else: # ICM20948
                self._yaw_radians = self._icm20948.yaw_radians
                self._yaw = self._icm20948.yaw
        else:
            self._yaw_radians = None
            self._yaw = None

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
        # yaw ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        usfs_yaw     = self._usfs.yaw
        icm20948_yaw = self._icm20948.yaw
        self._yaw_matches = isclose(usfs_yaw, icm20948_yaw, abs_tol=5.0)
        _style = Style.BRIGHT if self._has_matched_heading else Style.NORMAL
        if self._yaw_matches:
            heading_display = (usfs_yaw + icm20948_yaw) / 2
            yaw_str = '{:7.2f}          '.format(heading_display)
        elif self._prefer_usfs:
            yaw_str = _style + '{:7.2f} | '.format(usfs_yaw) + Style.DIM + '{:7.2f}'.format(icm20948_yaw)
        else:
            yaw_str = Style.DIM + '{:7.2f}'.format(usfs_yaw) + _style + ' | {:7.2f}'.format(icm20948_yaw)
        # trim ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if self._usfs.is_adjusting_trim():
            trim_str = '; trim: {:5.2f}; '.format(self._usfs.trim_adjust)
        else:
            trim_str = ';              ; '
        # stability ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if self._is_stable and self._yaw_matches:
            self._has_matched_heading = True
            _stability_str = Style.BRIGHT + 'stability: {:3.2f}; '.format(self.yaw_stability_score) + Style.NORMAL
            self._numeric_display.set_brightness(NumericDisplay.HIGH_BRIGHTNESS)
        else:
            _stability_str = 'stability: {:3.2f}; '.format(self.yaw_stability_score)
            self._numeric_display.set_brightness(NumericDisplay.LOW_BRIGHTNESS)
        # calibrated? ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _usfs_is_calibrated, _icm20948_is_calibrated = self.is_calibrated
        _calib_str = ' calib? {} {} {}'.format(
            '+' if _usfs_is_calibrated else '-',
            '+' if _icm20948_is_calibrated else '-',
            '+' if self._has_matched_heading else '-')
        # log output
        self._log.info(
              Fore.RED     + 'pitch: ' + pitch_str + '; '
            + Fore.GREEN   + Style.NORMAL + 'roll: ' + roll_str + '; '
            + Fore.BLUE    + Style.NORMAL + 'yaw (u/i): ' + yaw_str
            + Fore.YELLOW  + Style.NORMAL + trim_str
            + Fore.MAGENTA + Style.NORMAL + _stability_str
            + Fore.CYAN    + _calib_str
        )

    def simple_show_info(self):
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
        usfs_yaw     = self._usfs.yaw
        icm20948_yaw = self._icm20948.yaw
        if isclose(usfs_yaw, icm20948_yaw, abs_tol=3.0):
            pass
        self._log.info(
                Fore.YELLOW + 'pitch: {:4.2f} | {:4.2f}; '.format(usfs_pitch, icm20948_pitch)
              + Fore.WHITE  + 'roll: {:4.2f} | {:4.2f}; '.format(usfs_roll, icm20948_roll)     
              + Fore.GREEN  + 'yaw: {:4.2f} | {:4.2f}'.format(usfs_yaw, icm20948_yaw)     
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
