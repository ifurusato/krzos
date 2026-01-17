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
from core.rdof import RDoF
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
        self._max_dynamic_offset      = math.radians(_cfg.get('max_dynamic_offset_deg', 30.0))
        self._offset_decay_rate       = _cfg.get('offset_decay_rate', 0.01)
        self._small_error_threshold   = math.radians(_cfg.get('small_error_threshold_deg', 2.0))
        # forced alignment state
        self._alignment_active              = False
        self._icm20948_alignment_offset     = 0.0
        self._usfs_alignment_offset         = 0.0
        self._aligned_rdof                  = None
        self._usfs_yaw_adjusted_radians     = 0.0
        self._icm20948_yaw_adjusted_radians = 0.0
        # dynamic trim state
        self._adjusting_icm20948      = False
        self._adjusting_usfs          = False
        self._icm20948_yaw_offset     = 0.0 # dynamic offset for ICM20948
        self._usfs_yaw_offset         = 0.0 # dynamic offset for USFS
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
        Return the last-polled pitch value, for the preferred IMU.
        '''
        if self._prefer_usfs:
            return self._usfs.pitch
        else:
            return self._icm20948.pitch

    @property
    def roll(self):
        '''
        Return the last-polled roll value, for the preferred IMU.
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
        Return the current dynamic yaw offset in radians, for the preferred IMU.
        '''
        if self._prefer_usfs:
            return self._usfs_yaw_offset
        else:
            return self._icm20948_yaw_offset

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
        '''
        The polling function for both IMUs.
        '''
        # 1. read sensors
        self._usfs.poll()
        self._icm20948.poll()
        # 2. calculate metrics
        _error, _usfs_stdev, _icm_stdev = self._calculate_stability_metrics()
        # 3. dynamic trim, if enabled and we're not already using forced alignment
        if self._enable_dynamic_trim and not self._alignment_active:
            self._apply_dynamic_trim(_error, _usfs_stdev, _icm_stdev)
        else:
            self._adjusting_icm20948 = False
            self._adjusting_usfs     = False
        # 4. apply offsets and check matching
        _usfs_adjusted, _icm_adjusted = self._apply_offsets_and_check_matching()
        # 5. fuse or select
        self._fuse_or_select_heading(_usfs_adjusted, _icm_adjusted)

    def _calculate_stability_metrics(self):
        '''
        Calculate error and stability scores.
        Returns: (error, usfs_stdev, icm_stdev)
        '''
        _usfs_yaw_rad = self._usfs.yaw_radians
        _icm_yaw_rad = self._icm20948.yaw_radians
        _error = (_usfs_yaw_rad - _icm_yaw_rad + π) % (2 * π) - π
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
    #   self._log.debug(Style.DIM +
    #       'enable_dynamic_trim: {}; '.format(self._enable_dynamic_trim) +
    #       'usfs_cal:  {}; icm_cal:   {}; '.format(self._usfs.is_calibrated, self._icm20948.is_calibrated) +
    #       'error: {:.4f} ({:.2f}°) > min:   {:.4f}?  ; '.format(abs(_error), math.degrees(abs(_error)), self._min_error_threshold) +
    #       'usfs_stdev: {:.4f}; icm_stdev: {:.4f}; diff: {:.4f} > 0.01? ; '.format(_usfs_stdev, _icm_stdev, abs(_usfs_stdev - _icm_stdev)) +
    #       'usfs < thresh:  {}; icm < thresh: {}'.format(_usfs_stdev < self._min_stability_threshold, _icm_stdev < self._min_stability_threshold)
    #   )
    #   self._log.debug('USFS queue len:   {}; stdev: {:.6f}'.format(len(self._usfs._queue), _usfs_stdev))
        return _error, _usfs_stdev, _icm_stdev

    def _apply_dynamic_trim(self, error, usfs_stdev, icm_stdev):
        '''
        Apply dynamic trim adjustment to non-preferred IMU.
        '''
        if (self._usfs.is_calibrated and self._icm20948.is_calibrated
                and abs(error) > self._min_error_threshold):
            if self._prefer_usfs:
                self._adjusting_usfs = False
                # usfs is preferred reference - only adjust ICM20948 toward it
                # check if USFS is stable enough to trust
                if not self._require_absolute_stability or usfs_stdev < self._min_stability_threshold:
                    _adjustment = error * self._trim_adjustment_gain
                    self._icm20948_yaw_offset += _adjustment
                    # apply decay if error is small
                    if abs(error) < self._small_error_threshold:
                        self._icm20948_yaw_offset *= (1.0 - self._offset_decay_rate)
                    # normalize to ±π
                    self._icm20948_yaw_offset = (self._icm20948_yaw_offset + π) % (2 * π) - π
                    # clamp to maximum
                    self._icm20948_yaw_offset = max(-self._max_dynamic_offset,
                                                    min(self._max_dynamic_offset, self._icm20948_yaw_offset))
                    if self._verbose:
                        self._log.debug('adjusting ICM20948 toward USFS by {:+.4f} rad ({:+.2f}°); offset: {:+.4f} rad ({:+.2f}°)'.format(
                            _adjustment, math.degrees(_adjustment),
                            self._icm20948_yaw_offset, math.degrees(self._icm20948_yaw_offset)))
            else:
                self._adjusting_icm20948 = True
                # icm20948 is preferred reference - only adjust USFS toward it
                # check if ICM20948 is stable enough to trust
                if not self._require_absolute_stability or icm_stdev < self._min_stability_threshold:
                    _adjustment = -error * self._trim_adjustment_gain
                    self._usfs_yaw_offset += _adjustment
                    # apply decay if error is small
                    if abs(error) < self._small_error_threshold:
                        self._usfs_yaw_offset *= (1.0 - self._offset_decay_rate)
                    # normalize to ±π
                    self._usfs_yaw_offset = (self._usfs_yaw_offset + π) % (2 * π) - π
                    # clamp to maximum
                    self._usfs_yaw_offset = max(-self._max_dynamic_offset,
                                                min(self._max_dynamic_offset, self._usfs_yaw_offset))
                    if self._verbose:
                        self._log.debug('adjusting USFS toward ICM20948 by {:+.4f} rad ({:+.2f}°); offset: {:+.4f} rad ({:+.2f}°)'.format(
                            _adjustment, math.degrees(_adjustment),
                            self._usfs_yaw_offset, math.degrees(self._usfs_yaw_offset)))

    def _apply_offsets_and_check_matching(self):
        '''
        Apply all offsets (dynamic + alignment) and determine if adjusted values match.
        Returns: (usfs_adjusted, icm_adjusted) in radians, normalized to 0-2π
        '''
        _usfs_yaw_rad = self._usfs.yaw_radians
        _icm_yaw_rad = self._icm20948.yaw_radians
        # calculate fused heading with applied offsets (dynamic + alignment)
        _icm_adjusted = _icm_yaw_rad + self._icm20948_yaw_offset + self._icm20948_alignment_offset
        _usfs_adjusted = _usfs_yaw_rad + self._usfs_yaw_offset + self._usfs_alignment_offset
        # normalize both to 0-2π
        if _icm_adjusted < 0:
            _icm_adjusted += 2 * π
        elif _icm_adjusted >= 2 * π:
            _icm_adjusted -= 2 * π
        if _usfs_adjusted < 0:
            _usfs_adjusted += 2 * π
        elif _usfs_adjusted >= 2 * π:
            _usfs_adjusted -= 2 * π
        # store adjusted values for show_info()
        self._usfs_yaw_adjusted_radians     = _usfs_adjusted
        self._icm20948_yaw_adjusted_radians = _icm_adjusted
        # check if adjusted values match (not raw values)
        _adjusted_error = (_usfs_adjusted - _icm_adjusted + π) % (2 * π) - π
        self._yaw_matches = abs(_adjusted_error) < math.radians(5.0)  # 5° tolerance
        return _usfs_adjusted, _icm_adjusted

    def _fuse_or_select_heading(self, usfs_adjusted, icm_adjusted):
        '''
        Fuse headings if stable, otherwise select preferred or return None.
        '''
        # determine stability and fusion
        if self._usfs.is_calibrated and self._icm20948.is_calibrated:
            _is_stable = self.is_stable and self._yaw_matches
            # handle stability state transitions
            if _is_stable != self._is_stable:
                self._is_stable = _is_stable
                self._change_callback(self._is_stable)
            # continuously fuse when stable
            if self._is_stable:
                # use circular mean for proper angle averaging
                _sin_avg = (math.sin(usfs_adjusted) + math.sin(icm_adjusted)) / 2.0
                _cos_avg = (math.cos(usfs_adjusted) + math.cos(icm_adjusted)) / 2.0
                self._yaw_radians = math.atan2(_sin_avg, _cos_avg)
                if self._yaw_radians < 0:
                    self._yaw_radians += 2 * π
                self._yaw = int(round(math.degrees(self._yaw_radians)))
            else:
                self._yaw_radians = None
                self._yaw = None
        elif self._has_matched_heading: # we've matched in the past
            # use adjusted values from preferred IMU
            if self._prefer_usfs:
                self._yaw_radians = usfs_adjusted
            else:
                self._yaw_radians = icm_adjusted
            self._yaw = int(round(math.degrees(self._yaw_radians)))
        else:
            self._yaw_radians = None
            self._yaw = None

    def align(self, rdof):
        '''
        Forcibly align the two IMUs on the specified rotational degree of freedom
        by adding an alignment offset to the non-preferred IMU to match the preferred one.
        This offset is separate from and additional to the dynamic trim offset.

        Calling with rdof=None clears any active alignment.

        Args:
            rdof: RDoF enum (PITCH, ROLL, or YAW) or None to clear alignment
        '''
        self._log.info('🌸 align: {}'.format(rdof))
        # clear alignment
        if rdof is None:
            self._alignment_active = False
            self._icm20948_alignment_offset = 0.0
            self._usfs_alignment_offset = 0.0
            self._aligned_rdof = None
            # reset dynamic trim offsets
            self._icm20948_yaw_offset = 0.0
            self._usfs_yaw_offset = 0.0
            self._log.info('alignment cleared; dynamic trim offsets reset')
            return
        # validate argument
        if not isinstance(rdof, RDoF):
            raise ValueError('argument must be RDoF enum or None')
        # require both IMUs calibrated
        if not (self._usfs.is_calibrated and self._icm20948.is_calibrated):
            self._log.warning('both IMUs must be calibrated before alignment; ignoring align() call')
            return
        # reset dynamic trim offsets before calculating alignment
        self._icm20948_yaw_offset = 0.0
        self._usfs_yaw_offset = 0.0
        if rdof == RDoF.YAW:
            _usfs_yaw = self._usfs.yaw_radians
            _icm_yaw = self._icm20948.yaw_radians
            # calculate shortest angular distance (circular wrapping)
            _error = (_usfs_yaw - _icm_yaw + π) % (2 * π) - π
            if self._prefer_usfs:
                # align ICM20948 to match USFS
                self._icm20948_alignment_offset = _error
                self._usfs_alignment_offset = 0.0
                self._log.info('aligned ICM20948 yaw to USFS: alignment offset = {:+.2f}° ({:+.4f} rad)'.format(
                    math.degrees(self._icm20948_alignment_offset), self._icm20948_alignment_offset))
            else:
                # align USFS to match ICM20948
                self._usfs_alignment_offset = -_error
                self._icm20948_alignment_offset = 0.0
                self._log.info('aligned USFS yaw to ICM20948: alignment offset = {:+.2f}° ({:+.4f} rad)'.format(
                    math.degrees(self._usfs_alignment_offset), self._usfs_alignment_offset))
        elif rdof == RDoF.PITCH:
            _usfs_pitch = self._usfs.pitch_radians
            _icm_pitch = self._icm20948.pitch_radians
            _error = _usfs_pitch - _icm_pitch
            if self._prefer_usfs:
                # align ICM20948 pitch to match USFS
                self._icm20948_alignment_offset = _error
                self._usfs_alignment_offset = 0.0
                self._log.info('aligned ICM20948 pitch to USFS: alignment offset = {:+.2f}° ({:+.4f} rad)'.format(
                    math.degrees(self._icm20948_alignment_offset), self._icm20948_alignment_offset))
            else:
                # align USFS pitch to match ICM20948
                self._usfs_alignment_offset = -_error
                self._icm20948_alignment_offset = 0.0
                self._log.info('aligned USFS pitch to ICM20948: alignment offset = {:+.2f}° ({:+.4f} rad)'.format(
                    math.degrees(self._usfs_alignment_offset), self._usfs_alignment_offset))
        elif rdof == RDoF.ROLL:
            _usfs_roll = self._usfs.roll_radians
            _icm_roll = self._icm20948.roll_radians
            _error = _usfs_roll - _icm_roll
            if self._prefer_usfs:
                # align ICM20948 roll to match USFS
                self._icm20948_alignment_offset = _error
                self._usfs_alignment_offset = 0.0
                self._log.info('aligned ICM20948 roll to USFS: alignment offset = {:+.2f}° ({:+.4f} rad)'.format(
                    math.degrees(self._icm20948_alignment_offset), self._icm20948_alignment_offset))
            else:
                # align USFS roll to match ICM20948
                self._usfs_alignment_offset = -_error
                self._icm20948_alignment_offset = 0.0
                self._log.info('aligned USFS roll to ICM20948: alignment offset = {:+.2f}° ({:+.4f} rad)'.format(
                    math.degrees(self._usfs_alignment_offset), self._usfs_alignment_offset))
        # mark alignment as active
        self._alignment_active = True
        self._aligned_rdof = rdof
        self._has_matched_heading = True

    def show_info(self):
        '''
        display pitch, roll, yaw, trim, stability and calibration status.
        this method only displays information and does not modify any class variables.
        '''
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
        usfs_yaw_raw = self._usfs.yaw
        icm20948_yaw_raw = self._icm20948.yaw
        usfs_yaw_adjusted = math.degrees(self._usfs_yaw_adjusted_radians)
        icm20948_yaw_adjusted = math.degrees(self._icm20948_yaw_adjusted_radians)
        _style = Style.BRIGHT if self._has_matched_heading else Style.NORMAL
        if self._yaw_matches:
            if self._yaw is not None:
                yaw_str = '{:7.2f}          '.format(self._yaw) # use the fused value
            else:
                yaw_str = ' None            '
        elif self._alignment_active:
            # show raw→adjusted for both IMUs
            if self._prefer_usfs:
                yaw_str = _style + '{:3.0f}→{:3.0f} | '.format(usfs_yaw_raw, usfs_yaw_adjusted) \
                     + Style.DIM + '{:3.0f}→{:3.0f}'.format(icm20948_yaw_raw, icm20948_yaw_adjusted)
            else:
                yaw_str = Style.DIM + '{:3.0f}→{:3.0f}'.format(usfs_yaw_raw, usfs_yaw_adjusted) \
                        + _style + ' | {:3.0f}→{:3.0f}'.format(icm20948_yaw_raw, icm20948_yaw_adjusted)
        else:
            # show just raw values
            if self._prefer_usfs:
                yaw_str = _style + '{:7.2f} | '.format(usfs_yaw_raw) + Style.DIM + '{:7.2f}'.format(icm20948_yaw_raw)
            else:
                yaw_str = Style.DIM + '{:7.2f}'.format(usfs_yaw_raw) + _style + ' | {:7.2f}'.format(icm20948_yaw_raw)
        # trim ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if self._usfs.is_adjusting_trim():
            trim_str = '; trim: {:5.2f}; '.format(self._usfs.trim_adjust)
        else:
            trim_str = ';              ; '
        # stability ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if self._is_stable and self._yaw_matches:
            _stability_str = Style.BRIGHT + 'stability: {:3.2f}; '.format(self.yaw_stability_score) + Style.NORMAL
            self._numeric_display.set_brightness(NumericDisplay.HIGH_BRIGHTNESS)
        else:
            _stability_str = 'stability: {:3.2f}; '.format(self.yaw_stability_score)
            self._numeric_display.set_brightness(NumericDisplay.LOW_BRIGHTNESS)
        # calibrated?  ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _usfs_is_calibrated, _icm20948_is_calibrated = self.is_calibrated
        _icm_style = Style.BOLD if self._adjusting_icm20948 else Style.NORMAL
        _usfs_style = Style.BOLD if self._adjusting_usfs else Style.NORMAL
        _calib_str = ' calib?  {}{} {}{} {}{}'.format(
            _usfs_style,
            '+' if _usfs_is_calibrated else '-',
            _icm_style,
            '+' if _icm20948_is_calibrated else '-',
            Style.NORMAL,
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
