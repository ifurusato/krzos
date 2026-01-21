

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# refactor: 2026-01-21

import time
import traceback
import itertools
import math
from math import pi as π
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from icm20948 import ICM20948
from hardware.abstract_imu import AbstractIMU
from core.logger import Level
from core.rdof import RDoF
from core.rate import Rate
from hardware.player import Player
from hardware.digital_pot import DigitalPotentiometer
from hardware.rotation_controller import RotationController
from hardware.numeric_display import NumericDisplay
from core.convert import Convert
from core.cardinal import Cardinal

class Icm20948(AbstractIMU):
    NAME = 'icm20948'
    HALF_PI = π / 2.0

    def __init__(self, config, level=Level.INFO):
        super().__init__(config, level=level)
        self._log.info('initialising icm20948…')
        _cfg = config['kros'].get('hardware').get(self.NAME)
        # configuration
        self._vertical_mount       = _cfg.get('vertical_mount', False)
        self._declination          = math.radians(_cfg.get('declination', 23.04))
        self._pitch_trim           = _cfg.get('pitch_trim', 0.0)
        self._roll_trim            = _cfg.get('roll_trim', 0.0)
        self._fixed_yaw_trim       = _cfg.get('yaw_trim', 0.0)
        self._yaw_trim             = self._fixed_yaw_trim
        self._bench_calibrate      = _cfg.get('bench_calibrate', False)
        self._motion_calibrate     = _cfg.get('motion_calibrate', False)
        self._calibration_rotation = _cfg.get('calibration_rotation', 360)
        self._poll_rate_hz         = _cfg.get('poll_rate_hz', 20)
        self._cardinal_tolerance   = _cfg.get('cardinal_tolerance', 0.0523)
        self._show_console         = _cfg.get('show_console')
        self._show_matrix11x7      = _cfg.get('show_matrix11x7')
        self._play_sound           = _cfg.get('play_sound', False)
        self._queue_length         = _cfg.get('queue_length', 60)
        self._stability_threshold  = _cfg.get('stability_threshold', 0.09)
        self._display_rate = 10

        # device orientation/config
        if self._vertical_mount:
            self._log.info('using vertical mount.')
            self._X, self._Y, self._Z = 0, 1, 2
        else:
            self._log.info('using horizontal mount.')
            self._X, self._Y, self._Z = 2, 1, 0
        self._axes = (self._Z, self._Y)
        self._use_tilt_compensation = False

        self._amin = None
        self._amax = None
        self._last_mag = None  # will store most recent mag reading as list [x, y, z]
        self.__icm20948 = ICM20948(i2c_addr=_cfg.get('i2c_address'))

        self._digital_pot = None
        _component_registry = type(self).get_registry()
        _digital_pot = _component_registry.get(DigitalPotentiometer.NAME)
        if _digital_pot:
            self._digital_pot = _digital_pot
            self._log.info('using digital pot at: ' + Fore.GREEN + '0x0A')
        self._rotation_controller = _component_registry.get(RotationController.NAME)
        if self._rotation_controller is None and self._motion_calibrate:
            self._log.warning('rotation controller not found in registry; motion calibration disabled.')
            self._motion_calibrate = False

        self._numeric_display = None
        if self._show_matrix11x7:
            _numeric_display = _component_registry.get(NumericDisplay.NAME)
            if _numeric_display:
                self._numeric_display = _numeric_display
            else:
                self._numeric_display = NumericDisplay()
        self._yaw_count = 0
        self._last_yaw = 0

    def _read_hardware(self):
        '''
        Reads all hardware values and updates instance variables.
        '''
        mag = list(self.__icm20948.read_magnetometer_data())
        self._last_mag = mag.copy()
        if self._amin is None or self._amax is None:
            self._amin = mag.copy()
            self._amax = mag.copy()
        self._accel[0], self._accel[1], self._accel[2], self._gyro[0], self._gyro[1], self._gyro[2] = self.__icm20948.read_accelerometer_gyro_data()
        for i in range(3):
            v = mag[i]
            if v < self._amin[i]:
                self._amin[i] = v
            if v > self._amax[i]:
                self._amax[i] = v
            mag[i] -= self._amin[i]
            try:
                mag[i] /= self._amax[i] - self._amin[i]
            except ZeroDivisionError:
                pass
            mag[i] -= 0.5
        if self._use_tilt_compensation:
            mag = self._tilt_compensate_magnetometer(mag, self._pitch, self._roll)
        self._radians = math.atan2(mag[self._axes[0]], mag[self._axes[1]])
        self._radians += self._declination
        if self._adjust_rdof == RDoF.YAW and self._digital_pot:
            self._trim_adjust = self._digital_pot.get_scaled_value(False)
            self._yaw_trim = self._fixed_yaw_trim + self._trim_adjust
            self._radians += self._yaw_trim
        else:
            self._radians += self._fixed_yaw_trim
        if self._radians < 0:
            self._radians += 2 * math.pi
        self._yaw = int(round(math.degrees(self._radians)))
        _x = self._accel[self._X]
        _y = self._accel[self._Y]
        _z = self._accel[self._Z]
        if self._adjust_rdof == RDoF.PITCH and self._digital_pot:
            self._trim_adjust = self._digital_pot.get_scaled_value(False)
            self._pitch = math.atan2(_x, _z) - Icm20948.HALF_PI + self._pitch_trim + self._trim_adjust
        else:
            self._pitch = math.atan2(_x, _z) - Icm20948.HALF_PI + self._pitch_trim
        _xz = _x*_x + _z*_z
        if _xz == 0:
            _xz = 0.001
        if self._adjust_rdof == RDoF.ROLL and self._digital_pot:
            self._trim_adjust = self._digital_pot.get_scaled_value(False)
            self._roll = math.atan2(_y, math.sqrt(_xz)) + self._roll_trim + self._trim_adjust
        else:
            self._roll = math.atan2(_y, math.sqrt(_xz)) + self._roll_trim

    def bench_calibrate(self):
        '''
        Manual calibration using only _read_hardware and cached state, never direct hardware access.
        '''
        _start_time = dt.now()
        self._yaw_count = 0
        _rate = Rate(self._poll_rate_hz, Level.ERROR)
        _counter = itertools.count()
        _count = 0
        _limit = 1800
        # Initialise min/max from first _read_hardware call
        self._amin = None
        self._amax = None
        self._log.info(Fore.YELLOW + 'calibrating to stability threshold: {}…'.format(self._stability_threshold))
        if self._play_sound:
            pass
        self._log.info(Fore.WHITE + Style.BRIGHT + '\n\n    press Return to begin rotation phase…\n')
        input()
        self._log.info(Fore.WHITE + Style.BRIGHT + '\n    rotate sensor through a horizontal 360° motion, then press Return when complete…\n')
        try:
            _rotation_complete = False
            while self.enabled and not _rotation_complete:
                _count = next(_counter)
                if _count > _limit:
                    break
                try:
                    self._read_hardware()
                    _yaw_radians = self._radians
                    _yaw_degrees = int(round(math.degrees(_yaw_radians)))
                    if _count % 5 == 0:
                        self._log.info('[{:3d}] calibrating at: '.format(_count) + Fore.GREEN + '{}°…'.format(_yaw_degrees))
                    time.sleep(0.05)
                except Exception as e:
                    self._log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
                import sys, select
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    input()
                    _rotation_complete = True
                else:
                    _rate.wait()

            self._log.info(Fore.YELLOW + 'measuring stability, do not move sensor…')
            self.clear_queue()
            _counter = itertools.count()

            while self.enabled:
                _count = next(_counter)
                if self.is_calibrated or _count > _limit:
                    break
                try:
                    self._read_hardware()
                    _yaw_radians = self._radians
                    self._queue.append(_yaw_radians)
                    if len(self._queue) < 20:
                        continue
                    self._stdev = self._circular_stdev(self._queue)
                    if self._stdev < self._stability_threshold:
                        self._is_calibrated = True
                        break
                    if _count % 5 == 0:
                        _yaw_degrees = int(round(math.degrees(_yaw_radians)))
                        self._log.info(
                            Fore.CYAN + '[{:03d}] calibrating…\tstdev: {:.2f} < {:.2f}?'.format(_count, self._stdev, self._stability_threshold)
                        )
                    time.sleep(0.05)
                except Exception as e:
                    self._log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
                _rate.wait()
        finally:
            _elapsed_ms = round((dt.now() - _start_time).total_seconds() * 1000.0)
            if self.is_calibrated:
                self._log.info(Fore.GREEN + 'IMU calibrated: elapsed: {: d}ms'.format(_elapsed_ms))
                if self._play_sound:
                    pass
            elif self.enabled:
                self._log.error('unable to calibrate IMU after elapsed: {:d}ms'.format(_elapsed_ms))
                if self._play_sound:
                    pass
        return self.is_calibrated

    def motion_calibrate(self):
        '''
        Motion calibration using only _read_hardware and cached state, never direct hardware access.
        '''
        if self._rotation_controller is None:
            self._log.error('rotation controller not available; cannot motion calibrate.')
            return False
        _start_time = dt.now()
        self._yaw_count = 0
        _rate = Rate(self._poll_rate_hz, Level.ERROR)
        _counter = itertools.count()
        _count = 0
        _limit = 1800
        self._amin = None
        self._amax = None
        self._log.info(Fore.YELLOW + 'calibrating to stability threshold: {}…'.format(self._stability_threshold))
        if self._play_sound:
            Player.play('cheep')
        self._log.info(Fore.WHITE + Style.BRIGHT + '\n\n beginning automatic {}° rotation for calibration…\n'.format(self._calibration_rotation))
        if not self._rotation_controller.enabled:
            self._rotation_controller.enable()
        try:
            _imu_counter = itertools.count()
            def _imu_poll_callback():
                _c = next(_imu_counter)
                self._read_hardware()
            self._rotation_controller.add_poll_callback(_imu_poll_callback)
            self._rotation_controller.rotate_blocking(self._calibration_rotation, RotationController.COUNTER_CLOCKWISE)
            self._rotation_controller.remove_poll_callback(_imu_poll_callback)
            time.sleep(1.0)
            self._log.info(Fore.YELLOW + 'rotation complete, measuring stability…')
            self.clear_queue()
            _counter = itertools.count()
            while self.enabled:
                _count = next(_counter)
                if self.is_calibrated or _count > _limit:
                    break
                try:
                    self._read_hardware()
                    _yaw_radians = self._radians
                    self._queue.append(_yaw_radians)
                    if len(self._queue) < 20:
                        continue
                    self._stdev = self._circular_stdev(self._queue)
                    if self._stdev < self._stability_threshold:
                        self._is_calibrated = True
                        break
                    if _count % 5 == 0:
                        _queue_len = len(self._queue)
                        _queue_status = "filling" if _queue_len < 20 else "checking"
                        _yaw_degrees = int(round(math.degrees(_yaw_radians)))
                        self._log.info(
                            Fore.CYAN + '[{:03d}] calibrating…\t{}: {:3d}/{:3d}; ; '.format(
                                _count, _queue_status, _queue_len, self._queue_length)
                            + Fore.YELLOW + '{:.2f}°; '.format(_yaw_degrees)
                            + Fore.CYAN + Style.DIM + '(calibrated? {}; over limit? {})'.format(
                                self.is_calibrated, _count > _limit)
                        )
                except Exception as e:
                    self._log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
                _rate.wait()
        finally:
            _elapsed_ms = round((dt.now() - _start_time).total_seconds() * 1000.0)
            if self.is_calibrated:
                self._log.info(Fore.GREEN + 'IMU calibrated: elapsed: {: d}ms'.format(_elapsed_ms))
                if self._play_sound:
                    pass
            elif self.enabled:
                self._log.error('unable to calibrate IMU after {:d}ms elapsed.'.format(_elapsed_ms))
                if self._play_sound:
                    pass
        return self.is_calibrated

#EOF
