#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved.
#
# Refactored to subclass AbstractIMU and use real hardware I/O only in _read_hardware().
#
# author:   Ichiro Furusato
# refactor: 2026-01-21

import time
import traceback
import itertools
import math
from math import pi as π
from datetime import datetime as dt
from collections import deque
from colorama import init, Fore, Style
init()

from smbus2 import SMBus, i2c_msg

from hardware.abstract_imu import AbstractIMU
from core.logger import Level
from core.rdof import RDoF
from core.rate import Rate
from hardware.player import Player
from hardware.digital_pot import DigitalPotentiometer
from hardware.rotation_controller import RotationController
from hardware.numeric_display import NumericDisplay
from core.cardinal import Cardinal

class BNO085(AbstractIMU):
    NAME = 'bno085'
    DEFAULT_I2C_ADDRESS = 0x4A
    HALF_PI = π / 2.0

    def __init__(self, config, level=Level.INFO):
        super().__init__(config, level=level)
        self._log.info('initialising bno085…')
        _cfg = config['kros'].get('hardware').get(self.NAME)

        self._i2c_bus_number  = _cfg.get('i2c_bus_number', 1)
        self._i2c_address     = _cfg.get('i2c_address', BNO085.DEFAULT_I2C_ADDRESS)
        self._declination     = math.radians(_cfg.get('declination', 0.0))
        self._pitch_trim      = math.radians(_cfg.get('pitch_trim', 0.0))
        self._roll_trim       = math.radians(_cfg.get('roll_trim', 0.0))
        self._fixed_yaw_trim  = math.radians(_cfg.get('yaw_trim', 0.0))
        self._yaw_trim        = self._fixed_yaw_trim
        self._bench_calibrate = _cfg.get('bench_calibrate', False)
        self._motion_calibrate= _cfg.get('motion_calibrate', False)
        self._queue_length    = _cfg.get('queue_length', 100)
        self._stability_threshold = _cfg.get('stability_threshold', 0.09)
        self._calibration_rotation = _cfg.get('calibration_rotation', 360)
        self._poll_rate_hz    = _cfg.get('poll_rate_hz', 20)
        self._cardinal_tolerance = _cfg.get('cardinal_tolerance', 0.0523)
        self._show_console    = _cfg.get('show_console')
        self._show_matrix11x7 = _cfg.get('show_matrix11x7')
        self._play_sound      = _cfg.get('play_sound', False)
        self._include_accel_gyro = _cfg.get('include_accel_gyro', True)
        self._display_rate    = 10
        self._swap_pitch_roll = _cfg.get('swap_pitch_roll', False)
        self._invert_pitch    = _cfg.get('invert_pitch', False)
        self._invert_roll     = _cfg.get('invert_roll', False)
        self._invert_yaw      = _cfg.get('invert_yaw', False)
        self._min_calibration_accuracy = _cfg.get('min_calibration_accuracy', 2)
        self._yaw_count = 0
        self._last_yaw = 0

        # Hardware driver setup
        self._i2c = SMBus(self._i2c_bus_number)
        self._packet_buffer = bytearray(512)
        # The rest of your initialization from your original BNO085 driver, minus old Component logic.

        # Registry and optional peripherals
        self._rotation_controller = None
        _component_registry = type(self).get_registry()
        self._rotation_controller = _component_registry.get(RotationController.NAME)
        if self._rotation_controller is None and self._motion_calibrate:
            self._log.warning('rotation controller not found in registry; motion calibration disabled.')
            self._motion_calibrate = False

        self._digital_pot = None
        _digital_pot = _component_registry.get(DigitalPotentiometer.NAME)
        if _digital_pot:
            self._digital_pot = _digital_pot
            self._log.info('using digital pot at: ' + Fore.GREEN + '0x0A')
        self._numeric_display = None
        if self._show_matrix11x7:
            _numeric_display = _component_registry.get(NumericDisplay.NAME)
            if _numeric_display:
                self._numeric_display = _numeric_display
            else:
                self._numeric_display = NumericDisplay()
        self._queue = deque([], self._queue_length)
        self._stdev = 0.0
        self._amin = None
        self._amax = None
        self._calibration_status = 0

    def _read_hardware(self):
        '''
        Polls the BNO085 hardware and updates instance state.
        '''
        # --- Real hardware interaction code begins here ---
        # Replace this block with your known-working direct read logic by reading a rotation vector quaternion, and optionally raw accel/gyro
        try:
            # Read the expected rotation vector quaternion report via I2C
            self._i2c.write_byte(self._i2c_address, 0x05)  # rotation vector report ID, typically 0x05
            read_len = 14  # Quaternion with accuracy, per BNO085 docs
            read_msg = i2c_msg.read(self._i2c_address, read_len)
            self._i2c.i2c_rdwr(read_msg)
            rv = [b for b in list(read_msg)]
            # RV interpretation (per byte order in datasheet)
            qi_raw = int.from_bytes(rv[4:6], byteorder='little', signed=True)
            qj_raw = int.from_bytes(rv[6:8], byteorder='little', signed=True)
            qk_raw = int.from_bytes(rv[8:10], byteorder='little', signed=True)
            qr_raw = int.from_bytes(rv[10:12], byteorder='little', signed=True)
            accuracy = rv[12]  # accuracy in LSB degrees
            # Conversion for Q point
            q_point = 14
            scale = 2.0 ** -q_point
            quat = (qi_raw * scale, qj_raw * scale, qk_raw * scale, qr_raw * scale)
            self._last_quat = quat

            yaw, pitch, roll = self._quaternion_to_euler(*quat)
            yaw += self._declination
            if self._swap_pitch_roll:
                pitch, roll = roll, pitch
            if self._invert_pitch:
                pitch *= -1.0
            if self._invert_roll:
                roll *= -1.0
            if self._invert_yaw:
                yaw *= -1.0
            self._pitch = pitch + self._pitch_trim
            self._roll  = roll  + self._roll_trim
            self._radians = yaw + self._fixed_yaw_trim
            if self._radians < 0:
                self._radians += 2 * math.pi
            elif self._radians >= 2 * math.pi:
                self._radians -= 2 * math.pi
            self._yaw = int(round(math.degrees(self._radians)))
            # Ignore amin, amax for BNO085 unless doing explicit mag calibration with raw reports

            # Optionally, update self._accel and self._gyro if your existing methods do so
            self._accel = [0.0, 0.0, 0.0]
            self._gyro = [0.0, 0.0, 0.0]
            self._calibration_status = accuracy
        except Exception as e:
            self._log.error('{} in _read_hardware(): {}\n{}'.format(type(e), e, traceback.format_exc()))

    def _quaternion_to_euler(self, qi, qj, qk, qr):
        '''
        Convert quaternion to Tait-Bryan Euler angles (yaw, pitch, roll).
        '''
        sinr_cosp = 2.0 * (qr * qi + qj * qk)
        cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (qr * qj - qk * qi)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        siny_cosp = 2.0 * (qr * qk + qi * qj)
        cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw, pitch, roll

    @property
    def is_calibrated(self):
        # Typical BNO085 calibration field (mag accuracy from 0 to 3)
        return self._calibration_status >= self._min_calibration_accuracy

    def bench_calibrate(self):
        '''
        Manual calibration using only _read_hardware and cached state, never direct hardware access after init.
        Uses stability threshold and queue logic as for ICM20948.
        '''
        _start_time = dt.now()
        self._yaw_count = 0
        _rate = Rate(self._poll_rate_hz, Level.ERROR)
        _counter = itertools.count()
        _count = 0
        _limit = 1800
        self._log.info(Fore.YELLOW + 'calibrating with stability threshold: {}…'.format(self._stability_threshold))
        if self._play_sound:
            pass
        self._log.info(Fore.WHITE + Style.BRIGHT + '\n\n    press Return to begin rotation phase…\n')
        input()
        self._log.info(Fore.WHITE + Style.BRIGHT + '\n    rotate sensor through a 360° horizontal motion, then press Return when complete…\n')
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
                except Exception as e:
                    self._log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
                _rate.wait()
        finally:
            _elapsed_ms = round((dt.now() - _start_time).total_seconds() * 1000.0)
            if self.is_calibrated:
                self._log.info(Fore.GREEN + 'BNO085 calibrated: elapsed: {: d}ms'.format(_elapsed_ms))
                if self._play_sound:
                    pass
            elif self.enabled:
                self._log.error('unable to calibrate BNO085 after elapsed: {:d}ms'.format(_elapsed_ms))
                if self._play_sound:
                    pass
        return self.is_calibrated

    def motion_calibrate(self):
        '''
        Motion calibration using _read_hardware, queue logic, and rotation controller.
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
                self._log.info(Fore.GREEN + 'BNO085 calibrated: elapsed: {: d}ms'.format(_elapsed_ms))
                if self._play_sound:
                    pass
            elif self.enabled:
                self._log.error('unable to calibrate BNO085 after {:d}ms elapsed.'.format(_elapsed_ms))
                if self._play_sound:
                    pass
        return self.is_calibrated

#EOF
