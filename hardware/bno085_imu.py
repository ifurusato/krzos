#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2024-05-20
# modified: 2026-01-21
#
# BNO085 IMU subclass with KRZOS integration

from __future__ import annotations

import sys, select
import traceback
import time
import itertools
import math
from math import pi as π
from collections import deque
from colorama import init, Fore, Style
init()

from hardware.bno085 import (
    BNO085,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

from core.component import Component
from core.logger import Logger, Level
from core.rate import Rate
from core.rdof import RDoF
from core.rotation import Rotation
from hardware.digital_pot import DigitalPotentiometer
from hardware.numeric_display import NumericDisplay
from hardware.rotation_controller import RotationController

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Bno085(BNO085, Component):
    '''
    BNO085 IMU with KRZOS-specific features including trim adjustment,
    calibration workflows, stability tracking, and display integration.

    Args:
        config:  the application configuration
        level:   the log level
    '''
    NAME = 'bno085'
    HALF_PI = π / 2.0

    def __init__(self, config, level=Level.INFO):
        self._log = Logger(Bno085.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)

        self._log.info('initialising bno085…')
        if not isinstance(config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(config)))

        # configuration
        _cfg = config['kros'].get('hardware').get('bno085')
        _i2c_bus_number = _cfg.get('i2c_bus_number', 1)
        _i2c_address = _cfg.get('i2c_address', BNO085.DEFAULT_I2C_ADDRESS)

        # initialize base class
        BNO085.__init__(self, i2c_id=_i2c_bus_number, i2c_address=_i2c_address)

        # declination and trim (read from config in degrees, store in radians)
        _declination_degrees = _cfg.get('declination', 13.8)
        self._declination = math.radians(_declination_degrees)
        self._pitch_trim = _cfg.get('pitch_trim', 0.0)
        self._roll_trim = _cfg.get('roll_trim', 0.0)
        self._yaw_trim = _cfg.get('yaw_trim', 0.0)
        self._log.info('declination: {:+7.3f}° ({:+9.6f} rad)'.format(_declination_degrees, self._declination))
        self._log.info('pitch trim:  {:+7.3f}° ({:+9.6f} rad)'.format(math.degrees(self._pitch_trim), self._pitch_trim))
        self._log.info('roll trim:   {:+7.3f}° ({:+9.6f} rad)'.format(math.degrees(self._roll_trim), self._roll_trim))
        self._log.info('yaw trim:    {:+7.3f}° ({:+9.6f} rad)'.format(math.degrees(self._yaw_trim), self._yaw_trim))

        # axis configuration
        self._swap_pitch_roll = _cfg.get('swap_pitch_roll', False)
        self._invert_pitch = _cfg.get('invert_pitch', False)
        self._invert_roll = _cfg.get('invert_roll', False)
        self._invert_yaw = _cfg.get('invert_yaw', False)

        # calibration configuration
        self._bench_calibrate = _cfg.get('bench_calibrate', False)
        self._motion_calibrate = _cfg.get('motion_calibrate', False)
        self._calibration_rotation = _cfg.get('calibration_rotation', 450)
        self._auto_save_calibration = _cfg.get('auto_save_calibration', True)
        self._use_saved_calibration = _cfg.get('use_saved_calibration', False)
        self._play_sound = _cfg.get('play_sound', False)
        self._show_console = _cfg.get('show_console', False)
        self._show_matrix11x7 = _cfg.get('show_matrix11x7', False)

        # stability tracking
        self._queue_length = _cfg.get('queue_length', 100)
        self._stability_threshold = _cfg.get('stability_threshold', 0.09)
        self._min_calibration_accuracy = _cfg.get('min_calibration_accuracy', 2)
        self._queue = deque([], self._queue_length)
        self._stdev = 0.0
        self._mean_yaw = 0
        self._mean_yaw_radians = None

        # euler angles (uncorrected, in radians)
        self._pitch = 0.0
        self._roll = 0.0
        self._yaw = 0.0

        # corrected euler angles (after trim applied, in radians)
        self._corrected_pitch = 0.0
        self._corrected_roll = 0.0
        self._corrected_yaw = 0.0

        # trim adjust
        self._digital_pot = None
        self._trim_adjust = 0.0
        self._adjust_rdof = None  # which RDoF to adjust with pot

        # poll counter
        self._poll_counter = itertools.count()

        # configure potentiometer
        _component_registry = Component.get_registry()
        _digital_pot = _component_registry.get(DigitalPotentiometer.NAME)
        if _digital_pot:
            self._digital_pot = _digital_pot
            self._log.info('using digital pot at: ' + Fore.GREEN + '0x{:02X}'.format(self._digital_pot.i2c_address))

        # numeric display for heading
        self._numeric_display = None
        if self._show_matrix11x7:
            _numeric_display = _component_registry.get(NumericDisplay.NAME)
            if _numeric_display:
                self._numeric_display = _numeric_display
            else:
                self._numeric_display = NumericDisplay()

        # rotation controller for motion calibration
        self._rotation_controller = None
        if self._motion_calibrate:
            _rotation_controller = _component_registry.get(RotationController.NAME)
            if _rotation_controller:
                self._rotation_controller = _rotation_controller
            else:
                self._log.warning('rotation controller not found in registry; motion calibration disabled.')
                self._motion_calibrate = False

        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    # Properties

    @property
    def pitch(self):
        '''corrected pitch in degrees'''
        return math.degrees(self._corrected_pitch)

    @property
    def pitch_radians(self):
        '''corrected pitch in radians'''
        return self._corrected_pitch

    @property
    def roll(self):
        '''corrected roll in degrees'''
        return math.degrees(self._corrected_roll)

    @property
    def roll_radians(self):
        '''corrected roll in radians'''
        return self._corrected_roll

    @property
    def yaw(self):
        '''corrected yaw in degrees'''
        return math.degrees(self._corrected_yaw)

    @property
    def yaw_radians(self):
        '''corrected yaw in radians'''
        return self._corrected_yaw

    @property
    def is_calibrated(self):
        '''True if magnetometer accuracy ≥ threshold'''
        return self._magnetometer_accuracy >= self._min_calibration_accuracy

    @property
    def standard_deviation(self):
        '''circular stdev of yaw queue (radians)'''
        return self._stdev

    @property
    def mag_accuracy(self):
        '''magnetometer calibration quality (0-3)'''
        return self._magnetometer_accuracy

    @property
    def mean_yaw(self):
        '''mean yaw from stability queue (degrees)'''
        return self._mean_yaw

    @property
    def mean_yaw_radians(self):
        '''mean yaw from stability queue (radians)'''
        return self._mean_yaw_radians

    @property
    def numeric_display(self):
        '''numeric display instance if available'''
        return self._numeric_display

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    # Public methods

    def adjust_trim(self, rdof):
        '''enable trim adjustment for the specified rotational degree of freedom.'''
        if not isinstance(rdof, RDoF):
            raise ValueError('argument must be RDoF enum')
        if self._digital_pot is None:
            self._log.warning('digital potentiometer not available, trim adjustment disabled.')
            return
        self._adjust_rdof = rdof
        self._log.info('trim adjustment enabled for: {}'.format(rdof.label))

    def enable(self):
        '''enable the hardware for the sensor'''
        if not self.enabled:
            self._log.info('enabling bno085…')
            Component.enable(self)

            # call base class enable
            BNO085.enable(self)

            # check for saved calibration
            time.sleep(0.5)  # allow sensor to load saved calibration data
            self.update()

            if self._use_saved_calibration and self._magnetometer_accuracy >= self._min_calibration_accuracy:
                self._log.info(Fore.WHITE + Style.BRIGHT + 'BNO085 loaded saved calibration (mag accuracy: {})'.format(self._magnetometer_accuracy))
                self._run_stability_check()
            else:
                if not self._use_saved_calibration:
                    self._log.info('not using saved calibration, running fresh calibration…')
                else:
                    self._log.info('no saved calibration (mag accuracy: {}), running calibration…'.format(self._magnetometer_accuracy))
                self._run_calibration()

            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def enable_matrix11x7(self, enable):
        '''enable/disable Matrix11x7 display if available'''
        if self._numeric_display:
            self._show_matrix11x7 = enable

    def show_info(self):
        '''display pitch, roll, yaw with trim info'''
        _info = Fore.YELLOW + 'pitch: {:6.2f}°; '.format(self.pitch)
        _info += Fore.WHITE + 'roll: {:6.2f}°; '.format(self.roll)
        _info += Fore.GREEN + 'yaw: {:6.2f}°'.format(self.yaw)
        if self._adjust_rdof:
            _info += Fore.CYAN + '; '
            if self._adjust_rdof == RDoF.YAW:
                _info += Fore.BLUE + 'yaw trim: {:7.4f}'.format(self._yaw_trim)
            elif self._adjust_rdof == RDoF.PITCH:
                _info += Fore.BLUE + 'pitch trim: {:7.4f}'.format(self._pitch_trim)
            elif self._adjust_rdof == RDoF.ROLL:
                _info += Fore.BLUE + 'roll trim: {:7.4f}'.format(self._roll_trim)
        self._log.info(_info)

    def poll(self):
        '''
        poll sensor, update Euler angles with trim/declination applied.
        returns corrected yaw in degrees, or None if no quaternion available.
        '''
        if self.closed:
            self._log.warning('already closed.')
            return None

        # update base sensor data
        self.update()

        # get quaternion
        quat = self.quaternion
        if quat is None:
            return None

        # convert quaternion to Euler angles (yaw, pitch, roll in radians)
        self._yaw, self._pitch, self._roll = self._quaternion_to_euler(*quat)

        # apply declination to yaw
        self._yaw += self._declination

        # normalize yaw to [0, 2π)
        if self._yaw < 0:
            self._yaw += 2 * math.pi
        elif self._yaw >= 2 * math.pi:
            self._yaw -= 2 * math.pi

        # apply axis swapping/inversion
        if self._swap_pitch_roll:
            self._pitch, self._roll = self._roll, self._pitch
        if self._invert_pitch:
            self._pitch *= -1.0
        if self._invert_roll:
            self._roll *= -1.0
        if self._invert_yaw:
            self._yaw *= -1.0

        # apply trim adjustment if enabled
        if self._adjust_rdof == RDoF.PITCH and self._digital_pot:
            self._trim_adjust = self._digital_pot.get_scaled_value(False)
            self._pitch_trim = self._trim_adjust
        elif self._adjust_rdof == RDoF.ROLL and self._digital_pot:
            self._trim_adjust = self._digital_pot.get_scaled_value(False)
            self._roll_trim = self._trim_adjust
        elif self._adjust_rdof == RDoF.YAW and self._digital_pot:
            self._trim_adjust = self._digital_pot.get_scaled_value(False)
            self._yaw_trim = self._trim_adjust

        # apply trim corrections
        self._corrected_pitch = self._pitch + self._pitch_trim
        self._corrected_roll = self._roll + self._roll_trim
        self._corrected_yaw = self._yaw - self._yaw_trim

        # normalize corrected yaw to [0, 2π)
        if self._corrected_yaw < 0:
            self._corrected_yaw += 2 * math.pi
        elif self._corrected_yaw >= 2 * math.pi:
            self._corrected_yaw -= 2 * math.pi

        # update stability tracking
        self._queue.append(self._corrected_yaw)
        if len(self._queue) > 1:
            self._stdev = self._circular_stdev(self._queue)

        # calculate mean yaw from queue
        if len(self._queue) > 0:
            sin_sum = sum(math.sin(a) for a in self._queue)
            cos_sum = sum(math.cos(a) for a in self._queue)
            n = len(self._queue)
            self._mean_yaw_radians = math.atan2(sin_sum / n, cos_sum / n)
            if self._mean_yaw_radians < 0:
                self._mean_yaw_radians += 2 * π
            self._mean_yaw = int(round(math.degrees(self._mean_yaw_radians)))

        # update display if configured
        if self._show_matrix11x7 and self._numeric_display:
            if self.is_calibrated:
                self._numeric_display.set_brightness(NumericDisplay.MEDIUM_BRIGHTNESS)
            else:
                self._numeric_display.set_brightness(NumericDisplay.LOW_BRIGHTNESS)
            self._numeric_display.show_int(int(math.degrees(self._corrected_yaw)))

            _count = next(self._poll_counter)
            if _count % 5 == 0:
                if self._adjust_rdof == RDoF.PITCH and self._digital_pot:
                    self._log.info(Fore.WHITE + 'pitch: {:4.2f}°; corrected pitch: {:4.2f}°; pitch trim: '.format( \
                            math.degrees(self._pitch), math.degrees(self._corrected_pitch))
                            + Fore.GREEN + '{:5.3f}'.format(self._trim_adjust))
                elif self._adjust_rdof == RDoF.ROLL and self._digital_pot:
                    self._log.info(Fore.WHITE + 'roll: {:4.2f}°; corrected roll: {:4.2f}°; roll trim: '.format( \
                            math.degrees(self._roll), math.degrees(self._corrected_roll))
                            + Fore.GREEN + '{:5.3f}'.format(self._trim_adjust))
                elif self._adjust_rdof == RDoF.YAW and self._digital_pot:
                    self._log.info(Fore.WHITE + 'yaw: {:4.2f}°; corrected yaw: {:4.2f}°; yaw trim: '.format( \
                            math.degrees(self._yaw), math.degrees(self._corrected_yaw))
                            + Fore.GREEN + '{:5.3f}'.format(self._trim_adjust))

        return math.degrees(self._corrected_yaw)

    def clear_queue(self):
        '''clears the statistic queue'''
        self._queue.clear()

    def disable(self):
        '''disable sensor'''
        if not self.disabled:
            BNO085.disable(self)
            Component.disable(self)
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

    def close(self):
        '''closes the BNO085. This calls disable.'''
        Component.close(self)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    # Calibration methods

    def _run_stability_check(self):
        '''poll to fill queue and verify stability with saved calibration'''
        _rate = Rate(20, level=Level.WARN)
        for _ in range(20):
            self.poll()
            _rate.wait()
        if self.is_calibrated and self._stdev < self._stability_threshold:
            self._log.info('ready with saved calibration (stdev: {:.4f})'.format(self._stdev))
        else:
            self._log.warning('saved calibration unstable, running calibration…')
            self._run_calibration()

    def _run_calibration(self):
        '''run calibration based on config flags'''
        if self._motion_calibrate:
            success = self.motion_calibrate()
        elif self._bench_calibrate:
            success = self.bench_calibrate()
        else:
            self._log.warning('no calibration method configured')
            return
        if success and self._auto_save_calibration:
            self._log.info(Fore.WHITE + Style.BRIGHT + 'saving calibration to flash…')
            self.save_calibration_data()
            self._log.info('calibration saved.')

    def bench_calibrate(self):
        '''
        manual bench calibration - user rotates sensor through 3D space.
        returns True if successful, False otherwise.
        '''
        self._log.info(Fore.GREEN + 'starting bench calibration…')
        if self._play_sound:
            from hardware.player import Player
            Player.play('chatter-1')

        # begin hardware calibration
        self.begin_calibration()
        self.clear_queue()

        # rotation phase
        self._log.info(Fore.WHITE + Style.BRIGHT + '\n\n    press Return to begin rotation phase…\n')
        input()
        self._log.info(Fore.WHITE + Style.BRIGHT + '\n    rotate sensor through a horizontal 360° motion, then press Return when complete…\n')

        _rate = Rate(20, level=Level.WARN)
        _limit = 1800  # 90 seconds at 20Hz
        _count = 0
        _counter = itertools.count()

        # poll during rotation to allow sensor to collect calibration data
        _rotation_complete = False
        while self.enabled and not _rotation_complete:
            if _count > _limit:
                break
            self.poll()
            if self._show_console:
                if _count % 5 == 0:
                    self._log.info('mag accuracy: {}; stdev: {:.4f}'.format(self._magnetometer_accuracy, self._stdev))
            _count = next(_counter)

            # check if user pressed return
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = input()
                _rotation_complete = True
            else:
                _rate.wait()

        # stability measurement phase
        self._log.info(Fore.GREEN + 'measuring stability, do not move sensor…')
        self.clear_queue()
        _count = 0
        _counter = itertools.count()

        while _count < _limit:
            self.poll()
            if self._show_console:
                self._log.info('mag accuracy: {}; stdev: {:.4f}'.format(self._magnetometer_accuracy, self._stdev))

            # check completion criteria
            if self.is_calibrated and self._stdev < self._stability_threshold:
                self._log.info(Fore.GREEN + Style.BRIGHT + 'calibration successful! (mag: {}, stdev: {:.4f})'.format(
                    self._magnetometer_accuracy, self._stdev))
                if self._play_sound:
                    from hardware.player import Player
                    Player.play('chatter-2')
                return True

            _count = next(_counter)
            _rate.wait()

        # timeout
        self._log.warning('calibration timeout (mag: {}, stdev: {:.4f})'.format(
            self._magnetometer_accuracy, self._stdev))
        return False

    def motion_calibrate(self):
        '''
        automatic motion calibration - uses RotationController to rotate robot.
        returns True if successful, False otherwise.
        '''
        if not self._rotation_controller:
            self._log.error('rotation controller not available for motion calibration')
            return False

        self._log.info('starting motion calibration…')
        if self._play_sound:
            from hardware.player import Player
            Player.play('chatter-1')

        # begin hardware calibration
        self.begin_calibration()
        self.clear_queue()

        # register poll callback to update IMU during rotation
        self._rotation_controller.add_poll_callback(self.poll)

        # start rotation
        self._log.info(Fore.YELLOW + 'beginning automatic rotation of {}°…'.format(self._calibration_rotation))
        success = self._rotation_controller.rotate_blocking(self._calibration_rotation, Rotation.CLOCKWISE)

        # remove callback
        self._rotation_controller.remove_poll_callback(self.poll)

        if not success:
            self._log.error('rotation failed during calibration')
            return False

        self._log.info(Fore.GREEN + 'rotation complete, measuring stability…')

        # stability measurement phase
        self.clear_queue()
        _rate = Rate(20, level=Level.WARN)
        _limit = 1800  # 90 seconds at 20Hz
        _count = 0

        while _count < _limit:
            self.poll()
            if self._show_console:
                self._log.info('mag accuracy: {}  stdev: {:.4f}'.format(
                    self._magnetometer_accuracy, self._stdev))

            # check completion criteria
            if self.is_calibrated and self._stdev < self._stability_threshold:
                self._log.info(Fore.GREEN + Style.BRIGHT + 'calibration successful! (mag: {}, stdev: {:.4f})'.format(
                    self._magnetometer_accuracy, self._stdev))
                if self._play_sound:
                    from hardware.player import Player
                    Player.play('chatter-2')
                return True

            _rate.wait()
            _count += 1

        # timeout
        self._log.warning('calibration timeout (mag: {}, stdev: {:.4f})'.format(
            self._magnetometer_accuracy, self._stdev))
        return False

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    # Helper methods

    def _quaternion_to_euler(self, quat_i, quat_j, quat_k, quat_real):
        '''
        convert quaternion to Tait-Bryan Euler angles (yaw, pitch, roll).
        uses aerospace convention matching USFS.
        returns tuple of (yaw_rad, pitch_rad, roll_rad).
        '''
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (quat_real * quat_i + quat_j * quat_k)
        cosr_cosp = 1.0 - 2.0 * (quat_i * quat_i + quat_j * quat_j)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (quat_real * quat_j - quat_k * quat_i)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))  # clamp to [-1, 1]

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (quat_real * quat_k + quat_i * quat_j)
        cosy_cosp = 1.0 - 2.0 * (quat_j * quat_j + quat_k * quat_k)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw, pitch, roll

    def _circular_stdev(self, angles):
        '''
        calculate standard deviation for circular data (angles in radians).
        returns value in radians.
        '''
        if len(angles) < 2:
            return float('inf')
        sin_sum = sum(math.sin(a) for a in angles)
        cos_sum = sum(math.cos(a) for a in angles)
        n = len(angles)
        sin_mean = sin_sum / n
        cos_mean = cos_sum / n
        r = math.sqrt(sin_mean**2 + cos_mean**2)
        if r > 0.999999:
            return 0.0
        if r < 0.0001:
            return math.pi
        return math.sqrt(-2 * math.log(r))

#EOF
