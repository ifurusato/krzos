#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2024-05-20
# modified: 2026-01-16

import time
import traceback
import itertools
import asyncio
import math, statistics
from math import pi as π
from threading import Thread
from collections import deque
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from icm20948 import ICM20948
from hardware.numeric_display import NumericDisplay

from core.cardinal import Cardinal
from core.convert import Convert
from core.component import Component
from core.logger import Logger, Level
from core.orientation import Orientation
from core.rotation import Rotation
from core.rdof import RDoF
from core.rate import Rate
from core.ranger import Ranger
from hardware.player import Player
from hardware.digital_pot import DigitalPotentiometer
from hardware.rotation_controller import RotationController, RotationPhase


class Icm20948(Component):
    NAME = 'icm20948'
    HALF_PI = π / 2.0
    '''
    Wraps the functionality of an ICM20948 IMU largely as a compass, though
    pitch and roll are also available. This includes optional trim adjustment,
    a calibration check, an optional console, and numeric display of yaw,
    as well as making raw accelerometer and gyroscope values available.

    :param config:          the application configuration
    :param level:           the log level
    '''
    def __init__(self, config, level=Level.INFO):
        self._log = Logger(Icm20948.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._log.info('initialising icm20948…')
        if not isinstance(config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(name)))
        self._counter = itertools.count()
        # configuration
        _cfg = config['kros'].get('hardware').get('icm20948')
        self._verbose            = _cfg.get('verbose')
        self._show_console       = _cfg.get('show_console')
        self._show_matrix11x7    = _cfg.get('show_matrix11x7')
        self._play_sound         = _cfg.get('play_sound') # if True, play sound to indicate calibration
        # calibration
        self._bench_calibrate    = _cfg.get('bench_calibrate')
        self._motion_calibrate   = _cfg.get('motion_calibrate')
        # declination
        _declination_degrees = _cfg.get('declination', 23.04)
        self._declination = math.radians(_declination_degrees)
        self._log.info('declination: ' + Fore.GREEN + '{:5.3f}° ({:.6f} rad)'.format(_declination_degrees, self._declination))
        # set up trim control
        self._pitch_trim = _cfg.get('pitch_trim') # 0.0
        self._log.info('pitch trim: ' + Fore.GREEN + '{:.4f}'.format(self._pitch_trim))
        self._roll_trim  = _cfg.get('roll_trim') # 4.0
        self._log.info('roll trim:  ' + Fore.GREEN + '{:.4f}'.format(self._roll_trim))
        # use fixed yaw trim value
        self._fixed_yaw_trim = _cfg.get('yaw_trim') # - π
        self._log.info('yaw trim:   ' + Fore.GREEN + '{:.4f}'.format(self._fixed_yaw_trim))
        self._yaw_trim = self._fixed_yaw_trim # initial value if adjusting
        self._digital_pot = None
        self._trim_adjust = 0.0
        self._adjust_rdof = None  # which RDoF to adjust with pot
        _component_registry = Component.get_registry()
        # configure potentiometer
        _digital_pot = _component_registry.get(DigitalPotentiometer.NAME)
        if _digital_pot:
            self._digital_pot = _digital_pot
            self._log.info('using digital pot at: ' + Fore.GREEN + '0x0A')
            # assume output range is already set
        self._calibration_rotation = _cfg.get('calibration_rotation')
        self._rotation_controller = _component_registry.get(RotationController.NAME)
        if self._rotation_controller is None:
            self._log.warning('rotation controller not found in registry; motion calibration disabled.')
            self._motion_calibrate = False
        # add numeric display for yaw
        self._numeric_display   = None
        if self._show_matrix11x7:
            _numeric_display = _component_registry.get(NumericDisplay.NAME)
            if _numeric_display:
                self._numeric_display = _numeric_display
            else:
                self._numeric_display = NumericDisplay()
        self._cardinal_tolerance = _cfg.get('cardinal_tolerance') # tolerance to cardinal points (in radians)
        self._log.info('cardinal tolerance: {:.8f}'.format(self._cardinal_tolerance))
        # general orientation
        _vertical_mount = _cfg.get('vertical_mount')
        if _vertical_mount: # orientation of Pimoroni board mounted vertically, Y along front-rear axis of robot
            self._log.info('using vertical mount.')
            self._X = 0
            self._Y = 1
            self._Z = 2
        else: # orientation of Adafruit board mounted horizontally, Z vertical, X along front-rear axis of robot
            self._log.info('using horizontal mount.')
            self._X = 2
            self._Y = 1
            self._Z = 0
        self._use_tilt_compensation = False # TODO config
        # The two axes which relate to yaw/heading depend on orientation of the
        # sensor, think Left & Right, Forwards and Back, ignoring Up and Down.
        # When the sensor is sitting vertically upright in a Breakout Garden
        # socket, use (Z,Y), where hanging upside down would be (Y,Z).
        self._axes = self._Z, self._Y
        # queue for stability check stats
        self._stdev = 0.0
        self._queue_length = _cfg.get('queue_length') # also affects how fast mean catches up to data
        self._queue = deque([], self._queue_length)
        self._stability_threshold = _cfg.get('stability_threshold')
        # misc/variables
        self._yaw_count = 0
        self._display_rate = 10 # display every 10th set of values
        self._poll_rate_hz = _cfg.get('poll_rate_hz')
        self._log.info('poll rate: ' + Fore.GREEN + '{: d}Hz'.format(self._poll_rate_hz))
        self._radians = None
        self._amin = None
        self._amax = None
        self._pitch = 0.0
        self._roll  = 0.0
        self._yaw = 0
        self._last_yaw = 0
        self._formatted_yaw = lambda: 'Yaw: {: d}°'.format(self._yaw)
        self._mean_yaw = 0
        self._mean_yaw_radians = None
        self._accel = [0.0, 0.0, 0.0]
        self._gyro =  [0.0, 0.0, 0.0]
        self._include_accel_gyro = _cfg.get('include_accel_gyro')
        self._is_calibrated  = False
        # instantiate sensor class
        self.__icm20948 = ICM20948(i2c_addr=_cfg.get('i2c_address'))
        self._log.info('ready.')

    def enable_matrix11x7(self, enable):
        '''
        The flag is initially a configuration value but subsequently can be used
        to enable/disable the Matrix11x7 display (if available at configuration).
        If not, calls to this method are ignored.
        '''
        if self._numeric_display:
            self._show_matrix11x7 = enable

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def numeric_display(self):
        '''
        Return the numeric display if it has been instantiated.
        '''
        return self._numeric_display

    @property
    def queue_length(self):
        return self._queue_length

    @property
    def is_calibrated(self):
        '''
        Return true if the compass is calibrated. This generally requires
        turning the sensor through 360° to set minimum and maximum values,
        and then waiting unmoving for it to settle, until the standard
        deviation of a queue of values falls below a configured threshold.
        '''
        return self._is_calibrated

    @property
    def pitch(self):
        '''
        Return the last-polled pitch value in degrees.
        '''
        return math.degrees(self._pitch)

    @property
    def pitch_radians(self):
        '''
        Return the last-polled pitch value in radians.
        '''
        return self._pitch

    @property
    def roll(self):
        '''
        Return the last-polled roll value in degrees.
        '''
        return math.degrees(self._roll)

    @property
    def roll_radians(self):
        '''
        Return the last-polled roll value in radians.
        '''
        return self._roll

    @property
    def uncalibrated_yaw(self):
        '''
        Return the yaw (compass heading) in degrees from a potentially
        uncalibrated IMU.

        This may not be a valid value if the device is not calibrated.
        '''
        if self._amin is None or self._amax is None:
            self._amin = list(self.__icm20948.read_magnetometer_data())
            self._amax = list(self.__icm20948.read_magnetometer_data())
        return self._read_yaw(self._amin, self._amax)

    @property
    def yaw(self):
        '''
        Return the last-polled yaw (compass heading) in degrees (as an int).

        This is only valid if the device is calibrated.
        '''
        return self._yaw

    @property
    def yaw_radians(self):
        '''
        Return the last-polled yaw (compass heading) in radians.

        This is only valid if the device is calibrated.
        '''
        return self._radians

    @property
    def mean_yaw(self):
        '''
        Return the mean yaw (compass heading) in degrees (as an int). This is
        the mean value of the current queue, whose size and rate accumulated
        are set in configuration. Because this is calculated from the queue,
        if the queue is changing rapidly this returned value won't accurately
        reflect the mean. Depending on configuration this takes roughly 1s to
        stabilise to a mean reflective of the robot's position, which then
        doesn't change very quickly. It is therefore suitable for gaining an
        accurate yaw value of a resting robot.

        This is only valid if the device is calibrated.
        '''
        return self._mean_yaw

    @property
    def mean_yaw_radians(self):
        '''
        Return the mean yaw (compass heading) in radians (as a float).
        '''
        return self._mean_yaw_radians

    @property
    def standard_deviation(self):
        '''
        Return the current value of the standard deviation of yaw values
        calculated from the queue.
        '''
        return self._stdev

    @property
    def accelerometer(self):
        '''
        Return the IMU's accelerometer value as an x,y,z value.
        If not enabled this returns zeros.
        '''
        return self._accel

    @property
    def gyroscope(self):
        '''
        Return the IMU's gyroscope value as an x,y,z value.
        If not enabled this returns zeros.
        '''
        return self._gyro

    def include_accel_gyro(self, include):
        self._include_accel_gyro = include

    def adjust_trim(self, rdof):
        '''
        Enable trim adjustment for the specified rotational degree of freedom.
        '''
        if not isinstance(rdof, RDoF):
            raise ValueError('argument must be RDoF enum')
        if self._digital_pot is None:
            self._log.warning('digital potentiometer not available, trim adjustment disabled.')
            return
        self._adjust_rdof = rdof
        self._log.info('trim adjustment enabled for: {}'.format(rdof.label))

    def is_cardinal_aligned(self, cardinal=None):
        '''
        Returns True if the mean yaw is aligned within a 3° tolerance to
        the specified cardinal directory, or if the argument is None, any of
        the four cardinal directions.
        '''
        if self._mean_yaw_radians is None:
            return False
        _angle = self._mean_yaw_radians
        _degrees = int(Convert.to_degrees(_angle))
        if ((cardinal is None or cardinal is Cardinal.NORTH)
                    and math.isclose(_angle, Cardinal.NORTH.radians, abs_tol=self._cardinal_tolerance)):
            return True
        elif ((cardinal is None or cardinal is Cardinal.WEST)
                    and math.isclose(_angle, Cardinal.WEST.radians, abs_tol=self._cardinal_tolerance)):
            return True
        elif ((cardinal is None or cardinal is Cardinal.SOUTH)
                    and math.isclose(_angle, Cardinal.SOUTH.radians, abs_tol=self._cardinal_tolerance)):
            return True
        elif ((cardinal is None or cardinal is Cardinal.EAST)
                    and math.isclose(_angle, Cardinal.EAST.radians, abs_tol=self._cardinal_tolerance)):
            return True
        else:
            return False

    def difference_from_cardinal(self, cardinal):
        '''
        Returns the difference between the current yaw and the provided
        Cardinal direction, in radians.
        '''
        return Convert.get_offset_from_cardinal(self.yaw_radians, cardinal)

    def ratio_from_cardinal(self, cardinal):
        '''
        Returns a ratio (range: 0.0-1.0) between the current yaw and the
        provided Cardinal direction.
        '''
        return Convert.get_offset_from_cardinal(self.yaw_radians, cardinal) / Icm20948.HALF_PI

    def get_formatted_yaw(self):
        '''
        Return a lambda function whose result is the last-polled yaw (compass heading)
        in degrees, formatted as a string.

        This is only valid if the device is calibrated.
        '''
        return self._formatted_yaw

    def bench_calibrate(self):
        '''
        Manually calibrate the sensor by looping while the sensor is rotated through a 360°
        motion, then leave it to rest for a few seconds. This times out after 60 seconds.

        There is a ballistic behaviour in MotionController to perform this same function.

        Returns True or False upon completion (in addition to setting the class variable).
        '''
        _start_time = dt.now()
        self._yaw_count = 0
        _rate = Rate(self._poll_rate_hz, Level.ERROR)
        _ranger = Ranger(0.0, 180.0, 0.0, 0.5)
        _counter = itertools.count()
        _count = 0
        _limit = 1800 # 1 minute
        self._amin = list(self.__icm20948.read_magnetometer_data())
        self._amax = list(self.__icm20948.read_magnetometer_data())
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
                    _yaw_radians = self._read_yaw(self._amin, self._amax, calibrating=True)
                    _yaw_degrees = int(round(math.degrees(_yaw_radians)))
                    if _count % 5 == 0:
                        self._log.info('[{:3d}] calibrating at: '.format(_count) + Fore.GREEN + '{}°…'.format(_yaw_degrees))
                except Exception as e:
                    self._log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))

                import sys, select

                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    line = input()
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
                    _yaw_radians = self._read_yaw(self._amin, self._amax, calibrating=True)
                    _yaw_degrees = int(round(math.degrees(_yaw_radians)))
                    USE_ADAPTIVE_CALIBRATION = False
                    if USE_ADAPTIVE_CALIBRATION:
                        if self._calibration_check_adaptive(_yaw_radians):
                            break
                    else:
                        if self._calibration_check(_yaw_radians):
                            break
                    if _count % 5 == 0:
                        self._log.info(
                                Fore.CYAN + '[{:03d}] calibrating…\tstdev: {:.2f} < {:.2f}? ; '.format(_count, self._stdev, self._stability_threshold)
                                + Fore.YELLOW + '{:.2f}°; '.format(_yaw_degrees)
                                + Fore.CYAN + Style.DIM + '(calibrated?  {}; over limit? {})'.format(self.is_calibrated, _count > _limit)
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
                self._log.error('unable to calibrate IMU after elapsed: {:d}ms'.format(_elapsed_ms))
                if self._play_sound:
                    pass
        return self.is_calibrated

    def motion_calibrate(self):
        '''
        Programmatically calibrate the sensor by commanding the robot to rotate
        through a 360° motion, then measuring stability.

        Returns True or False upon completion (in addition to setting the class variable).
        '''
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
            # phase 1: rotation to capture min/max magnetometer values
            _imu_counter = itertools.count()
            def _imu_poll_callback():
                _count = next(_imu_counter)
                try:
                    _yaw_radians = self._read_yaw(self._amin, self._amax, calibrating=True)
                    if _count % 25 == 0:
                        self._log.info(Style.DIM + '[{:3d}] calibrating…'.format(_count))
                except Exception as e:
                    self._log.error('{} encountered: {}'.format(type(e), e))

            self._rotation_controller.add_poll_callback(_imu_poll_callback)
            self._rotation_controller.rotate_blocking(self._calibration_rotation, Rotation.COUNTER_CLOCKWISE)
            self._rotation_controller.remove_poll_callback(_imu_poll_callback)
            # allow robot to settle after rotation
            time.sleep(1.0)
            # phase 2: stability measurement (robot stationary)
            self._log.info(Fore.YELLOW + 'rotation complete, measuring stability…')
            self.clear_queue()
            _counter = itertools.count()

            while self.enabled:
                _count = next(_counter)
                if self.is_calibrated or _count > _limit:
                    break
                try:
                    # read yaw with trim applied (not calibrating anymore)
                    _yaw_radians = self._read_yaw(self._amin, self._amax, calibrating=False)
                    _yaw_degrees = int(round(math.degrees(_yaw_radians)))
                    # check calibration status
                    if self._calibration_check(_yaw_radians):
                        break
                    if _count % 5 == 0:
                        _queue_len = len(self._queue)
                        _queue_status = "filling" if _queue_len < 20 else "checking"
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

    def clear_queue(self):
        '''
        Clears the statistic queue.
        '''
        self._queue.clear()

    def _calibration_check(self, yaw_radians):
        '''
        Adds a yaw value in radians to the queue and checks to see if the IMU is
        calibrated, according to the contents of the queue having a standard
        deviation less than a set threshold.

        Note that this does not clear the queue.
        '''
        self._queue.append(yaw_radians)
        self._yaw_count += 1
        # require minimum samples before checking stability
        _min_samples = 20 # reasonable minimum for stdev calculation
        if len(self._queue) < _min_samples:
            return False
        # calculate stdev with available samples
        self._stdev = self._circular_stdev(self._queue)
        _stdev_degrees = math.degrees(self._stdev)
        # check if stable
        if self._stdev < self._stability_threshold:
            self._is_calibrated = True
        return self._is_calibrated

    def _calibration_check_adaptive(self, yaw_radians):
        '''
        Adds a yaw value in radians to the queue and checks calibration status.
        Uses adaptive threshold based on sample size.
        '''
        self._queue.append(yaw_radians)
        self._yaw_count += 1
        # minimum samples before checking
        _min_samples = 20
        if len(self._queue) < _min_samples:
            return False
        # calculate stdev
        self._stdev = self._circular_stdev(self._queue)
        # adaptive threshold: stricter with more samples
        # at 20 samples:  1.5x threshold
        # at 100 samples: 1.0x threshold (configured value)
        _sample_ratio = min(len(self._queue) / self._queue_length, 1.0)
        _adaptive_multiplier = 1.5 - (0.5 * _sample_ratio)  # 1.5 -> 1.0
        _current_threshold = self._stability_threshold * _adaptive_multiplier
        if self._stdev < _current_threshold:
            self._is_calibrated = True
        return self._is_calibrated

    def x_calibration_check(self, yaw_radians):
        '''
        Adds a yaw value in radians to the queue and checks to see if the IMU is
        calibrated, according to the contents of the queue having a standard
        deviation less than a set threshold.

        Note that this does not clear the queue.
        '''
        self._queue.append(yaw_radians)
        self._yaw_count += 1
        if len(self._queue) < self._queue_length:
            return False
        self._stdev = self._circular_stdev(self._queue)
        _stdev_degrees = math.degrees(self._stdev)
        if self._stdev < self._stability_threshold:
            self._is_calibrated = True
        return self._is_calibrated

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
            return π
        return math.sqrt(-2 * math.log(r))

    def scan(self, enabled=None, callback=None):
        '''
        This starts a loop that will repeat until the application exits or
        until the optional enabled flag becomes False. For a single read of
        the sensor use poll().

        The optional callback will be executed upon each loop.

        This is currently only used in testing and calibration, so it uses
        a slower poll rate.

        Note: calling this method will fail if not previously calibrated.
        '''
        self._log.info(Fore.YELLOW + 'begin scan…')
        _poll_rate_hz = 5
        _rate = Rate(_poll_rate_hz, Level.ERROR)
        if not self._is_calibrated:
#           self._log.warning('compass not calibrated yet, call calibrate() first.')
            raise Exception('compass not calibrated yet, call calibrate() first.')
        while enabled:
            self.poll()
            self.show_info()
            if callback:
                callback()
            _rate.wait()

    def _display_yaw(self):
        if self._show_matrix11x7 and self._numeric_display:
            self._numeric_display.show_int(self._yaw)

    def poll(self):
        '''
        An individual call to the sensor. This is called in a loop by scan(),
        but can be called independently. This sets pitch, roll and yaw.

        We have a fixed yaw trim value (in radians). If the digital pot
        is available, its value is added to the yaw trim.

        Note: calling this method will fail if not previously calibrated.
        '''
#       self._log.info('poll… ')
        try:
            self._radians = self._read_yaw(self._amin, self._amax)
            self._queue.append(self._radians)
            if len(self._queue) > 1:
                self._stdev = self._circular_stdev(self._queue)
                if self._stdev < self._stability_threshold:
                    self._is_calibrated = True
            if len(self._queue) > 0:
                sin_sum = sum(math.sin(a) for a in self._queue)
                cos_sum = sum(math.cos(a) for a in self._queue)
                n = len(self._queue)
                self._mean_yaw_radians = math.atan2(sin_sum / n, cos_sum / n)
                if self._mean_yaw_radians < 0:
                    self._mean_yaw_radians += 2 * π
                self._mean_yaw = int(round(math.degrees(self._mean_yaw_radians)))
            # use configured axis mapping
            _accel = self.accelerometer
            _x = _accel[self._X]
            _y = _accel[self._Y]
            _z = _accel[self._Z]
            # apply trim adjustment if enabled
            if self._adjust_rdof == RDoF.PITCH and self._digital_pot:
                self._trim_adjust = self._digital_pot.get_scaled_value(False)
                self._pitch = math.atan2(_x, _z) - Icm20948.HALF_PI + self._pitch_trim + self._trim_adjust
            elif self._adjust_rdof == RDoF.ROLL and self._digital_pot:
                self._trim_adjust = self._digital_pot.get_scaled_value(False)
                _xz = _x*_x + _z*_z
                if _xz == 0:
                    _xz = 0.001
                self._roll = math.atan2(_y, math.sqrt(_xz)) + self._roll_trim + self._trim_adjust
            else:
                self._pitch = math.atan2(_x, _z) - Icm20948.HALF_PI + self._pitch_trim
                _xz = _x*_x + _z*_z
                if _xz == 0:
                    _xz = 0.001
                self._roll = math.atan2(_y, math.sqrt(_xz)) + self._roll_trim
            # continue…
            self._last_yaw = self._yaw
            if next(self._counter) % self._display_rate == 0:
                if self._show_console:
                    if self._is_calibrated:
                        _style = Style.BRIGHT
                    else:
                        _style = Style.NORMAL
                    _variance = 3
                    if not math.isclose(self._yaw, self._last_yaw, abs_tol=_variance):
                        _info = _style + "yaw: {: 3d}° / mean: {: 3d}°;".format(self._yaw, int(self._mean_yaw))
                        _info += Fore.WHITE + " pitch: {: 4.2f}°; roll: {:4.2f}°".format(self.pitch, self.roll)
                        if self._adjust_rdof:
                            if self._adjust_rdof == RDoF.YAW:
                                _info += Fore.CYAN + Style.DIM + " yaw trim: fxd={:.2f} / adj={:.2f} / trm={:.2f}".format(
                                    self._fixed_yaw_trim, self._trim_adjust, self._yaw_trim)
                            elif self._adjust_rdof == RDoF.PITCH:
                                _info += Fore.CYAN + Style.DIM + " pitch trim: fxd={:.2f} / adj={:.2f}".format(
                                    self._pitch_trim, self._trim_adjust)
                            elif self._adjust_rdof == RDoF.ROLL:
                                _info += Fore.CYAN + Style.DIM + " roll trim: fxd={:.2f} / adj={:.2f}".format(
                                    self._roll_trim, self._trim_adjust)
                        _info += Fore.CYAN + Style.DIM + " stdev: {:.2f}".format(self._stdev)
                        self._log.info(_info)
                self._display_yaw()

            return self._yaw, self._pitch, self._roll
        except Exception as e:
            self._log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
            return None

    def show_info_simple(self):
        '''
        Just displays the RDoFs.
        '''
        self._log.info(
                Fore.YELLOW + 'pitch: {:4.2f}; '.format(self.pitch)
              + Fore.WHITE  + 'roll: {:4.2f}; '.format(self.roll)
              + Fore.GREEN  + 'yaw: {:4.2f}'.format(self.yaw)
        )

    def show_info(self):
        '''
        Displays the RDoFs along with the active trim adjustment.
        '''
        _info = Fore.YELLOW + 'pitch: {:6.2f}; '.format(self.pitch)
        _info += Fore.WHITE + 'roll: {:6.2f}; '.format(self.roll)
        _info += Fore.GREEN + 'yaw: {:6.2f}; '.format(self.yaw)
        if self._adjust_rdof:
            if self._adjust_rdof == RDoF.YAW:
                _info += Fore.CYAN + Style.DIM + 'yaw trim: fxd={:7.4f} / adj={:7.4f} / trm={:7.4f}'.format(
                    self._fixed_yaw_trim, self._trim_adjust, self._yaw_trim)
            elif self._adjust_rdof == RDoF.PITCH:
                _info += Fore.CYAN + Style.DIM + 'pitch trim: fxd={:7.4f} / adj={:7.4f}'.format(
                    self._pitch_trim, self._trim_adjust)
            elif self._adjust_rdof == RDoF.ROLL:
                _info += Fore.CYAN + Style.DIM + 'roll trim: fxd={:7.4f} / adj={:7.4f}'.format(
                    self._roll_trim, self._trim_adjust)
        self._log.info(_info)

    def _read_yaw(self, amin, amax, calibrating=False):
        '''
        Does the work of obtaining the yaw value in radians.
        '''
        if self._amin is None or self._amax is None:
            self._amin = list(self.__icm20948.read_magnetometer_data())
            self._amax = list(self.__icm20948.read_magnetometer_data())
        mag = list(self.__icm20948.read_magnetometer_data())
        if self._include_accel_gyro:
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
        # apply tilt compensation
        if self._use_tilt_compensation:
            mag = self._tilt_compensate_magnetometer(mag, self._pitch, self._roll)
        self._radians = math.atan2(mag[self._axes[0]], mag[self._axes[1]])
        # apply declination first
        self._radians += self._declination # rad
        if calibrating:
            # adjust nothing while calibrating
            pass
        elif self._adjust_rdof == RDoF.YAW and self._digital_pot:
            self._trim_adjust = self._digital_pot.get_scaled_value(False)
            self._yaw_trim = self._fixed_yaw_trim + self._trim_adjust
            self._radians += self._yaw_trim
        else:
            self._radians += self._fixed_yaw_trim
        if self._radians < 0:
            self._radians += 2 * math.pi
        self._yaw = int(round(math.degrees(self._radians)))
        return self._radians

    def _tilt_compensate_magnetometer(self, mag, pitch, roll):
        '''
        Apply tilt compensation to magnetometer reading using pitch and roll.
        Returns corrected magnetometer vector in robot frame.
        '''
        # rotation matrices for pitch and roll
        sin_pitch = math.sin(pitch)
        cos_pitch = math.cos(pitch)
        sin_roll  = math.sin(roll)
        cos_roll  = math.cos(roll)
        # apply pitch rotation (around Y axis)
        mag_pitch = [
            mag[0] * cos_pitch + mag[2] * sin_pitch,
            mag[1],
            -mag[0] * sin_pitch + mag[2] * cos_pitch
        ]
        # apply roll rotation (around X axis)
        mag_compensated = [
            mag_pitch[0],
            mag_pitch[1] * cos_roll - mag_pitch[2] * sin_roll,
            mag_pitch[1] * sin_roll + mag_pitch[2] * cos_roll
        ]
        return mag_compensated

    def enable(self):
        if not self.closed:
            if not self.enabled:
                Component.enable(self)
                # check calibration flags
                if self._motion_calibrate and not self._is_calibrated:
                    self._log.info('beginning motion calibration…')
                    self.motion_calibrate()
                elif self._bench_calibrate and not self._is_calibrated:
                    self._log.info('beginning bench calibration…')
                    self.bench_calibrate()
                self._log.info('enabled.')
            else:
                self._log.warning('already enabled.')

    def disable(self):
        super().disable()

#EOF
