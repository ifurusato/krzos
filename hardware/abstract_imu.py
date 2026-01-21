#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-01-21
# created:  2026-01-21
#
# Abstract base class for IMU devices.

import math
import itertools
from math import pi as π
from collections import deque
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level
from hardware.numeric_display import NumericDisplay
from core.convert import Convert
from core.cardinal import Cardinal
from core.rdof import RDoF

class AbstractIMU(Component):
    HALF_PI = π / 2.0
    '''
    Abstract base class for IMU sensors.

    Implements generic IMU behaviour: polling, queueing, stability checks,
    yaw/pitch/roll computation, trim logic, numeric display, and calibration flagging.

    Subclasses must implement:
        - _read_hardware(): fetches all raw sensor data and updates instance state
        - hardware-specific calibration routines as required
        - hardware-specific axis/rotation mapping if required

    Args:
        config:   the application configuration
        level:    the log level
    '''
    def __init__(self, config, level=Level.INFO):
        self._log = Logger(self.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._counter = itertools.count()
        _cfg = config['kros'].get('hardware').get(self.NAME)
        self._verbose            = _cfg.get('verbose')
        self._show_console       = _cfg.get('show_console')
        self._show_matrix11x7    = _cfg.get('show_matrix11x7')
        self._play_sound         = _cfg.get('play_sound')
        self._cardinal_tolerance = _cfg.get('cardinal_tolerance')
        self._queue_length       = _cfg.get('queue_length')
        self._queue = deque([], self._queue_length)
        self._stdev = 0.0
        self._stability_threshold = _cfg.get('stability_threshold')
        self._display_rate        = 10
        # orientation
        self._pitch = 0.0
        self._roll  = 0.0
        self._yaw = 0
        self._radians = None
        self._mean_yaw = 0
        self._mean_yaw_radians = None
        self._yaw_trim = 0.0
        self._fixed_yaw_trim = _cfg.get('yaw_trim', 0.0)
        self._pitch_trim = _cfg.get('pitch_trim', 0.0)
        self._roll_trim  = _cfg.get('roll_trim', 0.0)
        self._trim_adjust = 0.0
        self._adjust_rdof = None
        self._is_calibrated = False
        self._numeric_display = None
        _component_registry = Component.get_registry()
        if self._show_matrix11x7:
            _numeric_display = _component_registry.get(NumericDisplay.NAME)
            if _numeric_display:
                self._numeric_display = _numeric_display
            else:
                self._numeric_display = NumericDisplay()
        self._accel = [0.0, 0.0, 0.0]
        self._gyro  = [0.0, 0.0, 0.0]
        self._amin = None
        self._amax = None

    @property
    def name(self):
        return self.NAME

    @property
    def numeric_display(self):
        return self._numeric_display

    @property
    def queue_length(self):
        return self._queue_length

    @property
    def is_calibrated(self):
        return self._is_calibrated

    @property
    def pitch(self):
        return math.degrees(self._pitch)

    @property
    def pitch_radians(self):
        return self._pitch

    @property
    def roll(self):
        return math.degrees(self._roll)

    @property
    def roll_radians(self):
        return self._roll

    @property
    def yaw(self):
        return self._yaw

    @property
    def yaw_radians(self):
        return self._radians

    @property
    def mean_yaw(self):
        return self._mean_yaw

    @property
    def mean_yaw_radians(self):
        return self._mean_yaw_radians

    @property
    def standard_deviation(self):
        return self._stdev

    @property
    def accelerometer(self):
        return self._accel

    @property
    def gyroscope(self):
        return self._gyro

    def adjust_trim(self, rdof):
        '''
        Enable trim adjustment for the specified rotational degree of freedom.
        '''
        if not isinstance(rdof, RDoF):
            raise ValueError('argument must be RDoF enum')
        self._adjust_rdof = rdof

    def enable_matrix11x7(self, enable):
        '''
        Enable or disable the Matrix11x7 numeric display if available.
        '''
        if self._numeric_display:
            self._show_matrix11x7 = enable

    def clear_queue(self):
        self._queue.clear()

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

    def is_cardinal_aligned(self, cardinal=None):
        '''
        Returns True if the mean yaw is aligned within configured tolerance to cardinal direction.
        '''
        if self._mean_yaw_radians is None:
            return False
        _angle = self._mean_yaw_radians
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
        return Convert.get_offset_from_cardinal(self.yaw_radians, cardinal)

    def ratio_from_cardinal(self, cardinal):
        return Convert.get_offset_from_cardinal(self.yaw_radians, cardinal) / AbstractIMU.HALF_PI

    def poll(self):
        '''
        Poll the IMU hardware. Subclass must implement _read_hardware().
        Processing and queueing are handled here.
        '''
        self._read_hardware()
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

    def show_info(self):
        '''
        Displays the RDoFs and trim.
        '''
        _info = Fore.YELLOW + 'pitch: {:6.2f}; '.format(self.pitch)
        _info += Fore.WHITE + 'roll: {:6.2f}; '.format(self.roll)
        _info += Fore.GREEN + 'yaw: {:6.2f}; '.format(self.yaw)
        if self._adjust_rdof:
            if self._adjust_rdof == RDoF.YAW:
                _info += Fore.CYAN + Style.DIM + 'yaw trim: fxd={:7.4f} / adj={:7.4f}'.format(
                    self._fixed_yaw_trim, self._trim_adjust)
            elif self._adjust_rdof == RDoF.PITCH:
                _info += Fore.CYAN + Style.DIM + 'pitch trim: fxd={:7.4f} / adj={:7.4f}'.format(
                    self._pitch_trim, self._trim_adjust)
            elif self._adjust_rdof == RDoF.ROLL:
                _info += Fore.CYAN + Style.DIM + 'roll trim: fxd={:7.4f} / adj={:7.4f}'.format(
                    self._roll_trim, self._trim_adjust)
        self._log.info(_info)

    def enable(self):
        if not self.closed:
            if not self.enabled:
                Component.enable(self)
            else:
                self._log.warning('already enabled.')

    def disable(self):
        super().disable()

#EOF
