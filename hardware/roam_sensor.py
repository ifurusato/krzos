#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-07
# modified: 2025-11-13
#
# Provides fused distance readings for Roam behaviour, blending an IR distance
# sensor with the VL53L5CX sensor.

import time
from datetime import datetime as dt
import numpy as np
import itertools
from collections import deque
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.orientation import Orientation
from hardware.vl53l5cx_sensor import Vl53l5cxSensor
from hardware.fore_sensor import ForeSensor

class RoamSensor(Component):
    NAME = 'roam-sensor'
    '''
    RoamSensor fuses the forward-facing near-field ForeSensor (IR) based on
    a Pololu/Sharp analog IR distance sensor, with the VL53L5CX multi-zone
    ToF sensor to provide a single front-facing distance value optimized for
    obstacle avoidance and speed limiting.

    The fusion logic takes the minimum of the two sensor readings to make
    sure that the robot always reacts to the closest obstacle.
    Smoothing is optionally applied to stabilize the output.

    If either sensor is not provided, RoamSensor will instantiate its own.

    :param config:             the application configuration
    :param vl53l5cx_sensor:    optional VL53L5CX sensor instance
    :param fore_sensor:        optional ForeSensor instance
    :param level:              the log level
    '''
    def __init__(self, config, vl53l5cx_sensor=None, fore_sensor=None, level=Level.INFO):
        self._log = Logger(RoamSensor.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        if config is None or not isinstance(config, dict):
            raise ValueError('invalid configuration argument.')
        _cfg = config['kros'].get('hardware').get('roam_sensor')
        if _cfg is None:
            raise ValueError('missing kros.hardware.roam_sensor config section.')
        # fusion parameters
        self._smoothing      = _cfg.get('smoothing', True)
        _smoothing_window    = _cfg.get('smoothing_window', 5)
        self._window         = deque(maxlen=_smoothing_window) if self._smoothing else None
        self._log.info('smoothing={}, window={}'.format(self._smoothing, _smoothing_window))
        self._min_distance   = _cfg.get('min_distance')   # mm
        self._max_distance   = _cfg.get('max_distance')   # mm, the "no obstacle" threshold
        self._stale_timeout_ms = _cfg.get('stale_timeout_ms', 250) # 100ms default
        self._non_floor_row_offset = _cfg.get('non_floor_row_offset', 0)
        self._log.info('non-floor row offset: {}'.format(self._non_floor_row_offset))
        self._verbose        = False
        self._counter = itertools.count()
        # sensor instantiation
        self._fore_sensor = None
        self._vl53l5cx = None
        _component_registry = Component.get_registry()
        # Vl53l5cxSensor class
        if vl53l5cx_sensor is not None:
            self._vl53l5cx = vl53l5cx_sensor
        else:
            self._vl53l5cx = _component_registry.get(Vl53l5cxSensor.NAME)
            if self._vl53l5cx is None:
                self._log.info('creating VL53L5CX sensor…')
                self._vl53l5cx = Vl53l5cxSensor(config, level=Level.INFO)
        # ForeSensor class
        if fore_sensor is not None:
            self._fore_sensor = fore_sensor
        else:
            self._fore_sensor = _component_registry.get(ForeSensor.NAME)
            if self._fore_sensor is None:
                self._log.info('creating ForeSensor…')
                self._fore_sensor = ForeSensor(config, level=Level.INFO)
        if not self._fore_sensor:
            raise Exception('no forward IR sensor available.')
        self._last_value     = None
        self._last_read_time = dt.now()
        self._log.info('ready.')

    @property
    def name(self):
        return RoamSensor.NAME

    @property
    def min_distance(self):
        return self._min_distance

    @property
    def max_distance(self):
        return self._max_distance

    def _get_vl53l5cx_front_distance(self):
        '''
        Returns the median distance from the central 4 pixels of the VL53L5CX grid.
        Uses the middle 2 columns (3, 4) and the first 2 non-floor rows (dynamically
        determined from sensor calibration).

        Row 0 = bottom/floor, Row 7 = top/far.
        '''
        if not self._vl53l5cx.enabled:
            self._log.warning('VL53L5CX not enabled.')
            return None
        data = self._vl53l5cx.get_distance_mm()
        if data is None:
            return None
        try:
            grid = np.array(data).reshape((8, 8))
            # get first two non-floor rows from calibration
#           non_floor_rows = self._vl53l5cx.non_floor_rows[:2]
            non_floor_rows = self._vl53l5cx.non_floor_rows[self._non_floor_row_offset:self._non_floor_row_offset+2]
            center_cols = [3, 4]  # middle two columns
            values = [grid[row, col] for row in non_floor_rows for col in center_cols]
            # filter out invalid readings
            values = [v for v in values if v is not None and v > 0]
            if not values:
                self._log.warning('No valid VL53L5CX center values.')
                return None
            median = float(np.median(values))
            return median
        except Exception as e:
            self._log.error('Error extracting VL53L5CX center values: {}'.format(e))
            return None

    def _fuse(self, ir_value, vl53_value):
        '''
        Fuses IR and VL53L5CX sensor values.
        Returns the minimum valid distance, or None if both are invalid.
        '''
        valid_distances = []
        if ir_value is not None and ir_value > 0:
            valid_distances.append(ir_value)
        if vl53_value is not None and vl53_value > 0:
            valid_distances.append(vl53_value)
        if not valid_distances:
            return None
        return min(valid_distances)

    def _smooth(self, value):
        '''
        Applies median filter smoothing to the fused value if enabled.
        Returns smoothed value (mm) or original value. It filters out
        zeros from the window to prevent spurious readings from affecting
        the output.
        '''
        if value is None:
            return None
        if not self._smoothing:
            return value
        self._window.append(value)
        # filter out any zeros before calculating median
        valid_window = [v for v in self._window if v > 0]
        if not valid_window:
            return None
#       smoothed = np.mean(self._window)
        # using median is more robust to outliers than mean
        smoothed = np.median(valid_window)
        return smoothed

    def get_distance(self):
        '''
        Returns a fused and smoothed distance value for Roam behaviour.
        Takes the minimum of the IR ForeSensor and the VL53L5CX ToF sensor.
        '''
        _ir_distance_cm = self._fore_sensor.get_distance_cm()
        _tof_distance_mm  = self._get_vl53l5cx_front_distance()
        if _ir_distance_cm is None or _tof_distance_mm is None:
            now = dt.now()
            elapsed_ms = (now - self._last_read_time).total_seconds() * 1000.0
            if self._last_value is not None and elapsed_ms < self._stale_timeout_ms:
                self._log.info('both sensors None, using last value: {:.1f}mm (age: {:.0f}ms)'.format(
                    self._last_value, elapsed_ms))
                return self._last_value
            return -1.0
        _ir_distance_mm = _ir_distance_cm * 10.0 # convert to mm
        value = self._fuse(_ir_distance_mm, _tof_distance_mm)
        if self._smoothing and value is not None:
            value = self._smooth(value)
        if value is not None:
            self._last_value = value
            self._last_read_time = dt.now()
            return value
        return None

    def check_timeout(self):
        '''
        Returns True if the last value is stale (timeout exceeded).
        '''
        elapsed_ms = (dt.now() - self._last_read_time).total_seconds() * 1000.0
        return elapsed_ms > self._stale_timeout_ms

    def enable(self):
        '''
        Enables both sensors.
        '''
        if not self.enabled:
            self._fore_sensor.enable()
            self._vl53l5cx.enable()
            super().enable()
            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def disable(self):
        '''
        Disables both sensors.
        '''
        if self.enabled:
            if self._fore_sensor:
                self._fore_sensor.disable()
            if self._vl53l5cx:
                self._vl53l5cx.disable()
            super().disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

    def close(self):
        '''
        Closes both sensors and frees resources.
        '''
        self.disable()
        if not self.closed:
            if self._fore_sensor:
                self._fore_sensor.close()
            if self._vl53l5cx:
                self._vl53l5cx.close()
            super().close()
            self._log.info('closed.')
        else:
            self._log.warning('already closed.')

#EOF
