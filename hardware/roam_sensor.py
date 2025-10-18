#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-07
# modified:  2025-10-07
#
# Provides fused distance readings for Roam behaviour, blending PWM and VL53L5CX sensor values.

import time
from datetime import datetime as dt
import numpy as np
from collections import deque
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.orientation import Orientation
from hardware.easing import Easing
from hardware.vl53l5cx_sensor import Vl53l5cxSensor
from hardware.distance_sensors import DistanceSensors

class RoamSensor(Component):
    NAME = 'roam-sensor'
    '''
    RoamSensor fuses the forward-facing near-field PWM-based DistanceSensor
    with the VL53L5CX multi-zone ToF sensor to provide a single front-facing
    distance value optimized for obstacle avoidance and speed limiting.

    The fusion uses a sigmoid weighting: the PWM sensor is prioritized for
    close range (0–max_range), VL53L5CX for mid-to-far range. Smoothing is
    applied to stabilize the output.

    If either sensor is not provided, RoamSensor will instantiate its own.

    :param config:             the application configuration
    :param vl53l5cx_sensor:    optional VL53L5CX sensor instance
    :param distance_sensors:   optional DistanceSensors instance
    :param level:              the log level
    '''
    def __init__(self, config, vl53l5cx_sensor=None, distance_sensors=None, level=Level.INFO):
        self._log = Logger(RoamSensor.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        if config is None or not isinstance(config, dict):
            raise ValueError('invalid configuration argument.')
        _cfg = config['kros'].get('hardware').get('roam_sensor')
        if _cfg is None:
            raise ValueError('missing kros.hardware.roam_sensor config section.')
        # sigmoid fusion parameters
        self._sigmoid_d0     = _cfg.get('sigmoid_d0', 150)  # inflection point (mm)
        self._sigmoid_k      = _cfg.get('sigmoid_k', 40)    # steepness
        self._smoothing      = _cfg.get('smoothing', True)
        _smoothing_window    = _cfg.get('smoothing_window', 5)
        self._window         = deque(maxlen=_smoothing_window) if self._smoothing else None
        self._min_distance   = _cfg.get('min_distance')   # mm
        self._max_distance   = _cfg.get('max_distance')   # mm, the "no obstacle" threshold
        self._pwm_max_range  = _cfg.get('pwm_max_range')  # PWM sensor range in mm for fusion
        self._use_sigmoid    = _cfg.get('use_sigmoid_fusion')
        _easing_value        = _cfg.get('easing', 'logarithmic')
        self._stale_timeout_ms = _cfg.get('stale_timeout_ms', 250) # 100ms default
        self._easing         = Easing.from_string(_easing_value)
        self._log.info('easing function: {}'.format(self._easing.name))
        # sensor instantiation
        _component_registry = Component.get_registry()
        # Vl53l5cxSensor class
        if vl53l5cx_sensor is not None:
            self._vl53l5cx = vl53l5cx_sensor
        else:
            self._vl53l5cx = _component_registry.get(Vl53l5cxSensor.NAME)
            if self._vl53l5cx is None:
                self._vl53l5cx = Vl53l5cxSensor(config, level=Level.INFO)
        self._distance_sensor = None
        if distance_sensors is None:
            _distance_sensors = _component_registry.get(DistanceSensors.NAME)
            if _distance_sensors is None:
                _distance_sensors = DistanceSensors(config, level=Level.INFO)
                _distance_sensors.enable()
            self._distance_sensor = _distance_sensors.get(Orientation.FWD)
        else:
            self._distance_sensor = distance_sensors.get(Orientation.FWD)
        if not self._distance_sensor:
            raise Exception('no forward distance sensor available.')
        if not self._distance_sensor.enabled:
            raise Exception('forward distance sensor not enabled.')
        self._last_value     = None
        self._last_read_time = dt.now()
        self._log.info('roam sensor instantiated [sigmoid_d0={}, sigmoid_k={}, smoothing={}, window={}]'
                .format(self._sigmoid_d0, self._sigmoid_k, self._smoothing, _smoothing_window))

    @property
    def name(self):
        return RoamSensor.NAME

    @property
    def min_distance(self):
        return self._min_distance

    @property
    def max_distance(self):
        return self._max_distance

    def sigmoid_weight(self, pwm_value):
        '''
        Returns the weighting for the PWM sensor based on its measured value.
        '''
        if pwm_value is None:
            return 0.0
        # Clamp value to range for safety
        d = max(0, min(pwm_value, self._pwm_max_range))
        w = 1.0 / (1.0 + np.exp((d - self._sigmoid_d0) / self._sigmoid_k))
        return w

    def _get_vl53l5cx_front_distance(self):
        '''
        Returns the median distance from the central 4 pixels of the VL53L5CX grid.
        Now reads the latest value from the queue (via Vl53l5cxSensor.get_distance_mm()).
        '''
        if not self._vl53l5cx.enabled:
            self._log.warning('VL53L5CX not enabled.')
            return None
        data = self._vl53l5cx.get_distance_mm()  # now non-blocking
        if data is None:
#           self._log.warning('VL53L5CX returned no data.')
            return None
        # Assume 8x8 grid, take rows 3,4 and cols 3,4 (0-indexed, for central 4 pixels)
        try:
            grid = np.array(data).reshape((8, 8))
            center_rows = [3, 4]  # middle two rows
            center_cols = [3, 4]  # middle two cols
            values = [grid[row, col] for row in center_rows for col in center_cols]
            # Filter out invalid readings (e.g., 0 or None)
            values = [v for v in values if v is not None and v > 0]
            if not values:
                self._log.warning('No valid VL53L5CX center values.')
                return None
            median = float(np.median(values))
            return median
        except Exception as e:
            self._log.error('Error extracting VL53L5CX center values: {}'.format(e))
            return None

    def _fuse(self, pwm_value, vl53_value):
        '''
        Fuses PWM and VL53L5CX sensor values according to fusion strategy.
        Returns fused value (mm) or None.
        '''
        if pwm_value is None and vl53_value is None:
            return None
        elif pwm_value is not None and vl53_value is None:
            return pwm_value
        elif pwm_value is None and vl53_value is not None:
            return vl53_value
        elif self._use_sigmoid:
            # return sigmoid fusion between PWM and VL53
            weight = self.sigmoid_weight(pwm_value)
            return weight * pwm_value + (1.0 - weight) * vl53_value
        else:
            # just return minimum value
            return min(pwm_value, vl53_value)

    def _smooth(self, value):
        '''
        Applies smoothing to the fused value if enabled.
        Returns smoothed value (mm) or None.
        '''
        if value is None:
            return None
        if not self._smoothing:
#           self._log.info(Fore.WHITE + 'unsmoothed: {:.2f}'.format(value))
            return value
        self._window.append(value)
        smoothed = np.mean(self._window)
#       self._log.info(Fore.WHITE + 'smoothed: {:.2f}'.format(smoothed))
        return smoothed

    def _normalise_and_ease(self, value):
        '''
        Clamps, normalises, eases, and rescales the value.
        Returns eased value (mm) or None.
        '''
        if value is None:
            return None
        clamped = min(value, self._max_distance)
        normalised = clamped / self._max_distance
        eased = self._easing.apply(normalised)
        eased_mm = eased * self._max_distance
#       self._log.info(Fore.WHITE + 'eased: {:.2f}'.format(eased_mm))
        return eased_mm

    def get_distance(self, apply_easing=True):
        '''
        Returns a fused, smoothed, and eased value for Roam behaviour.
        Tries to get a new value; if unavailable, returns previous value up to timeout.
        Returns None if value is stale.
        '''
        new_value = self._smooth(self._fuse(
            self._distance_sensor.get_distance(),
            self._get_vl53l5cx_front_distance()
        ))
        now = dt.now()
        if new_value is not None:
            self._last_value = new_value
            self._last_read_time = now
        elapsed_ms = (now - self._last_read_time).total_seconds() * 1000.0
        if self._last_value is not None and elapsed_ms < self._stale_timeout_ms:
            # return the last value (eased if requested)
            return self._last_value if not apply_easing else self._normalise_and_ease(self._last_value)
        else:
            # value is stale or never set
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
            self._distance_sensor.enable()
            self._vl53l5cx.enable()
            Component.enable(self)
            self._log.info('roam sensor enabled.')
        else:
            self._log.info('already enabled.')

    def disable(self):
        '''
        Disables both sensors.
        '''
        if self.enabled:
            if self._distance_sensor:
                self._distance_sensor.disable()
            if self._vl53l5cx:
                self._vl53l5cx.disable()
            Component.disable(self)
            self._log.info('roam sensor disabled.')
        else:
            self._log.info('already disabled.')

    def close(self):
        '''
        Closes both sensors and frees resources.
        '''
        self.disable()
        if not self.closed:
            if self._distance_sensor:
                self._distance_sensor.close()
            if self._vl53l5cx:
                self._vl53l5cx.close()
            Component.close(self)
            self._log.info('roam sensor closed.')
        else:
            self._log.info('already closed.')

#EOF
