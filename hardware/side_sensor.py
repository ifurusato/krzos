#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-25
# modified: 2025-11-02
#
# Provides fused distance readings for sensing directly sideways.

import time
from datetime import datetime as dt
import numpy as np
from collections import deque
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.cardinal import Cardinal
from core.component import Component
from core.orientation import Orientation
from hardware.distance_sensor import DistanceSensor
from hardware.distance_sensors import DistanceSensors

class SideSensor(Component):
    '''
    SideSensor fuses the side-facing near-field PWM-based DistanceSensors
    with one of the VL53L0X ToF sensors from Radiozoa to provide a single
    side-facing distance value optimized for obstacle avoidance.

    The fusion optionally uses a sigmoid weighting: the PWM sensor is
    prioritized for close range (0–max_range), VL53L0X for mid-to-far
    range. Smoothing is optionally applied to stabilize the output.

    The RadiozoaSensor and DistanceSensor(s) can already exist in the
    Component Registry or be passed as arguments.

    :param config:             the application configuration
    :param radiozoa_sensor:    optional radiozoa sensor instance
    :param level:              the log level
    '''
    def __init__(self, config, orientation=None, radiozoa_sensor=None, distance_sensors=None, level=Level.INFO):
        if not orientation:
            raise ValueError('missing required orientation argument.')
        self._orientation = orientation
        self._name = '{}-side-sensor'.format(orientation.label)
        self._log = Logger(self._name, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        if config is None or not isinstance(config, dict):
            raise ValueError('invalid configuration argument.')
        _cfg = config['kros'].get('hardware').get('side_sensor')
        if _cfg is None:
            raise ValueError('missing kros.hardware.side_sensor config section.')
        self._smoothing      = _cfg.get('smoothing', True)
        _smoothing_window    = _cfg.get('smoothing_window', 5)
        self._window         = deque(maxlen=_smoothing_window) if self._smoothing else None
        self._min_distance   = _cfg.get('min_distance')   # mm
        self._max_distance   = _cfg.get('max_distance')   # mm, the "no obstacle" threshold
        self._pwm_max_range  = _cfg.get('pwm_max_range')  # PWM sensor range in mm for fusion
        self._stale_timeout_ms = _cfg.get('stale_timeout_ms', 100) # 100ms default
        # we use roam's config for fusion
        _roam_cfg = config['kros'].get('hardware').get('roam_sensor')
        if _roam_cfg is None:
            raise ValueError('missing kros.hardware.roam_sensor config section.')
        # sigmoid fusion parameters
        self._sigmoid_d0     = _roam_cfg.get('sigmoid_d0', 150)  # inflection point (mm)
        self._sigmoid_k      = _roam_cfg.get('sigmoid_k', 40)    # steepness
        self._use_sigmoid    = _roam_cfg.get('use_sigmoid_fusion')
        self._use_radiozoa   = _roam_cfg.get('use_radiozoa')
        # sensor instantiation
        _component_registry = Component.get_registry()
        self._proximity_sensor = None
        if self._use_radiozoa:
            from hardware.radiozoa_sensor import RadiozoaSensor

            # Vl53l0x sensors, obtained from Radiozoa ┈┈┈┈┈┈
            if radiozoa_sensor is None:
                radiozoa_sensor = _component_registry.get(RadiozoaSensor.NAME)
            if radiozoa_sensor is not None:
                if orientation is Orientation.PORT:
                    self._proximity_sensor = radiozoa_sensor.get_sensor(Cardinal.WEST)
                elif orientation is Orientation.STBD:
                    self._proximity_sensor = radiozoa_sensor.get_sensor(Cardinal.EAST)
                else:
                    raise ValueError('required EAST or WEST orientation, not {}'.format(orientation.name))
            else:
                raise ValueError('radiozoa sensor not available.')
        # get distance sensor for orientation ┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._distance_sensor = None
        if distance_sensors is not None:
            # pass in constructor argument
            self._distance_sensor = distance_sensors.get_sensor(orientation)
        else:
            # see if it's in component registry
            self._distance_sensor = _component_registry.get("distance-{}".format(orientation.label))
        if not self._distance_sensor:
            # otherwise create it
            self._distance_sensor = DistanceSensor(config, orientation, level=level)
        if not self._distance_sensor.enabled:
            self._distance_sensor.enable()
            time.sleep(0.5)
        self._last_value     = None
        self._last_read_time = dt.now()
        self._log.info('roam sensor instantiated [sigmoid_d0={}, sigmoid_k={}, smoothing={}, window={}]'
                .format(self._sigmoid_d0, self._sigmoid_k, self._smoothing, _smoothing_window))

    @property
    def name(self):
        return self._name

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

    def _fuse(self, pwm_value, vl53_value):
        '''
        Fuses PWM and VL53L0X sensor values according to fusion strategy.
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
            return value
        self._window.append(value)
        smoothed = np.mean(self._window)
        return smoothed

    def get_distance(self):
        '''
        Returns a fused and smoothed value. Tries to get a new value; if
        unavailable, returns previous value up to timeout, returning None
        if the value is stale.
        '''
        if  self._proximity_sensor:
            new_value = self._smooth(self._fuse(
                self._distance_sensor.get_distance(),
                self._proximity_sensor.get_distance()
            ))
        else:
            new_value = self._distance_sensor.get_distance()
        now = dt.now()
        if new_value is not None:
            self._last_value = new_value
            self._last_read_time = now
        elapsed_ms = (now - self._last_read_time).total_seconds() * 1000.0
        if self._last_value is not None and elapsed_ms < self._stale_timeout_ms:
            return self._last_value
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
            if self._proximity_sensor:
                if not self._proximity_sensor.enabled:
                    self._proximity_sensor.enable()
                if not self._proximity_sensor.is_ranging():
                    self._proximity_sensor.start_ranging()
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
            if self._proximity_sensor:
                if self._proximity_sensor:
                    self._proximity_sensor.disable()
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
            if self._proximity_sensor:
                self._proximity_sensor.close()
            Component.close(self)
            self._log.info('roam sensor closed.')
        else:
            self._log.info('already closed.')

#EOF
