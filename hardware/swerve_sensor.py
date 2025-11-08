#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-07
# modified: 2025-11-07

import time
import numpy as np
import itertools
from collections import deque
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component, MissingComponentError
from hardware.vl53l5cx_sensor import Vl53l5cxSensor

class SwerveSensor(Component):
    NAME = 'swerve-sensor'
    '''
    SwerveSensor extracts port and starboard side distance readings from the
    VL53L5CX 8x8 grid for lateral obstacle avoidance.

    Uses the outer columns of the sensor grid (port: cols 0-2, starboard: cols 5-7)
    and ignores the center columns (3-4) which are used by RoamSensor.

    This provides the Swerve behavior with side awareness for lateral avoidance
    without requiring additional hardware sensors.

    :param config:             the application configuration
    :param vl53l5cx_sensor:    optional VL53L5CX sensor instance (shared with RoamSensor)
    :param level:              the log level
    '''
    def __init__(self, config, vl53l5cx_sensor=None, level=Level.INFO):
        self._log = Logger(SwerveSensor.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        if config is None or not isinstance(config, dict):
            raise ValueError('invalid configuration argument.')
        _cfg = config['kros'].get('hardware').get('swerve_sensor')
        if _cfg is None:
            raise ValueError('missing kros.hardware.swerve_sensor config section.')
        # column configuration
        self._port_columns = _cfg.get('port_columns', [0, 1, 2])
        self._stbd_columns = _cfg.get('stbd_columns', [5, 6, 7])
        # get VL53L5CX flip_horizontal setting
        _vl53_cfg = config['kros'].get('hardware').get('vl53l5cx_sensor')
        _flip_horizontal = _vl53_cfg.get('flip_horizontal', False)
        # swap columns if sensor is horizontally flipped
        if _flip_horizontal:
            self._port_columns, self._stbd_columns = self._stbd_columns, self._port_columns
            self._log.info('🍏 sensor flip horizontal: swapped port/stbd columns')
        # distance thresholds
        self._min_distance = _cfg.get('min_distance', 100)  # mm
        self._max_distance = _cfg.get('max_distance', 500)  # mm
        self._additional_floor_rows = _cfg.get('additional_floor_rows', 1)  # extra margin beyond calibration
        self._exclude_rows = _cfg.get('exclude_rows', [])  # rows to exclude from sampling
        # smoothing
        self._smoothing = _cfg.get('smoothing', True)
        _smoothing_window = _cfg.get('smoothing_window', 3)
        self._port_window = deque(maxlen=_smoothing_window) if self._smoothing else None
        self._stbd_window = deque(maxlen=_smoothing_window) if self._smoothing else None
        self._verbose = False
        self._counter = itertools.count()
        # computed during enable() after VL53L5CX calibration
        self._non_floor_rows = None
        # get or create VL53L5CX sensor
        _component_registry = Component.get_registry()
        if vl53l5cx_sensor is not None:
            self._vl53l5cx = vl53l5cx_sensor
            self._log.info('using provided VL53L5CX sensor.')
        else:
            self._vl53l5cx = _component_registry.get(Vl53l5cxSensor.NAME)
            if self._vl53l5cx is None:
                raise MissingComponentError('VL53L5CX sensor not available.')
            self._log.info('using existing VL53L5CX sensor from registry.')
        self._log.info('swerve sensor ready [port cols: {}, stbd cols: {}, smoothing: {}]'.format(
            self._port_columns, self._stbd_columns, self._smoothing))

    @property
    def name(self):
        return SwerveSensor.NAME

    @property
    def min_distance(self):
        return self._min_distance

    @property
    def max_distance(self):
        return self._max_distance

    def _get_side_distance(self, columns):
        '''
        Returns the minimum distance from the specified columns using non-floor rows.
        Uses pre-computed non-floor rows that exclude VL53L5CX's calibrated floor rows
        plus an additional safety margin to reduce false positives from textured surfaces.

        Args:
            columns: List of column indices to sample

        Returns:
            float: Minimum distance in mm, or None if no valid readings
        '''
        if not self._vl53l5cx.enabled:
            self._log.warning('VL53L5CX not enabled.')
            return None
        if self._non_floor_rows is None:
            self._log.warning('non-floor rows not yet computed; call enable() first.')
            return None
        data = self._vl53l5cx.get_distance_mm()
        if data is None:
            return None
        try:
            grid = np.array(data).reshape((8, 8))
            # sample specified columns across pre-computed non-floor rows
            values = [grid[row, col] for row in self._non_floor_rows for col in columns]
            # filter out invalid readings and enforce min_distance threshold
            values = [v for v in values if v is not None and v > self._min_distance]
            if not values:
                return None
            # return minimum distance (closest obstacle)
            min_distance = float(np.min(values))
            return min_distance
        except Exception as e:
            self._log.error('error extracting side distance: {}'.format(e))
            return None

    def _smooth(self, value, window):
        '''
        Applies smoothing to the value if enabled.

        Args:
            value: Raw distance value
            window: deque to use for smoothing

        Returns:
            float: Smoothed value or original if smoothing disabled
        '''
        if not self._smoothing:
            return value
        elif value is None:
            window.clear()  # ← Clear stale data when no valid reading
            return None
        window.append(value)
        smoothed = np.mean(window)
        return smoothed

    def get_side_distances(self):
        '''
        Returns port and starboard side distances from VL53L5CX outer columns.

        Returns:
            tuple: (port_distance, stbd_distance) in mm, or None for either if invalid
        '''
        port_distance = self._get_side_distance(self._port_columns)
        stbd_distance = self._get_side_distance(self._stbd_columns)

        # apply smoothing
        if self._smoothing:
            port_distance = self._smooth(port_distance, self._port_window)
            stbd_distance = self._smooth(stbd_distance, self._stbd_window)

        # diagnostic output
        if self._verbose and next(self._counter) % 10 == 0:
            self._log.info('side distances: port={}, stbd={}'.format(
                '{:.0f}mm'.format(port_distance) if port_distance else 'None',
                '{:.0f}mm'.format(stbd_distance) if stbd_distance else 'None'))

        return (port_distance, stbd_distance)

    def enable(self):
        if not self.enabled:
            if not self._vl53l5cx.enabled:
                self._log.info('VL53L5CX not enabled, enabling now…')
                self._vl53l5cx.enable()
            # compute non-floor rows once after VL53L5CX calibration
            calibrated_floor_rows = self._vl53l5cx.floor_rows
            max_floor_row = max(calibrated_floor_rows) if calibrated_floor_rows else -1
            # extend floor rows by adding additional margin rows
            extended_floor_rows = list(range(max_floor_row + 1 + self._additional_floor_rows))
            # compute non-floor rows (everything above extended floor)
            self._non_floor_rows = [r for r in range(8) if r not in extended_floor_rows]
            # exclude configured problematic rows
            if self._exclude_rows:
                original_rows = self._non_floor_rows.copy()
                self._non_floor_rows = [r for r in self._non_floor_rows if r not in self._exclude_rows]
                excluded = set(original_rows) - set(self._non_floor_rows)
                if excluded:
                    self._log.info('excluding rows {} from side distance sampling (configured as unreliable)'.format(sorted(excluded)))
            if not self._non_floor_rows:
                self._log.error('no non-floor rows available after adding {} margin row(s) and excluding {}'.format(
                    self._additional_floor_rows, self._exclude_rows))
            else:
                self._log.info('computed non-floor rows: {} (calibrated floor: {}, extended floor: {}, excluded: {})'.format(
                    self._non_floor_rows, calibrated_floor_rows, extended_floor_rows, self._exclude_rows))
            Component.enable(self)
            self._log.info('swerve sensor enabled.')
        else:
            self._log.info('already enabled.')

    def disable(self):
        if self.enabled:
            Component.disable(self)
            self._log.info('swerve sensor disabled.')
        else:
            self._log.info('already disabled.')

    def close(self):
        self.disable()
        if not self.closed:
            Component.close(self)
            self._log.info('swerve sensor closed.')
        else:
            self._log.info('already closed.')

#EOF
