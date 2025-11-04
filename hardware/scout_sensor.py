#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-02
# modified: 2025-11-04

import sys
import time
from threading import Thread
import numpy as np
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level
from core.rate import Rate
from hardware.vl53l5cx_sensor import Vl53l5cxSensor

class ScoutSensor(Component):
    NAME = 'scout-sensor'
    '''
    ScoutSensor analyzes VL53L5CX multizone data to determine the most
    open direction, returning a heading offset in degrees for navigation.
    
    This is a stateless sensor - each reading is independent and based solely
    on current sensor data. Smoothing is handled by the consuming behavior (Scout).
    
    The sensor processes the 8x8 grid column-by-column, ignoring floor rows
    (detected during Vl53l5cxSensor calibration), and identifies the column
    with the greatest average distance as the most open path.
    
    The control loop runs in a thread when enabled, and can send display
    data to an external visualiser class via callback.
    
    If the Vl53l5cxSensor is not provided it will be created.
    '''
    def __init__(self, config, vl53l5cx=None, visualiser=None, level=Level.INFO):
        self._log = Logger(ScoutSensor.NAME, level=level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._log.info('initialising ScoutSensor…')
        if config is None or not isinstance(config, dict):
            raise ValueError('invalid configuration argument.')
        if vl53l5cx is None:
            # see if it's in the ComponentRegistry
            _component_registry = Component.get_registry()
            self._vl53l5cx = _component_registry.get(Vl53l5cxSensor.NAME)
            if self._vl53l5cx is None:
                # otherwise we make one
                self._vl53l5cx = Vl53l5cxSensor(config, level=Level.INFO)
        else:
            self._vl53l5cx = vl53l5cx
        # configuration
        _vl53_cfg = config['kros'].get('hardware').get('vl53l5cx_sensor')
        self._cols = _vl53_cfg.get('cols', 8)
        self._rows = _vl53_cfg.get('rows', 8)
        self._fov  = _vl53_cfg.get('fov', 47.0)
        _cfg = config['kros'].get('hardware').get('scout_sensor')
        self._distance_threshold = _cfg.get('distance_threshold', 1000)
        self._weights = np.array(_cfg.get('weights', [0.6, 0.3, 0.1]))
        self._rate = Rate(_cfg.get('loop_freq_hz', 5)) # 5Hz, 200ms default
        # variables
        self.set_visualiser(visualiser)
        self._thread  = None
        self._running = False
        self._heading_offset_degrees = 0.0
        self._max_open_distance = self._distance_threshold
        self._log.info('scout sensor ready.')

    def set_visualiser(self, visualiser):
        '''
        Set the optional visualiser used by the sensor.
        '''
        self._visualiser = visualiser
    
    def get_heading_offset(self):
        '''
        Returns the current heading offset and the distance at that heading.
        
        Returns:
            tuple: (heading_offset_degrees, distance_mm)
                - heading_offset_degrees: Negative = turn left (port), Positive = turn right (starboard)
                  Range: approximately -23.5° to +23.5° (half of horizontal FOV)
                  Returns 0.0 if no obstacles detected or path is clear ahead.
                - distance_mm: The weighted average distance in the direction of the heading offset
                  Returns large value (e.g., max distance) if path is clear
        '''
        return (self._heading_offset_degrees, self._max_open_distance)

    def _control_loop(self):
        '''
        Continuous polling loop that updates heading offset.
        '''
        while self._running:
            distance_mm = self.get_distance_mm()
            if distance_mm is not None:
                result = self._process(distance_mm)
                if self._visualiser is not None:
                    self._visualiser.update(distance_mm, self._vl53l5cx.floor_row_means, self._vl53l5cx.floor_margin, result)
                else:
                    self._log.debug('heading offset: ' + Fore.CYAN + '{:+.1f}°'.format(self._heading_offset_degrees))
            self._rate.wait()

    def get_distance_mm(self):
        '''
        Returns raw distance data from VL53L5CX sensor.
        '''
        if not self.enabled:
            self._log.warning('get_distance_mm called while ScoutSensor is not enabled.')
            return None
        return self._vl53l5cx.get_distance_mm()

    def _process(self, distance_mm):
        '''
        Analyzes distance data to determine most open direction.
        Points toward the column with greatest average distance.
        This is stateless - returns instantaneous reading based solely on current data.
        
        Row 0 = bottom/floor, Row 7 = top/far
        
        Returns dict with analysis results for visualization.
        '''
        distance = np.array(distance_mm).reshape((self._rows, self._cols))
        # compute angle for each column center (negative = left, positive = right)
        pixel_angles = [-(self._fov/2) + (i + 0.5) * (self._fov/self._cols) for i in range(self._cols)]
        # get obstacle rows (non-floor rows) from sensor calibration - same for all columns
        obstacle_rows = [r for r in range(self._rows) if self._vl53l5cx.floor_row_means[r] is None]
        # if no obstacle rows or no obstacles detected within threshold, path is clear
        if not obstacle_rows or not np.any(distance[obstacle_rows, :] < self._distance_threshold):
            self._heading_offset_degrees = 0.0
            self._max_open_distance = self._distance_threshold
            weighted_avgs = [0] * self._cols
            highlighted_idx = min(range(self._cols), key=lambda i: abs(pixel_angles[i]))
            return dict(
                weighted_avgs=weighted_avgs,
                target_offset=0.0,
                heading_offset=0.0,
                highlighted_idx=highlighted_idx,
                pixel_angles=pixel_angles
            )
        # compute weighted average distance for each column (using obstacle rows only)
        weighted_avgs = []
        weights = self._weights[:len(obstacle_rows)] if len(self._weights) >= len(obstacle_rows) else np.ones(len(obstacle_rows))
        for col in range(self._cols):
            # compute weighted average from obstacle rows
            values = distance[obstacle_rows, col]
            avg = np.average(values, weights=weights)
            # critical: check floor rows for obstacles peeking through
            for floor_row in [r for r in range(self._rows) if self._vl53l5cx.floor_row_means[r] is not None]:
                floor_mean = self._vl53l5cx.floor_row_means[floor_row]
                floor_value = distance[floor_row, col]
                # obstacle detected if value is significantly less than calibrated floor mean
                if floor_value < (floor_mean - self._vl53l5cx.floor_margin):
                    avg = min(avg, floor_value)
                    self._log.debug('obstacle in floor row {}, col {}: {}mm < {}mm'.format(
                        floor_row, col, floor_value, floor_mean - self._vl53l5cx.floor_margin))
            weighted_avgs.append(avg)
        # store the maximum (most open) distance
        self._max_open_distance = max(weighted_avgs)
        weighted_avgs_array = np.array(weighted_avgs)
        # find columns that are "open enough" (within 20% of max)
        threshold = self._max_open_distance * 0.8
        open_columns = [i for i, dist in enumerate(weighted_avgs) if dist >= threshold]
        # among open columns, choose the one closest to straight ahead (0°)
        # this provides stability - robot prefers to go straight when multiple options exist
        if open_columns:
#           max_idx = min(open_columns, key=lambda i: abs(pixel_angles[i]))
            if len(open_columns) >= self._cols - 1:  # all or nearly all columns are open
                target_offset = 0.0  # path is clear, go straight
                max_idx = min(range(self._cols), key=lambda i: abs(pixel_angles[i]))  # highlight column closest to center
            else:
                max_idx = min(open_columns, key=lambda i: abs(pixel_angles[i]))
                target_offset = pixel_angles[max_idx]
        else:
            # fallback: just use the absolute max
            max_idx = int(np.argmax(weighted_avgs))
        # target offset is the angle of the chosen column - no filtering, return instantaneous value
        target_offset = pixel_angles[max_idx]
        self._heading_offset_degrees = target_offset
        # for visualization: highlight the chosen column
        highlighted_idx = max_idx
        return dict(
            weighted_avgs=weighted_avgs,
            target_offset=target_offset,
            heading_offset=self._heading_offset_degrees,
            highlighted_idx=highlighted_idx,
            pixel_angles=pixel_angles
        )

    def enable(self):
        '''
        Enables the sensor and starts the processing thread.
        '''
        if not self.enabled:
            self._vl53l5cx.enable()
            super().enable()
            self._log.info('enabled and sensor ranging.')
            self._running = True
            self._thread = Thread(target=self._control_loop, daemon=True)
            self._thread.start()
        else:
            self._log.info('already enabled.')

    def disable(self):
        '''
        Disables the sensor and stops the processing thread.
        '''
        self._running = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        if self.enabled:
            self._vl53l5cx.disable()
            super().disable()
            self._log.info('disabled and sensor stopped ranging.')
        else:
            self._log.info('already disabled.')

    def close(self):
        '''
        Closes the sensor and frees resources.
        '''
        self.disable()
        if not self.closed:
            self._vl53l5cx.close()
            super().close()
            self._log.info('closed.')
        else:
            self._log.info('already closed.')

#EOF
