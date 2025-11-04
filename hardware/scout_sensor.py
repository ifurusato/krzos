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
        self._history_depth = _cfg.get('history_depth', 3)
        self._weights = np.array(_cfg.get('weights', [0.6, 0.3, 0.1]))
        self._rate = Rate(_cfg.get('loop_freq_hz', 5)) # 5Hz, 200ms default
        # variables
        self.set_visualiser(visualiser)
        self._thread  = None
        self._running = False
        self._heading_offset_degrees = 0.0
        self._max_open_distance = self._distance_threshold
        self._distance_history = []
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
        Uses temporal filtering to ignore sporadic noise - only persistent
        obstacles (appearing in consecutive frames) are considered real.
        
        Returns dict with analysis results for visualization.
        '''
        distance = np.array(distance_mm).reshape((self._rows, self._cols))
        # add current reading to history buffer
        self._distance_history.append(distance.copy())
        if len(self._distance_history) > self._history_depth:
            self._distance_history.pop(0)
        # compute angle for each column center (negative = left, positive = right)
        pixel_angles = [-(self._fov/2) + (i + 0.5) * (self._fov/self._cols) for i in range(self._cols)]
        # get obstacle rows (non-floor rows) from sensor calibration
        obstacle_rows = [r for r in range(self._rows) if self._vl53l5cx.floor_row_means[r] is None]
        # if not enough history yet, assume path is clear
        if len(self._distance_history) < self._history_depth:
            self._heading_offset_degrees = 0.0
            self._max_open_distance = self._distance_threshold
            return dict(
                weighted_avgs=[0]*self._cols,
                target_offset=0.0,
                heading_offset=0.0,
                highlighted_idx=self._cols//2,
                pixel_angles=pixel_angles
            )
        # create mask of persistent obstacles: cells that are < threshold in ALL recent frames
        history_array = np.array(self._distance_history)
        persistent_obstacles = np.all(history_array < self._distance_threshold, axis=0)
        # filter out floor reflections using calibrated floor data
        for row in obstacle_rows:
            for col in range(self._cols):
                if persistent_obstacles[row, col]:
                    val = distance[row, col]
                    # check if this reading matches any calibrated floor row
                    for floor_row in range(self._rows):
                        floor_mean = self._vl53l5cx.floor_row_means[floor_row]
                        if floor_mean is not None:
                            if abs(val - floor_mean) < self._vl53l5cx.floor_margin * 2:
                                persistent_obstacles[row, col] = False
                                break
        # check if any persistent obstacles exist in obstacle rows
        has_obstacle = False
        if obstacle_rows:
            for row in obstacle_rows:
                if np.any(persistent_obstacles[row, :]):
                    has_obstacle = True
                    break
        # if no persistent obstacles, path is clear
        if not has_obstacle:
            self._heading_offset_degrees = 0.0
            self._max_open_distance = np.mean(distance[obstacle_rows, :]) if obstacle_rows else self._distance_threshold
            return dict(
                weighted_avgs=[int(np.mean(distance[obstacle_rows, col])) if obstacle_rows else 0 for col in range(self._cols)],
                target_offset=0.0,
                heading_offset=0.0,
                highlighted_idx=self._cols//2,
                pixel_angles=pixel_angles
            )
        # obstacles detected - compute weighted average for each column
        weighted_avgs = []
        weights = self._weights[:len(obstacle_rows)] if obstacle_rows else [1.0]
        
        for col in range(self._cols):
            values = distance[obstacle_rows, col] if obstacle_rows else distance[:, col]
            if len(weights) == len(values):
                weighted_avgs.append(np.average(values, weights=weights))
            else:
                weighted_avgs.append(np.mean(values))
        # find the most open column
        self._max_open_distance = max(weighted_avgs)
        max_idx = int(np.argmax(weighted_avgs))
        target_offset = pixel_angles[max_idx]
        self._heading_offset_degrees = target_offset
        return dict(
            weighted_avgs=weighted_avgs,
            target_offset=target_offset,
            heading_offset=target_offset,
            highlighted_idx=max_idx,
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
