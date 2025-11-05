#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-02
# modified: 2025-11-05

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
        self._verbose = _cfg.get('verbose', False)
        # variables
        self.set_visualiser(visualiser)
        self._thread  = None
        self._running = False
        self._heading_offset_degrees = 0.0
        self._min_obstacle_distance = self._distance_threshold
        self._max_open_distance = self._distance_threshold
        self._distance_history = []
        self._log.info('scout sensor ready.')

    @property
    def distance_threshold(self):
        return self._distance_threshold

    def set_visualiser(self, visualiser):
        '''
        Set the optional visualiser used by the sensor.
        '''
        self._visualiser = visualiser

    def get_heading_offset(self):
        '''
        returns current heading offset and max open distance.

        returns:
            tuple: (heading_offset_degrees, max_open_distance_mm)
                - heading_offset_degrees: negative = turn port, positive = turn starboard
                  range: approximately -23.5° to +23.5° (half of horizontal fov)
                  returns 0.0° if no obstacles detected or path is clear ahead.
                - max_open_distance_mm: the weighted average distance in the direction of the heading offset
                  returns large value (distance_threshold) if path is clear
        '''
        return (self._heading_offset_degrees, self._max_open_distance)

    def get_distance_mm(self):
        '''
        Returns raw distance data from VL53L5CX sensor.
        '''
        if not self.enabled:
            self._log.warning('get_distance_mm called while ScoutSensor is not enabled.')
            return None
        return self._vl53l5cx.get_distance_mm()

    def _control_loop(self):
        '''
        Continuous polling loop that updates heading offset.
        '''
        while self._running:
            distance_mm = self.get_distance_mm()
            if distance_mm is not None:
                result = self._process()
                if self._visualiser is not None:
                    self._visualiser.update(distance_mm, self._vl53l5cx.floor_row_means, self._vl53l5cx.floor_margin, result)
                else:
                    self._log.debug('heading offset: ' + Fore.CYAN + '{:+.1f}°'.format(self._heading_offset_degrees))
            self._rate.wait()

    def _calculate_weighted_column_averages(self):
        '''
        Calculate weighted average distance for each column using non-floor rows.
        Ignores zero values and uses weights for row priority.
        Returns list of 8 averaged distances (mm).
        '''
        distance_mm = self.get_distance_mm()
        if distance_mm is None:
            return [self._distance_threshold] * self._cols
        distance = np.array(distance_mm).reshape((self._rows, self._cols))
        # get non-floor rows, fallback to upper half if calibration failed
        obstacle_rows = self._vl53l5cx.non_floor_rows
        if not obstacle_rows:
            obstacle_rows = list(range(self._rows // 2, self._rows))
        # calculate weighted average for each column
        weighted_avgs = []
        for col in range(self._cols):
            values = []
            weights = []
            for idx, row in enumerate(obstacle_rows):
                val = distance[row, col]
                # ignore zero values (sensor noise)
                if val > 0:
                    values.append(val)
                    # weight by row index (higher rows = more weight)
                    if idx < len(self._weights):
                        weights.append(self._weights[idx])
                    else:
                        weights.append(0.1)  # minimal weight for extra rows
            if values:
                if weights:
                    weighted_avgs.append(np.average(values, weights=weights[:len(values)]))
                else:
                    weighted_avgs.append(np.mean(values))
            else:
                # no valid readings in this column - assume threshold
                weighted_avgs.append(self._distance_threshold)
        return weighted_avgs

    def _process(self):
        '''
        Process VL53L5CX data and calculate recommended heading offset.
        Returns weighted average heading based on all open columns for smooth navigation.
        '''
        weighted_avgs = self._calculate_weighted_column_averages()
        # calculate pixel angles for each column center
        pixel_angles = [-(self._fov/2) + (i + 0.5) * (self._fov/self._cols) for i in range(self._cols)]
        # find minimum distance (for diagnostics)
        self._min_obstacle_distance = min(weighted_avgs)
        # if all columns are open (>= threshold), no heading change needed
        if self._min_obstacle_distance >= self._distance_threshold:
            self._heading_offset_degrees = 0.0
            self._max_open_distance = self._distance_threshold
            target_offset = 0.0
            highlighted_idx = self._cols // 2  # center columns
            if self._verbose:
                print("SCOUT SENSOR: No obstacles, offset=0.0°, min_dist={:.0f}mm".format(self._min_obstacle_distance))
            return dict(
                weighted_avgs=weighted_avgs,
                target_offset=target_offset,
                heading_offset=target_offset,
                highlighted_idx=highlighted_idx,
                pixel_angles=pixel_angles)
        # obstacles present - calculate weighted average heading toward openness
        # only use columns above threshold for steering
        weights = []
        angles = []
        open_indices = []
        for i, dist in enumerate(weighted_avgs):
            if dist >= self._distance_threshold:
                # open column - weight by how much more open than threshold
                weight = dist - self._distance_threshold + 1.0
                weights.append(weight)
                angles.append(pixel_angles[i])
                open_indices.append(i)
        if weights:
            # calculate weighted average angle toward openness
            total_weight = sum(weights)
            weighted_angle = sum(a * w for a, w in zip(angles, weights)) / total_weight
            target_offset = weighted_angle
            self._heading_offset_degrees = weighted_angle
            self._max_open_distance = max(weighted_avgs)
            # highlight the most open column for visualization
            highlighted_idx = open_indices[weights.index(max(weights))]
        else:
            # no open columns - steer toward least obstructed
            highlighted_idx = int(np.argmax(weighted_avgs))
            target_offset = pixel_angles[highlighted_idx]
            self._heading_offset_degrees = target_offset
            self._max_open_distance = weighted_avgs[highlighted_idx]
        if self._verbose:
            print("SCOUT SENSOR: offset={:+.2f}°, min_dist={:.0f}mm, max_dist={:.0f}mm".format(
                self._heading_offset_degrees, self._min_obstacle_distance, self._max_open_distance))
        return dict(
            weighted_avgs=weighted_avgs,
            target_offset=target_offset,
            heading_offset=target_offset,
            highlighted_idx=highlighted_idx,
            pixel_angles=pixel_angles)

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
