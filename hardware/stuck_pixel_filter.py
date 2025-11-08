
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-08
# modified: 2025-11-08

import numpy as np
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level

class StuckPixelFilter(Component):
    '''
    Detects and filters VL53L5CX pixels that remain constant despite robot motion.
    
    Tracks pixel values over time and marks pixels as "stuck" if their variance
    remains below a threshold while the robot is moving. Automatically clears
    history when the robot stops to avoid false positives.
    
    Args:
        config: Application configuration dict
        motor_controller: Optional motor controller reference for motion detection
        level: Logging level
    '''
    NAME = 'stuck-pixel-filter'
    
    def __init__(self, config, motor_controller=None, level=Level.INFO):
        self._log = Logger(StuckPixelFilter.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        _cfg = config['kros'].get('hardware').get('stuck_pixel_filter')
        if _cfg is None:
            _cfg = {}  # use defaults if not configured
        self._motor_controller = motor_controller
        self._threshold = _cfg.get('threshold', 5)  # readings before considered stuck
        self._tolerance_mm = _cfg.get('tolerance_mm', 5)  # mm variance allowed
        self._min_motion_speed = _cfg.get('min_motion_speed', 0.05)  # minimum speed to consider "moving"
        # state
        self._pixel_history = {}  # {(row, col): [val1, val2, ...]}
        self._stuck_pixels = set()  # {(row, col), ...}
        self._log.info('😥 ready [threshold={}, tolerance={}mm, min_speed={}]'.format(
            self._threshold, self._tolerance_mm, self._min_motion_speed))
    
    @property
    def stuck_pixels(self):
        '''
        Returns set of (row, col) tuples currently marked as stuck.
        '''
        return self._stuck_pixels.copy()
    
    def is_stuck(self, row, col):
        '''
        Check if a specific pixel is currently stuck.
        '''
        return (row, col) in self._stuck_pixels
    
    def clear(self):
        '''
        Clear all history and stuck pixel markers.
        '''
        self._pixel_history.clear()
        self._stuck_pixels.clear()
        self._log.debug('cleared all stuck pixel history')
    
    def update(self, grid, rows_to_check=None, cols_to_check=None):
        '''
        Update stuck pixel detection with new sensor grid data.
        
        Args:
            grid: numpy array of shape (rows, cols) with distance readings
            rows_to_check: Optional list of row indices to check (default: all)
            cols_to_check: Optional list of column indices to check (default: all)
        '''
        if not self.enabled:
            return
        rows, cols = grid.shape
        if rows_to_check is None:
            rows_to_check = range(rows)
        if cols_to_check is None:
            cols_to_check = range(cols)
        # only analyze when robot is moving
        if not self._motor_controller.is_stopped:
            for row in rows_to_check:
                for col in cols_to_check:
                    val = grid[row, col]
                    if val is None or val == 0:
                        continue
                    key = (row, col)
                    # initialize history for this pixel
                    if key not in self._pixel_history:
                        self._pixel_history[key] = []
                    # add current reading
                    self._pixel_history[key].append(val)
                    # keep only recent history
                    if len(self._pixel_history[key]) > self._threshold:
                        self._pixel_history[key].pop(0)
                    # check if pixel is stuck (all values within tolerance)
                    if len(self._pixel_history[key]) >= self._threshold:
                        values = self._pixel_history[key]
                        variance = max(values) - min(values)
                        if variance <= self._tolerance_mm:
                            if key not in self._stuck_pixels:
                                self._stuck_pixels.add(key)
                                self._log.warning('stuck pixel detected: row={}, col={}, value={}mm (variance={}mm)'.format(
                                    row, col, val, variance))
                        else:
                            # pixel is varying, remove from stuck set
                            if key in self._stuck_pixels:
                                self._stuck_pixels.remove(key)
                                self._log.info('pixel recovered: row={}, col={}'.format(row, col))
        else:
            # robot stopped, clear history to avoid false positives on next movement
            if len(self._pixel_history) > 0:
                self._log.debug('robot stopped, clearing history')
                self.clear()
    
    def filter_values(self, values_with_coords):
        '''
        Filter out stuck pixels from a list of (value, row, col) tuples.
        
        Args:
            values_with_coords: List of (value, row, col) tuples
            
        Returns:
            List of (value, row, col) tuples with stuck pixels removed
        '''
        return [(val, row, col) for val, row, col in values_with_coords 
                if (row, col) not in self._stuck_pixels]

#EOF
