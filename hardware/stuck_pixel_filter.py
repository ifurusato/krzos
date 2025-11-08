
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
    
    Tracks pixel values over time and marks pixels as "stuck" if they don't
    change at all across multiple sensor polls, this only occurring when the
    robot is moving. Automatically clears history when the robot stops to avoid
    false positives.
    
    Args:
        config: Application configuration dict
        level: Logging level
    '''
    NAME = 'stuck-pixel-filter'
    
    def __init__(self, config, level=Level.INFO):
        self._log = Logger(StuckPixelFilter.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        _cfg = config['kros'].get('hardware').get('stuck_pixel_filter')
        if _cfg is None:
            _cfg = {}  # use defaults if not configured
        self._threshold = _cfg.get('threshold', 5)  # readings before considered stuck
        # state
        self._pixel_history = {}  # {(row, col): [val1, val2, ...]}
        self._stuck_pixels = set()  # {(row, col), ...}
        self._log.info('ready [threshold={}]'.format(self._threshold))
    
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
                # check if pixel is stuck (all values exactly the same)
                if len(self._pixel_history[key]) >= self._threshold:
                    values = self._pixel_history[key]
                    # pixel is stuck if all values are identical
                    if len(set(values)) == 1:
                        if key not in self._stuck_pixels:
                            self._stuck_pixels.add(key)
                            self._log.debug('stuck pixel detected: row={}, col={}, value={}mm (unchanging)'.format(
                                row, col, val))
                    else:
                        # pixel is varying, remove from stuck set
                        if key in self._stuck_pixels:
                            self._stuck_pixels.remove(key)
                            self._log.debug('pixel recovered: row={}, col={}'.format(row, col))
    
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
