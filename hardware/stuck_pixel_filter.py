#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-08
# modified: 2025-11-29

import numpy as np
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level

class StuckPixelFilter(Component):
    NAME = 'stuck-pixel'
    '''
    Detects and filters VL53L5CX pixels that are unreliable. This includes:

    1. Real-time Zero Interpolation: On a per-frame basis, replaces sporadic
       zero-value pixels with the median of their valid neighbors. This handles
       transient dropouts.
    2. Stuck Pixel Detection: Over time, tracks pixels that remain constant
       despite robot motion and marks them as "stuck".

    Args:
        config: Application configuration dict
        level: Logging level
    '''
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
        self._verbose = False
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
        if self._verbose:
            self._log.info('cleared all stuck pixel history')

    def interpolate_zeros(self, grid):
        '''
        Performs real-time spatial filtering to replace zero-value pixels.

        For each pixel with a value of 0, its value is replaced by the median
        of its valid (non-zero) cardinal neighbors (up, down, left, right).
        This is a stateless operation performed on a single frame.

        Args:
            grid: A 2D numpy array representing the sensor data frame.

        Returns:
            A tuple containing:
            - A new 2D numpy array with zero-values interpolated.
            - The number of pixels that were corrected.
        '''
        if not self.enabled:
            return grid, 0

        rows, cols = grid.shape
        corrected_grid = np.copy(grid)
        corrected_count = 0
        
        # identify all zero-pixels first
        zero_pixels = np.argwhere(grid == 0)

        for row, col in zero_pixels:
            neighbors = []
            # check cardinal neighbors (up, down, left, right)
            if row > 0 and grid[row - 1, col] > 0:
                neighbors.append(grid[row - 1, col])
            if row < rows - 1 and grid[row + 1, col] > 0:
                neighbors.append(grid[row + 1, col])
            if col > 0 and grid[row, col - 1] > 0:
                neighbors.append(grid[row, col - 1])
            if col < cols - 1 and grid[row, col + 1] > 0:
                neighbors.append(grid[row, col + 1])

            if neighbors:
                # replace the zero value with the median of its neighbors
                median_value = int(np.median(neighbors))
                corrected_grid[row, col] = median_value
                corrected_count += 1
                if self._verbose:
                    self._log.info('interpolated pixel ({},{}): replaced 0 with median value {}'.format(row, col, median_value))

        if self._verbose:
            if corrected_count > 0:
                self._log.info('interpolated {} zero-value pixels.'.format(corrected_count))
        return corrected_grid, corrected_count

    def update(self, grid, rows_to_check=None, cols_to_check=None):
        '''
        Update stuck pixel detection with new sensor grid data. This tracks
        pixels that are stuck at a non-zero value over time.

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
                if val is None or val == 0: # ignore zeros, they are handled by interpolation
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
                            if self._verbose:
                                self._log.info('stuck pixel detected: row={}, col={}, value={}mm (unchanging)'.format(row, col, val))
                    else:
                        # pixel is varying, remove from stuck set
                        if key in self._stuck_pixels:
                            self._stuck_pixels.remove(key)
                            if self._verbose:
                                self._log.info('pixel recovered: row={}, col={}'.format(row, col))

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
