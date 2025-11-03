#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2019-12-23
# modified: 2025-11-03

import numpy as np
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level

class OpenPathVisualiser(Component):
    '''
    Handles console display of open path sensor data.
    '''
    DIST_COLORS = [
        (0,     150,  Fore.RED),
        (151,   300,  Fore.YELLOW),
        (301,   500,  Fore.GREEN),
        (501,   800,  Fore.BLUE),
        (801,  1000,  Fore.BLACK),
        (1001, 99999, Fore.BLACK + Style.DIM),
    ]
    FLOOR_COLOR = "\033[35;1m"  # Bright magenta

    def __init__(self, cols, rows):
        self._log = Logger('display', level=Level.INFO)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self.cols = cols
        self.rows = rows
        self._log.info('ready.')

    def get_dist_color(self, val):
        for low, high, color in self.DIST_COLORS:
            if low <= val <= high:
                return color
        return "\033[30m"

    def update(self, distance_mm, floor_row_means, margin, result):
        '''
        Updates the display with current sensor data and heading analysis.
        
        Args:
            distance_mm: Flattened 64-element distance array
            floor_row_means: List of floor row mean values (None for non-floor rows)
            margin: Floor detection margin in mm
            result: Dict containing analysis results from OpenPathSensor._process()
        '''
        distance = np.array(distance_mm).reshape((len(floor_row_means), self.cols))
        self.print_colored_grid(distance, self.cols, floor_row_means, margin)
        self.print_target_row(
            result['weighted_avgs'],
            result['highlighted_idx'],
            result['pixel_angles'],
            result['target_offset'],
            result['filtered_offset'],
            result.get('heading_offset', 0.0)  # NEW: heading offset in degrees
        )

    def print_colored_grid(self, distance, COLS, floor_row_means, margin):
        '''
        Prints the 8x8 distance grid with color coding:
        - Magenta: Floor rows (detected during calibration)
        - Red/Yellow/Green/Blue: Obstacle distances
        - Black: Far/no obstacle
        '''
        print("\033[1m" + "{}".format("┈" * (COLS * 6)) + "\033[0m")
        self._log.debug("floor_row_means = {}".format(floor_row_means))
        self._log.debug("Floor rows (indices): {}".format([i for i, v in enumerate(floor_row_means) if v is not None]))
        for row in reversed(range(distance.shape[0])):
            line = ""
            for col in range(distance.shape[1]):
                val = distance[row, col]
                floor_mean = floor_row_means[row]
                # Only color as floor for values in floor rows, unless obstacle
                if floor_mean is not None:
                    if val >= (floor_mean - margin):
                        color = self.FLOOR_COLOR
                    else:
                        color = self.get_dist_color(val)  # obstacle color
                else:
                    color = self.get_dist_color(val)
                line += "{}{:4d}{} ".format(color, val, Style.RESET_ALL)
            print(line)
        print(Style.RESET_ALL)

    def print_target_row(self, weighted_avgs, highlighted_idx, pixel_angles, target_offset, filtered_offset, heading_offset):
        '''
        Prints the weighted average row showing which column is most open,
        and displays the calculated heading offset.
        
        Args:
            weighted_avgs: List of weighted average distances per column
            highlighted_idx: Index of the most open column
            pixel_angles: List of angles corresponding to each column
            target_offset: Raw calculated offset (before filtering)
            filtered_offset: Low-pass filtered offset
            heading_offset: Final heading offset in degrees (negative=left, positive=right)
        '''
        line = ""
        for i, val in enumerate(weighted_avgs):
            if highlighted_idx is not None and i == highlighted_idx:
                color = Fore.WHITE + Style.BRIGHT
            else:
                color = Fore.BLACK
            line += "{}{:4d}{} ".format(color, int(val), Style.RESET_ALL)
        print(line)
        
        # Display heading information with directional color coding
        if heading_offset < -2.0:
            direction_color = Fore.RED  # Strong left
            direction = "← LEFT"
        elif heading_offset > 2.0:
            direction_color = Fore.GREEN  # Strong right
            direction = "RIGHT →"
        else:
            direction_color = Fore.CYAN  # Near center
            direction = "CENTER"
        
        self._log.info("target: {:+.2f}°; filtered: {:+.2f}°; {}heading: {:+.2f}° {}".format(
            target_offset, filtered_offset, direction_color, heading_offset, direction + Style.RESET_ALL))

#EOF
