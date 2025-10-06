#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2019-12-23
# modified: 2025-08-30

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
        distance = np.array(distance_mm).reshape((len(floor_row_means), self.cols))
        self.print_colored_grid(distance, self.cols, floor_row_means, margin)
        self.print_target_row(
            result['weighted_avgs'],
            result['highlighted_idx'],
            result['pixel_angles'],
            result['target_offset'],
            result['filtered_offset'],
            result['port_mult'],
            result['stbd_mult']
        )

    def print_colored_grid(self, distance, COLS, floor_row_means, margin):
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

    def print_target_row(self, weighted_avgs, highlighted_idx, pixel_angles, target_offset, filtered_offset, port_mult, stbd_mult):
        line = ""
        for i, val in enumerate(weighted_avgs):
            if highlighted_idx is not None and i == highlighted_idx:
                color = Fore.WHITE + Style.BRIGHT
            else:
                color = Fore.BLACK
            line += "{}{:4d}{} ".format(color, int(val), Style.RESET_ALL)
        print(line)
        self._log.info("target offset: {:+.2f}°; filtered: {:+.2f}°".format(target_offset, filtered_offset))
        self._log.info('port multiplier: ' + Fore.RED + '{:.2f}'.format(port_mult)
                + Fore.CYAN + '; stbd multiplier: ' + Fore.GREEN + '{:.2f}'.format(stbd_mult))

#EOF
