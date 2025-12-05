#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-12-03
# modified: 2025-12-03

from colors import*

class PixelCycler:
    def __init__(self, pixel, count):
        self.pixel = pixel
        self.count = count
#       self._colors = [
#           COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW
#       ]
        self._colors = [
            COLOR_RED,
            COLOR_DARK_CYAN,
            COLOR_DARK_CYAN,
            COLOR_DARK_CYAN
        ]
        self._color_index = 0
        self._pixel_index = 0
        self._phase = 0   # 0 = turn ON, 1 = turn OFF

    def step(self):
        """
        Advance exactly one step of the animation.
        Call this from your Timer callback.
        """
        color = self._colors[self._color_index]
        if self._phase == 0:
            # turn pixel ON
            self.pixel.set_color(index=self._pixel_index, color=color)
            self._phase = 1
        else:
            # turn pixel OFF
            self.pixel.set_color(index=self._pixel_index, color=COLOR_BLACK)
            self._phase = 0
            # move to next pixel after turning off
            self._pixel_index += 1
            if self._pixel_index >= self.count:
                self._pixel_index = 0
                self._color_index = (self._color_index + 1) % len(self._colors)

#EOF
