#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-12-03
# modified: 2025-12-30

from colors import*

class PixelCycler:
    def __init__(self, pixel, count):
        self.pixel = pixel
        self.count = count
        self._colors = [
            COLOR_RED,
            COLOR_DARK_CYAN,
            COLOR_DARK_CYAN,
            COLOR_DARK_CYAN
        ]
        self._color_index = 0
        self._pixel_index = 0
        self._phase = 0

    def step(self):
        '''
        Advance one step of the animation.
        '''
        color = self._colors[self._color_index]
        if self._phase == 0:
            self.pixel.set_color(index=self._pixel_index, color=color)
            self._phase = 1
        else:
            self.pixel.set_color(index=self._pixel_index, color=COLOR_BLACK)
            self._phase = 0
            self._pixel_index += 1
            if self._pixel_index >= self.count:
                self._pixel_index = 0
                self._color_index = (self._color_index + 1) % len(self._colors)

#EOF
