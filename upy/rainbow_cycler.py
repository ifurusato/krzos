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

from pixel import Pixel
from colors import*

class RainbowCycler:
    '''
    Unicorns fart rainbows around the ring.
    '''
    def __init__(self, pixel, count, hue_step=0.002):
        '''
        :param pixel:  your pixel controller instance
        :param count:  number of LEDs
        :param hue_step: how much to advance the hue for each rotation (0–1)
        '''
        self._enabled = True
        self._pixel = pixel
        self._count = count

        self._hue = 0.0
        self._hue_step = hue_step
        self._pixel_index = 0
        self._phase = 0   # 0 = ON, 1 = OFF

    def step(self):
        '''
        One non-blocking step.
        Call this from your Timer.
        '''
        if not self._enabled:
            return

        # Compute current color from current hue
        color = Pixel.hsv_to_rgb(self._hue)

        if self._phase == 0:
            # Turn pixel ON with rainbow hue
            self._pixel.set_color(index=self._pixel_index, color=color)
            self._phase = 1

        else:
            # Turn pixel OFF
            self._pixel.set_color(index=self._pixel_index, color=COLOR_BLACK)
            self._phase = 0

            # Move to next pixel
            self._pixel_index += 1

            # Finished a rotation?
            if self._pixel_index >= self._count:
                self._pixel_index = 0

                # Advance hue each completed rotation
                self._hue += self._hue_step
                if self._hue >= 1.0:
                    self._hue -= 1.0

    def close(self):
        self._enabled = False
        for idx in range(24):
            self._pixel.set_color(index=idx, color=COLOR_BLACK)
        print('closed.')

#EOF
