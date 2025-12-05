#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-12-02
# modified: 2025-12-03

from colors import*

class BlinkPattern:
    def __init__(self, pixel, count, offset=0, auto_rotate=False):
        '''
        :param pixel:       pixel controller instance
        :param count:       number of LEDs
        :param offset:      initial index of the red pixel (0–count-1)
        :param auto_rotate: if True, offset increments after each full cycle
        '''
        self.pixel       = pixel
        self.count       = count
        self.offset      = offset % count
        self.auto_rotate = auto_rotate
        self.pixel_index = 0
        self.phase       = 0   # 0 = ON, 1 = OFF

    def step(self):
        '''
        One non-blocking step of the blinking animation.
        A callback from the Timer.
        '''
        # apply offset: logical pixel → physical pixel
        physical_index = (self.pixel_index + self.offset) % self.count
        # color selection
        if self.pixel_index == 0:
            # pixel 0 (logical index) is the red one
            color = COLOR_RED
        else:
            color = COLOR_DEEP_CYAN
        if self.phase == 0:
            # turn pixel ON
            self.pixel.set_color(index=physical_index, color=color)
            self.phase = 1
        else:
            # turn pixel OFF
            self.pixel.set_color(index=physical_index, color=COLOR_BLACK)
            self.phase = 0
            # advance logical pixel index
            self.pixel_index += 1
            # completed a full rotation?
            if self.pixel_index >= self.count:
                self.pixel_index = 0
                # move the offset (if enabled)
                if self.auto_rotate:
                    self.offset = (self.offset + 1) % self.count

#EOF
