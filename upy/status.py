#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-07-02
# modified: 2025-07-15

from pixel import Pixel
from colors import *

class Status:
    COLOR_OFF      = (0, 0, 0)
    COLOR_ERROR    = (120, 8, 0)
    COLOR_READY    = (30, 70, 0)
    COLOR_INACTIVE = (11, 11, 11)
    APP = 0
    M0  = 1
    M1  = 3
    M2  = 5
    M3  = 7

    def __init__(self, pixel):
        if pixel.pixel_count < 8:
            raise ValueError('at least eight pixels required.')
        self._motors = [ Status.M0, Status.M1, Status.M2, Status.M3 ]
        self._pixel = pixel
        self._brightness = pixel.brightness

    def off(self):
        self._pixel.set_color(index=0, color=Status.COLOR_OFF)
        self._pixel.off()

    def rgb(self, color=None):
        self._pixel.set_color(index=0, color=COLOR_CYAN if color == None else color)

    def ready(self):
        self._pixel.set_color(index=0, color=Status.COLOR_READY)

    def error(self):
        self._pixel.set_color(index=0, color=Status.COLOR_ERROR)

    def motors(self, values):
        if all(v == 0.0 for v in values):
            self.off()
        else:
            for index in range(0,4):
                speed = values[index]
                if speed == 0.0:
                    color = Status.COLOR_INACTIVE
                else:
                    color = self._get_color(speed)
                self.motor(index, color)

    def _get_color(self, speed):
        if speed == 0:
            return COLOR_BLACK
        hue = 0.0 if speed < 0 else 0.27 # red or yellow-green 
        value = abs(speed / 10.0) * self._brightness
        rgb = Pixel.hsv_to_rgb(hue, 1.0, value)
        return rgb

    def motor(self, index, color):
        if 0 <= index <= 3:
            pixel = self._motors[index]
            self._pixel.set_color(index=pixel, color=color)
        else:
            raise ValueError('expected 0-3 for index, not {}'.format(index))

#EOF
