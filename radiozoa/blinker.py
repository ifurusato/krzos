#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-01-27
# modified: 2026-02-01

from machine import Timer

import tinys3

from pixel import Pixel
from colors import*

class Blinker:
    def __init__(self, pixel=None, on_ms=100, off_ms=100):
        if pixel:
            self._pixel = pixel
        else:
            self._pixel = Pixel(pin=tinys3.RGB_DATA, pixel_count=1)
        self.on_ticks  = on_ms
        self.off_ticks = off_ms
        self.is_on     = True
        self.counter   = 0
        self._timer = Timer(3)
        self._led_on()
        self._timer.init(freq=1000, callback=self._toggle)
        print('blinker ready.')

    def set_duty_cycle(self, on_ms, off_ms):
        self.on_ticks  = on_ms
        self.off_ticks = off_ms

    def _led_on(self):
        self._pixel.set_color(0, COLOR_RED)

    def _led_off(self):
        self._pixel.set_color(0, COLOR_BLACK)
    
    def _toggle(self, timer):
        self.counter += 1
        if self.is_on and self.counter >= self.on_ticks:
            self._led_off()
            self.is_on = False
            self.counter = 0
        elif not self.is_on and self.counter >= self.off_ticks:
            self._led_on()
            self.is_on = True
            self.counter = 0

#EOF
