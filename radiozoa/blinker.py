#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-01-27
# modified: 2026-01-28

from pyb import LED, Timer

class Blinker:
    def __init__(self, on_ms, off_ms):
        self.led = LED(1)
        self.on_ticks = on_ms
        self.off_ticks = off_ms
        self.is_on = True
        self.counter = 0
        self.timer = Timer(4)
        self.led.on()
        self.timer.init(freq=1000, callback=self.toggle)
    
    def set_duty_cycle(self, on_ms, off_ms):
        self.on_ticks = on_ms
        self.off_ticks = off_ms

    def toggle(self, timer):
        self.counter += 1
        if self.is_on and self.counter >= self.on_ticks:
            self.led.off()
            self.is_on = False
            self.counter = 0
        elif not self.is_on and self.counter >= self.off_ticks:
            self.led.on()
            self.is_on = True
            self.counter = 0

#EOF
