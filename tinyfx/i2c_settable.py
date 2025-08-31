#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-28
# modified: 2024-09-18
#
# A Tiny FX device whose value can be set via I2C.

from machine import Pin
from picofx import Updateable

class I2CSettableFX(Updateable):
    def __init__(self, pin=None, interval=0.1):
        self.interval = interval
        self.pin = Pin(pin, Pin.IN, Pin.PULL_UP)
        self.__value = 0.0
        self.__time = 0

    def __call__(self):
        return self.__value

    def tick(self, delta_ms):
        self.__time += delta_ms

        # Check if the interval has elapsed
        if self.__time >= (self.interval * 1000):
            self.__time -= (self.interval * 1000)
            if self.pin.value() == 1:
                self.__value = 1.0
            else:
                self.__value = 0.0
#           print('tick…; value: {}'.format(self.__value))

#EOF
