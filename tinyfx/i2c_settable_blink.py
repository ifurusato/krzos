#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2024-09-07
# modified: 2024-09-07
#
# Uses two pins to determine if one of three LED channels is to be turned on.
#

from machine import Pin
from picofx import Cycling

class I2CSettableBlinkFX(Cycling):
    def __init__(self, channel=0, pin0=16, pin1=17, interval=0.1, speed=1, phase=0.0, duty=0.5, additive=True):
        self._channel = channel
        self._pin0 = Pin(pin0, Pin.IN, Pin.PULL_UP)
        self._pin1 = Pin(pin1, Pin.IN, Pin.PULL_UP)
        self.interval = interval
        super().__init__(speed)
        self.phase = phase
        self.duty  = duty
        self._additive = additive
        self._enabled = False
        self.__time = 0
        self._ignore_pins = True

    def on(self):
        self._enabled = True

    def off(self):
        self._enabled = False

    def tick(self, delta_ms):
        super().tick(delta_ms)
        if self._ignore_pins:
            return
        self.__time += delta_ms
        # check if the interval has elapsed
        if self.__time >= (self.interval * 1000):
            self.__time -= (self.interval * 1000)
            _v0 = int(self._pin0.value())
            _v1 = int(self._pin1.value())
            # we reverse the logic so low is True
            _v0 = 0 if _v0 == 1 else 1
            _v1 = 0 if _v1 == 1 else 1
            _decimal = int('{}{}'.format(_v1, _v0), 2)
            if _decimal == 0:
                self._enabled = False
            elif _decimal == self._channel:
                self._enabled = True
            elif not self._additive:
                self._enabled = False
            else:
                pass # no change

    def __call__(self):
        if self._enabled:
            percent = (self.__offset + self.phase) % 1.0
            return 1.0 if percent < self.duty else 0.0
        else:
            return 0.0

#EOF
