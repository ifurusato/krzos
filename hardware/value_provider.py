#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-12
# modified: 2025-07-04

from upy.mode import Mode
from hardware.digital_pot_async import DigitalPotentiometer
from hardware.rotary_encoder import RotaryEncoder

class ValueProvider:
    def __call__(self):
        raise NotImplementedError("subclasses must implement __call__()")

    def off(self):
        raise NotImplementedError("subclasses must implement off()")

class DigitalPotSpeedProvider(ValueProvider):

    def __init__(self, multiplier=100.0):
        self._digital_pot = DigitalPotentiometer(multiplier=multiplier)
        self._digital_pot.start()

    def __call__(self):
        return self._digital_pot.data

    def close(self):
        self._digital_pot.off()

class GoCommandProvider(ValueProvider):

    def __init__(self):
        pass

    def __call__(self):
        return 'GO'

    def close(self):
        pass

class RotaryEncoderCommandProvider(ValueProvider):

    def __init__(self):
        self._encoder = RotaryEncoder(i2c_addr=0x0F, multiplier=20, brightness=1.0)
        self._encoder.start()

    def __call__(self):
        _mode, hue, r, g, b = self._encoder.update()
        index = min(int(hue * 16), 15)
        mode = Mode.from_index(index)
        if mode is None:
            raise ValueError("no mode for hue '{}'".format(hue))
        else:
#           return 'GO'
            return mode.code

    def close(self):
        self._encoder.off()

#EOF
