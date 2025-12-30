#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-06-26
# modified: 2025-06-26
#
# Note that this changes the default 0x0F for 0x0B

import colorsys
import ioexpander as io
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level

class RotaryEncoder:
    I2C_ADDR  = 0x0F
    PIN_RED   = 1
    PIN_GREEN = 7
    PIN_BLUE  = 2
    POT_ENC_A = 12
    POT_ENC_B = 3
    POT_ENC_C = 11

    def __init__(self, i2c_addr=0x0F, multiplier=1, brightness=0.5):
        self._log = Logger('encoder', level=Level.INFO)
        self._i2caddress = i2c_addr
        self._count = 0
        self._use_stepped_hue = True
        self._multiplier = multiplier
        self._brightness = brightness
        self._period = int(255 / self._brightness)
        self.ioe = None
        self._log.info('ready.')

    def set_color(self, red, green, blue):
        self.ioe.output(self.PIN_RED, red)
        self.ioe.output(self.PIN_GREEN, green)
        self.ioe.output(self.PIN_BLUE, blue)

    def start(self):
        self.ioe = io.IOE(i2c_addr=self._i2caddress, interrupt_pin=4)
        # swap interrupt pin for rotary encoder breakout
        if self._i2caddress == 0x0B:
            self.ioe.enable_interrupt_out(pin_swap=True)
        self.ioe.setup_rotary_encoder(1, RotaryEncoder.POT_ENC_A, RotaryEncoder.POT_ENC_B, pin_c=RotaryEncoder.POT_ENC_C)
        self.ioe.set_pwm_period(self._period)
        self.ioe.set_pwm_control(divider=2)  # PWM as fast as possible
        self.ioe.set_mode(RotaryEncoder.PIN_RED, io.PWM, invert=True)
        self.ioe.set_mode(RotaryEncoder.PIN_GREEN, io.PWM, invert=True)
        self.ioe.set_mode(RotaryEncoder.PIN_BLUE, io.PWM, invert=True)
        self._log.info("started: RGB encoder with a period of {}, and {} brightness steps.".format(self._period, int(self._period * self._brightness)))

    def update(self):
        if self.ioe.get_interrupt():
            _count = self.ioe.read_rotary_encoder(1)
            self._count = _count * self._multiplier
            self.ioe.clear_interrupt()
        hue = (self._count % 360) / 360.0
        mode = round(hue * 10) / 10.0
        if self._use_stepped_hue:
            r, g, b = [int(c * self._period * self._brightness) for c in colorsys.hsv_to_rgb(mode, 1.0, 1.0)]
        else:
            r, g, b = [int(c * self._period * self._brightness) for c in colorsys.hsv_to_rgb(hue, 1.0, 1.0)]
        self.ioe.output(RotaryEncoder.PIN_RED, r)
        self.ioe.output(RotaryEncoder.PIN_GREEN, g)
        self.ioe.output(RotaryEncoder.PIN_BLUE, b)
        return mode, hue, r, g, b

    @staticmethod
    def bounded_rollover(value, limit):
        if value >= limit or value <= -limit:
            return 0
        return value

    def off(self):
        self._log.info('off.')
        self.ioe.output(RotaryEncoder.PIN_RED, 0)
        self.ioe.output(RotaryEncoder.PIN_GREEN, 0)
        self.ioe.output(RotaryEncoder.PIN_BLUE, 0)

#EOF
