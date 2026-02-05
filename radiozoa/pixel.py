#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-05-23
# modified: 2025-02-01

import time
from machine import Pin
from neopixel import NeoPixel
from colors import Color

class Pixel:
    def __init__(self, pin=None, pixel_count=1, color_order='GRB', brightness=0.33):
        if pin is None:
            raise ValueError('pin must be specified.')
        elif isinstance(pin, Pin):
            _pin = pin
        elif isinstance(pin, int) or isinstance(pin, str):
            _pin = Pin(pin, Pin.OUT)
        else:
            raise TypeError('expected Pin or int for pin specification, not {}.'.format(type(pin)))
#       print('configuring neopixel on pin {}.'.format(pin))
        self._pixel_count = pixel_count
        self._pixel_index = 0
        self._brightness = brightness
        self._neopixel = NeoPixel(_pin, pixel_count, color_order=color_order, brightness=brightness)
        self.set_color(index=None, color=None)
#       print('neopixel ready on pin {}.'.format(pin))

    @property
    def pixel_count(self):
        return self._pixel_count

    @property
    def brightness(self):
        return self._brightness

    def rainbow_cycle(self, delay=0.05, steps=-1):
        step = 0
        while True:
            hue = (step % steps) / steps if steps != -1 else (step % 360) / 360
            color = Pixel.hsv_to_rgb(hue)
            self._neopixel[self._pixel_index] = color
            self._neopixel.write()
            time.sleep(delay)
            step += 1
            if steps != -1 and step >= steps:
                break

    def set_color(self, index=None, color=None):
        _index = self._pixel_index if index is None else index
        if isinstance(color, Color):
            self._neopixel[_index] = color.rgb
        elif color is None:
            self._neopixel[_index] = (0, 0, 0)
        else:
            self._neopixel[_index] = color
        self._neopixel.write()

    def off(self):
        for i in range(self._pixel_count):
            self._neopixel[i] = (0, 0, 0)
        self._neopixel.write()

    @staticmethod
    def hsv_to_rgb(h, s=1.0, v=1.0):
        i = int(h * 6)
        f = (h * 6) - i
        p = int(v * (1 - s) * 255)
        q = int(v * (1 - f * s) * 255)
        t = int(v * (1 - (1 - f) * s) * 255)
        v = int(v * 255)
        i %= 6
        if i == 0:
            return (v, t, p)
        elif i == 1:
            return (q, v, p)
        elif i == 2:
            return (p, v, t)
        elif i == 3:
            return (p, q, v)
        elif i == 4:
            return (t, p, v)
        else:
            return (v, p, q)

    @staticmethod
    def rgb_to_hsv(r, g, b):
        r_f = r / 255.0
        g_f = g / 255.0
        b_f = b / 255.0
        max_c = max(r_f, g_f, b_f)
        min_c = min(r_f, g_f, b_f)
        delta = max_c - min_c
        if delta == 0:
            h = 0.0
        elif max_c == r_f:
            h = ((g_f - b_f) / delta) % 6
        elif max_c == g_f:
            h = ((b_f - r_f) / delta) + 2
        else:  # max_c == b_f
            h = ((r_f - g_f) / delta) + 4
        h /= 6.0
        s = 0.0 if max_c == 0 else delta / max_c
        v = max_c
        return (h, s, v)

#EOF
