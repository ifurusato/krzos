#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2024 by Murray Altheim. All rights reserved. This file is part of
# the Robot Operating System project and is released under the "Apache Licence,
# Version 2.0". Please see the LICENSE file included as part of this package.
#   
# author:   Murray Altheim
# created:  2025-05-23
# modified: 2025-07-14

import time
import pyb
from neopixel import NeoPixel

from colorama import Fore, Style
from logger import Logger, Level
from colors import *

class Pixel:
    def __init__(self, config=None, pixel_count=1, color_order='GRB', brightness=0.33):
        self._log = Logger('pixel', Level.INFO)
        if config is None:
            pin = 'E1'
        else:
            pin = config['kros']['pixel']['pin']
        self._pixel_count = pixel_count
        self._pixel_index = 0
        self._brightness = brightness
        self._neopixel = NeoPixel(pyb.Pin(pin, pyb.Pin.OUT), pixel_count, color_order=color_order, brightness=brightness)
        self.set_color(index=None, color=None)
        self._log.info(Fore.GREEN + 'neopixel ready on pin {}.'.format(pin))

    @property
    def pixel_count(self):
        return self._pixel_count

    @property
    def brightness(self):
        return self._brightness

    def set_color(self, index=None, color=None):
        '''
        Function to set a single pixel to a color as a tuple.
        '''
        _index = self._pixel_index if index is None else index
        if color is None:
            self._neopixel[_index] = (0, 0, 0)
        else:
#           hsv = Pixel.rgb_to_hsv(*color)
#           hue = hsv[0]
            self._neopixel[_index] = color
        self._neopixel.write()

    def off(self):
        '''
        Function to turn off all LEDs.
        '''
        for i in range(self._pixel_count):
            self._neopixel[i] = (0, 0, 0) # turn off each LED
        self._neopixel.write()       # apply the changes
#       time.sleep(0.05) # rest a bit

    def rainbow_cycle(self, delay=0.05, steps=-1):
        '''
        Cycle a single NeoPixel through the colors of the rainbow.

        This function steps through hues from 0.0 to 1.0, converts them to RGB,
        and writes the result to the specified pixel.

        :param np:             (NeoPixel) An instance of the NeoPixel class.
        :param delay:          (float) Time in seconds to wait between color updates. Default is 0.01.
        :param steps:          (int) Number of color steps to cycle through the rainbow. 
                                     Higher = smoother. Default is 255. If -1 runs indefinitely.
        '''
        step = 0
        while True:
            hue = (step % steps) / steps if steps != -1 else (step % 360) / 360
            color = Pixel.hsv_to_rgb(hue)
            self._neopixel[self.pixel_index] = color
            self._neopixel.write()
            time.sleep(delay)
            step += 1
            if steps != -1 and step >= steps:
                break

    @staticmethod
    def hsv_to_rgb(h, s=1.0, v=1.0):
        '''
        Convert HSV color to RGB.

        Returns:
            tuple: (R, G, B) tuple with 8-bit integer values in range [0, 255].

        :param h:     (float) Hue, in range [0.0, 1.0]. Represents color on the color wheel.
        :param s:     (float) Saturation, in range [0.0, 1.0]. 1.0 is full color, 0.0 is grayscale.
        :param v:     (float) Value (brightness), in range [0.0, 1.0].
        '''
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
        '''
        Convert RGB color to HSV.

        Returns:
            tuple: (H, S, V) where:
                H is hue in [0.0, 1.0]
                S is saturation in [0.0, 1.0]
                V is value (brightness) in [0.0, 1.0]

        :param r: (int) Red, in range [0, 255]
        :param g: (int) Green, in range [0, 255]
        :param b: (int) Blue, in range [0, 255]
        '''
        r_f = r / 255.0
        g_f = g / 255.0
        b_f = b / 255.0
        max_c = max(r_f, g_f, b_f)
        min_c = min(r_f, g_f, b_f)
        delta = max_c - min_c
        # Hue calculation
        if delta == 0:
            h = 0.0
        elif max_c == r_f:
            h = ((g_f - b_f) / delta) % 6
        elif max_c == g_f:
            h = ((b_f - r_f) / delta) + 2
        else:  # max_c == b_f
            h = ((r_f - g_f) / delta) + 4
        h /= 6.0
        # Saturation
        s = 0.0 if max_c == 0 else delta / max_c
        # Value
        v = max_c
        return (h, s, v)

#EOF
