#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-14
# modified: 2025-10-14

import time
import colorsys
import ioexpander as io
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component

class CompassEncoder(Component):
    NAME = 'compass-encoder'
    '''
    A compass heading adjuster using a Pimoroni RGB Encoder Breakout.

    Features:
    - compass-aligned colors: North=Blue, East=Magenta, South=Yellow, West=Green
    - adjustable number of rotary ticks per full 360° sweep (wrap)
    - 8-point compass heading names
    - method get_degrees() to return current compass degrees

    :param config:       the application configuration.
    :param i2c_address:  the optional I2C address for the IO Expander
    :param level:        the log level.
    '''
    def __init__(self, config, i2c_address=0x0F, level=Level.INFO):
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['kros'].get('hardware').get('compass_encoder')
        # 0x18 for IO Expander, 0x0F for the RGB Encoder breakout
        if i2c_address is not None:
            self._i2c_address = i2c_address
        else:
            self._i2c_address = _cfg.get('i2c_address')
        self._log = Logger(CompassEncoder.NAME, level=level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._pin_red    = 1
        self._pin_green  = 7
        self._pin_blue   = 2
        self._brightness = _cfg.get('brightness', 1.0) # effectively max fraction of period LED will be on
        self._period     = int(255 / self._brightness)
        self._wrap       = _cfg.get('wrap', 24) # 24 is one turn per compass rotation, 360 is max/very slow
        self.ioe = io.IOE(i2c_addr=self._i2c_address, interrupt_pin=4)
        if self._i2c_address == 0x0F:
            self.ioe.enable_interrupt_out(pin_swap=True)
        self.ioe.setup_rotary_encoder(1, 12, 3)
        self.ioe.set_pwm_period(self._period)
        self.ioe.set_pwm_control(divider=2)
        self.ioe.set_mode(self._pin_red, io.PWM, invert=True)
        self.ioe.set_mode(self._pin_green, io.PWM, invert=True)
        self.ioe.set_mode(self._pin_blue, io.PWM, invert=True)
        self.count = 0
        self._mapping = {
            "N": Fore.BLUE,
            "NE": Fore.MAGENTA,
            "E": Fore.MAGENTA,
            "SE": Fore.YELLOW,
            "S": Fore.YELLOW,
            "SW": Fore.GREEN,
            "W": Fore.GREEN,
            "NW": Fore.BLUE
        }
        self._dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        self._log.info('ready.')

    def compass_to_rgb(self, count):
        degrees = (count % self._wrap) * (360.0 / self._wrap)
        hue = ((degrees + 240) % 360) / 360.0
        r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
        return int(r * self._period * self._brightness), int(g * self._period * self._brightness), int(b * self._period * self._brightness), degrees

    def heading_name(self, degrees):
        idx = int((degrees % 360) / 45) % 8
        return self._dirs[idx]

    def get_colorama_fore(self, degrees):
        '''
        Return the colorama Fore value for the main compass directions (N, E, S, W).
        '''
        return self._mapping.get(self.heading_name(degrees), Fore.WHITE)

    def set_color(self, r, g, b):
        self.ioe.output(self._pin_red, r)
        self.ioe.output(self._pin_green, g)
        self.ioe.output(self._pin_blue, b)

    def update(self):
        if self.ioe.get_interrupt():
            self.count = self.ioe.read_rotary_encoder(1)
            self.ioe.clear_interrupt()
        r, g, b, degrees = self.compass_to_rgb(self.count)
        self.set_color(r, g, b)
        self._log.info(self.get_colorama_fore(degrees)
                + ("{} {} → RGB({}, {}, {})".format(int(degrees), self.heading_name(degrees), r, g, b)))

    def get_degrees(self):
        _, _, _, degrees = self.compass_to_rgb(self.count)
        return degrees

    def close(self):
        self.set_color(0, 0, 0)

# usage ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

#   compass = CompassEncoder()
#
#   try:
#       while True:
#           compass.update()
#           time.sleep(1.0 / 30)
#   except KeyboardInterrupt:
#       pass
#   finally:
#       if compass:
#           compass.close()

#EOF
