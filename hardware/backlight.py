#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-09-24
# modified: 2025-09-24
#
# Sets the backlight of the Display HAT Mini on or off.

import RPi.GPIO as GPIO
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Backlight(Component):
    '''
    Controls the backlight of the Display HAT Mini.
    '''
    def __init__(self, level=Level.INFO):
        self._log = Logger('backlight', level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._pin = 13 # TODO from config
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin, GPIO.OUT)
        self._log.info('ready.')

    def on(self):
        GPIO.output(self._pin, GPIO.HIGH)
        self._log.debug("backlight on")

    def off(self):
        GPIO.output(self._pin, GPIO.LOW)
        self._log.debug("backlight off")

    def close(self):
        GPIO.cleanup( self._pin)
        self._log.debug("closed.")

#EOF
