#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Portions copyright 2020-2025 by Murray Altheim. All rights reserved. This file
# is part of the Robot Operating System project, released under the MIT License.
# Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
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
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin, GPIO.OUT)
        self._log.info('ready.')

    def on(self):
        GPIO.output(self._pin, GPIO.HIGH)
        print(Fore.CYAN + "backlight on" + Style.RESET_ALL)

    def off(self):
        GPIO.output(self._pin, GPIO.LOW)
        print(Fore.CYAN + "backlight off" + Style.RESET_ALL)

    def close(self):
        GPIO.cleanup( self._pin)

#EOF
