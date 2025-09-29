#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-01-18
# modified: 2025-09-22
#
# Refactored to replace pigpio with gpiozero. gpiozero does not require a daemon;
# it uses RPi.GPIO or similar under the hood.
#
# For installing gpiozero, see:  https://gpiozero.readthedocs.io/en/stable/installing.html
#

import sys, traceback
from colorama import init, Fore, Style
init()

try:
    from gpiozero import RotaryEncoder
except ImportError as ie:
    print("This script requires the gpiozero module.\nInstall with: pip3 install --user gpiozero")
    raise ModuleNotFoundError('gpiozero not installed.')
from colorama import init, Fore, Style
init()

from core.logger import Logger

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Decoder(object):
    '''
    Class to decode mechanical rotary encoder pulses, implemented
    using gpiozero's RotaryEncoder class.

    Decodes the rotary encoder A and B pulses, e.g.:

                     +---------+         +---------+      0
                     |         |         |         |
           A         |         |         |         |
                     |         |         |         |
           +---------+         +---------+         +----- 1

               +---------+         +---------+            0
               |         |         |         |
           B   |         |         |         |
               |         |         |         |
           ----+         +---------+         +---------+  1

    '''

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __init__(self, orientation, gpio_a, gpio_b, callback, level):
        '''
        Instantiate the class with the gpios connected to
        rotary encoder contacts A and B. The common contact should
        be connected to ground. The callback is called when the
        rotary encoder is turned. It takes one parameter which is
        +1 for clockwise and -1 for counterclockwise.

        EXAMPLE

        from lib.decoder import Decoder

        def my_callback(self, step):
           self._steps += step

        pin_a = 17
        pin_b = 18
        decoder = Decoder(pi, pin_a, pin_b, my_callback)
        ...
        decoder.cancel()

        :param orientation:  the motor orientation
        :param gpio_a:        pin number for A
        :param gpio_b:        pin number for B
        :param callback:     the callback method
        :param level:        the log Level
        '''
        self._log = Logger('enc:{}'.format(orientation.label), level)
        self._gpio_a    = gpio_a
        self._gpio_b    = gpio_b
        self._log.debug('pin A: {:d}; pin B: {:d}'.format(self._gpio_a,self._gpio_b))
        self._callback  = callback
        self._reversed = False

        try:
            # gpiozero RotaryEncoder takes pin_a, pin_b as args.
            self._encoder = RotaryEncoder(self._gpio_a, self._gpio_b, max_steps=0)
            self._encoder.when_rotated = self._rotated
            self._last_value = self._encoder.value
            self._log.info('configured {} motor encoder with channel A on pin {}, channel B on pin {}.'.format(orientation.name, self._gpio_a, self._gpio_b))
        except Exception as e:
            self._log.error('error importing and/or configuring Motor: {}'.format(e))
            traceback.print_exc(file=sys.stdout)
            raise Exception('unable to configure decoder.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_reversed(self):
        '''
        If called, sets this encoder for reversed operation.
        '''
        self._reversed = True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _rotated(self):
        '''
        Internal handler called by gpiozero when the encoder is rotated.
        Calls user callback with +1 or -1 depending on direction.
        '''
        step = self._encoder.steps - self._last_value
        forward = step >= 0
        if self._reversed:
            forward = not forward
        if forward: # forward
            self._log.info(Fore.BLUE + Style.BRIGHT + 'FORWARD on pin A: {:d}; pin B: {:d}; {} steps.'.format(self._gpio_a,self._gpio_b, self._encoder.steps))
            self._callback(1)
        else: # reverse
            self._log.info(Fore.RED + Style.BRIGHT + 'REVERSE on pin A: {:d}; pin B: {:d}; {} steps.'.format(self._gpio_a,self._gpio_b, self._encoder.steps))
            self._callback(-1)
        self._last_value = self._encoder.steps

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def cancel(self):
        '''
        Cancel the rotary encoder decoder.
        '''
        # gpiozero RotaryEncoder does not have a cancel method,
        # but we can close it to clean up resources.
        self._encoder.close()

#EOF
