#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-22
# modified: 2024-10-31
#

import sys, traceback, time
import itertools
import RPi.GPIO as GPIO
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Button(object):
    '''
    A simple button that can be configured to either use a GPIO pin or a pin
    on the IOExpander as input.

    Uses BCM mode, i.e., GPIO numbering, not pin numbers. When using the
    IOExpander the pin is on the IOExpander.

    This introduces a slight delay on returning a value in order
    to debounce the switch.

    :param config:        the application configuration
    :param pin:           the optional pin number (overrides config)
    :param impl:          the chosen implementation for GPIO handling
    :param waitable:      if True support wait()
    :param level:         the log level
    '''
    def __init__(self, config, pin=None, impl=None, waitable=False, level=Level.INFO):
        _cfg = config['krzos'].get('hardware').get('button')
        self._pin = _cfg.get('pin') if pin is None else pin
        self._log = Logger('button:{}'.format(self._pin), level)
        self._impl = impl if impl is not None else _cfg.get('impl') # either 'gpio' or 'ioe' or 'gpiozero'
        self._ioe = None
        if waitable:

            import RPi.GPIO as GPIO

            # set up flashing LED
            self._led_pin = _cfg.get('led_pin')
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._led_pin, GPIO.OUT)
            self._counter = itertools.count()
            self._toggle = itertools.cycle([True, False])
        if self._impl == 'gpio':

            import RPi.GPIO as GPIO

            GPIO.setwarnings(False)
            GPIO.cleanup()
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#           GPIO.setup(self._pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
            self._log.info('ready: pushbutton on GPIO pin {:d}'.format(self._pin))

        elif self._impl == 'ioe':

            import ioexpander as io

            _i2c_address = _cfg.get('i2c_address')
            self._ioe = io.IOE(i2c_addr=_i2c_address)
            self._ioe.set_mode(self._pin, io.IN_PU)
            self._log.info('ready: pushbutton on IO Expander pin {:d}'.format(self._pin))

        elif self._impl == 'gpiozero':

            from gpiozero import Button

            self._button = Button(self._pin, pull_up=True)  # create a Button object for the specified pin
            self._callbacks = []             # list to store callback functions
            self._button.when_pressed = self._button_pressed  # set the internal callback
            self._log.info('ready: pushbutton on GPIO pin {:d} using gpiozero.'.format(self._pin))

        else:
            raise Exception('unrecognised source: {}'.format(self._impl))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def add_callback(self, callback_method, bouncetime_ms=300):
        '''
        Set up a callback on the button's pin. This only works with the
        GPIO or gpiozero implementations. The 'bouncetime_ms ' argument
        is only used in the former.
        '''
        if not callable(callback_method):
            raise TypeError('provided callback was not callable.')
        if self._impl == 'gpio':
            try:
                # set up the event detection on pin
                GPIO.add_event_detect(self._pin, GPIO.FALLING, callback=callback_method, bouncetime=bouncetime_ms)
                self._log.info('added callback on GPIO pin {:d}'.format(self._pin))
            except Exception as e:
                self._log.error('{} error adding callback: {}\n{}'.format(type(e), e, traceback.format_exc()))
        elif self._impl == 'gpiozero':
              self._callbacks.append(callback_method)
        else:
            raise Exception('{} implementation does not support callbacks.'.format(self._impl))

    def _button_pressed(self):
        '''
        Internal method called when the button is pressed.
        '''
        self._log.info("button pressed!")
        for callback in self._callbacks:  # execute registered callbacks
            callback()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def pin(self):
        '''
        Returns the GPIO pin used by the pushbutton, using BCM numbering.
        '''
        return self._pin

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def pushed(self):
        '''
        Returns True if the button is pushed (low).
        '''
        if self._impl == 'gpio':
            _value = not GPIO.input(self._pin)
            time.sleep(0.1)
            return _value
        elif self._impl == 'ioe':
            return self._ioe.input(self._pin) == 0
        elif self._impl == 'gpiozero':
            return self._button.is_pressed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def wait(self):
        self._log.info(Fore.GREEN + 'waiting for button push…')
        while not self.pushed():
            if next(self._counter) % 5 == 0:
                GPIO.output(self._led_pin, GPIO.HIGH if next(self._toggle) else GPIO.LOW )
            time.sleep(0.1)
        GPIO.output(self._led_pin, GPIO.LOW)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        GPIO.cleanup(self._pin)

#EOF
