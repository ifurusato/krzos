#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-22
# modified: 2025-10-18
#
# This is the simple implementation of Button, implemented using gpiozero.

import traceback
import RPi.GPIO as GPIO
import gpiozero
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Button(Component):
    '''
    A simple button configured to use a GPIO pin and the gpiozero library.

    This introduces a slight delay on returning a value in order
    to debounce the switch.

      * https://gpiozero.readthedocs.io/en/latest/

    :param config:        the application configuration
    :param name:          the optional button name (used for logging)
    :param pin:           the optional pin number (overrides config)
    :param level:         the log level
    '''
    def __init__(self, config, level=Level.INFO):
        _cfg = config['kros'].get('hardware').get('button')
        self._pin = _cfg.get('pin')
        self._log = Logger('btn-{}'.format(self._pin), level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._callbacks = [] # list of callback functions
        self._button = gpiozero.Button(self._pin, pull_up=True)  # create a Button object for the specified pin
        self._button.when_pressed = self._gpiozero_button_pressed  # set the internal callback
        self._log.info('ready: pushbutton on GPIO pin {:d} using gpiozero.'.format(self._pin))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def add_callback(self, callback_method):
        '''
        Set up a callback on the button's pin. 
        '''
        if not callable(callback_method):
            raise TypeError('provided callback was not callable.')
        self._callbacks.append(callback_method)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _gpiozero_button_pressed(self):
        '''
        Internal method called when the button is pressed.
        '''
        self._log.info(Fore.MAGENTA + "button pressed!")
        for callback in self._callbacks:  # execute registered callbacks
            callback()

    def _close_gpzio(self):
        if self._button:
            try:
                self._log.info("performing gpiozero cleanup…")
                # cancel subsequent callbacks
                self._button.when_pressed = None
                self._button.close()
            except Exception as e:
                self._log.error("{} raised gpiozero cleanup: {}\n{}".format(type(e), e, traceback.format_exc()))
            finally:
                self._button = None

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
        Returns True if the button is pushed (low). This is not supported
        on all implementations.
        '''
        return self._button.is_pressed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        self._close_gpzio()
        self._callbacks.clear()
        Component.close(self) # calls disable

#EOF
