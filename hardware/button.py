#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-22
# modified: 2025-11-23

import traceback
import time
import warnings
import sys
from gpiozero import Button as GpioZeroButton
from gpiozero.exc import CallbackSetToNone
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component

class Button(Component):
    '''
    A button configured to use a GPIO pin and the gpiozero library. This is
    somewhat more complicated than desired to handle the vagaries of gpiozero.

    This introduces a slight delay on returning a value in order to debounce
    the switch.

    See: https://gpiozero.readthedocs.io/en/latest/

    :param config:        the application configuration
    :param level:         the log level
    '''
    def __init__(self, config, name=None, level=Level.INFO):
        _cfg = config['kros'].get('hardware').get('button')
        self._pin = _cfg.get('pin')
        _log_name = 'btn-{}'.format(self._pin) if name is None else name
        self._log = Logger(_log_name, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._callbacks = []  # list of callback functions
        self._button = None
        self._closed = False
        try:
            # create Button with pull_up=True (assumes button connects pin to ground)
            # bounce_time adds debouncing (default 0.01s may be too short)
            self._button = GpioZeroButton(self._pin, pull_up=True, bounce_time=0.05)
            self._button.when_released = self._released
#           self._button.when_pressed  = lambda: print("released.")
            # patch __del__ to prevent the "GPIO busy" exception during shutdown
            _original_del = self._button.__class__.__del__
            def _silent_del(self):
                try:
                    _original_del(self)
                except Exception:
                    print('{} raised in _silent_del: {}'.format(type(e), e))
                    pass
            self._button.__class__.__del__ = _silent_del
            self._log.info('ready: pushbutton on GPIO pin {:d} using gpiozero.'.format(self._pin))
        except Exception as e:
            self._log.error('failed to initialize button on pin {}: {}'.format(self._pin, e))
            raise
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def clear_callbacks(self):
        self._callbacks.clear()

    def add_callback(self, callback_method):
        '''
        Add a callback to be executed when the button is pressed.
        '''
        if not callable(callback_method):
            raise TypeError('provided callback was not callable.')
        self._callbacks.append(callback_method)

    def execute_callbacks(self):
        '''
        Execute any callbacks attached to this Button.
        '''
        for callback in self._callbacks:
            try:
                callback()
            except Exception as e:
                self._log.error('error in button callback: {}'.format(e))

    def _released(self):
        '''
        Internal method called when the button is released.
        '''
        if self._closed:
            return # ignore callbacks during/after shutdown
        self._log.info(Fore.MAGENTA + "button pressed!")
        self.execute_callbacks()

    def _close_gpiozero(self):
        '''
        Properly clean up gpiozero resources.
        '''
        if self._button is None:
            return
        try:
            self._log.info("performing gpiozero cleanup…")
            # clear callbacks first to prevent new events
            with warnings.catch_warnings():
                warnings.filterwarnings('ignore', category=CallbackSetToNone)
                self._button.when_pressed = None
                self._button.when_released = None
            # delay to let pending callbacks complete
            time.sleep(0.05)
            try:
                self._button.close()
                self._button = None
            except Exception as e:
                self._log.info("{} raised closing gpiozero button: {}".format(type(e), e))
            self._log.info("gpiozero cleanup complete.")
        except Exception as e:
            self._log.error("{} raised during gpiozero cleanup: {}".format(type(e).__name__, e))
            if self._log.level == Level.DEBUG:
                self._log.debug(traceback.format_exc())

    @property
    def pin(self):
        '''
        Returns the GPIO pin used by the pushbutton, using BCM numbering.
        '''
        return self._pin

    def pushed(self):
        '''
        Returns True if the button is currently pushed (low).
        '''
        if self._button is None:
            return False
        return self._button.is_pressed

    def disable(self):
        '''
        Disable the button (stops processing events).
        '''
        if self.enabled:
            Component.disable(self)
            self._log.info('disabled.')

    def close(self):
        '''
        Permanently close the button and clean up GPIO resources.
        '''
        if not self._closed:
            self._log.info('closing button…')
            self._closed = True
            self.clear_callbacks()
            self._close_gpiozero()
            Component.close(self)  # calls disable
            self._log.info('closed.')

#EOF
