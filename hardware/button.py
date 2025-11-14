#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-22
# modified: 2025-11-09
#
# Simple implementation of Button using gpiozero.

import traceback
import time
import warnings
from gpiozero import Button as GpioZeroButton
from gpiozero.exc import CallbackSetToNone
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Button(Component):
    '''
    A simple button configured to use a GPIO pin and the gpiozero library.

    This introduces a slight delay on returning a value in order
    to debounce the switch.

      * https://gpiozero.readthedocs.io/en/latest/

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
            # Create Button with pull_up=True (assumes button connects pin to ground)
            # bounce_time adds debouncing (default 0.01s may be too short)
            self._button = GpioZeroButton(self._pin, pull_up=True, bounce_time=0.05)
            self._button.when_pressed = self._gpiozero_button_pressed
            self._log.info('ready: pushbutton on GPIO pin {:d} using gpiozero.'.format(self._pin))
        except Exception as e:
            self._log.error('failed to initialize button on pin {}: {}'.format(self._pin, e))
            raise

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def clear_callbacks(self):
        self._callbacks.clear()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def add_callback(self, callback_method):
        '''
        Add a callback to be executed when the button is pressed.
        '''
        if not callable(callback_method):
            raise TypeError('provided callback was not callable.')
        self._callbacks.append(callback_method)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _gpiozero_button_pressed(self):
        '''
        Internal method called when the button is pressed.
        '''
        if self._closed:
            return  # ignore callbacks during/after shutdown

        self._log.info(Fore.MAGENTA + "button pressed!")
        for callback in self._callbacks:
            try:
                callback()
            except Exception as e:
                self._log.error('error in button callback: {}'.format(e))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _close_gpiozero(self):
        '''
        Properly clean up gpiozero resources.
        '''
        if self._button is None:
            return
        try:
            self._log.info("performing gpiozero cleanup…")
            # cancel callback first to prevent new events
            # and suppress the CallbackSetToNone warning
            with warnings.catch_warnings():
                warnings.filterwarnings('ignore', category=CallbackSetToNone)
                self._button.when_pressed = None
                self._button.when_released = None
            # small delay to let any pending callbacks complete
            time.sleep(0.05)
            # close the button object
            self._button.close()
            self._log.info("gpiozero cleanup complete.")
        except Exception as e:
            if self._log.level == Level.DEBUG:
                # log but don't raise - we're shutting down anyway
                self._log.debug("{} during gpiozero cleanup: {}".format(type(e).__name__, e))
                self._log.debug(traceback.format_exc())
        finally:
            self._button = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def pin(self):
        '''
        Returns the GPIO pin used by the pushbutton, using BCM numbering.
        '''
        return self._pin

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def pushed(self):
        '''
        Returns True if the button is currently pushed (low).
        '''
        if self._button is None:
            return False
        return self._button.is_pressed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable the button (stops processing events).
        '''
        if self.enabled:
            Component.disable(self)
            self._log.info('disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
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
