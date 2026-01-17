#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
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
    :param name:          the optional button name
    :param level:         the log level
    '''
    def __init__(self, config, name=None, level=Level.INFO):
        if name is None:
            name = 'button'
        _cfg = config['kros'].get('hardware').get('button')
        self._pin = _cfg.get('pin')
        _log_name = 'btn-{}'.format(self._pin) if name is None else name
        self._log = Logger(_log_name, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._callbacks = []  # list of callback functions
        self._button = None
        self._closed = False
        try:
            self._button = GpioZeroButton(self._pin, pull_up=True, bounce_time=0.05)
            self._button.when_released = self._released
            # patch __del__ to prevent the "GPIO busy" exception during shutdown
            _original_del = self._button.__class__.__del__
            log = self._log
            def _silent_del(self):
                nonlocal log
                try:
                    _original_del(self)
                except Exception as e:
                    log.debug('{} raised in _silent_del: {}'.format(type(e), e))
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
        if self.closed:
            return # ignore callbacks during/after shutdown
        self._log.info(Fore.MAGENTA + "button pressed!")
        self.execute_callbacks()

    def _close_gpiozero(self):
        '''
        Properly clean up gpiozero resources. A complete PITA.
        '''
        if self._button is None:
            return False
        self._log.info("performing gpiozero cleanup…")
        # clear callbacks first to prevent new events
        with warnings.catch_warnings():
            warnings.filterwarnings('ignore', category=CallbackSetToNone)
            if self._button.when_pressed is not None:
                self._button.when_pressed = None
            if self._button.when_released is not None:
                self._button.when_released = None
        # delay to let pending callbacks complete
        time.sleep(0.05)
        try:
            self._button.close()
            if self._button.closed:
                self._log.debug('closed.')
            time.sleep(0.2)
        except Exception as f:
            self._log.warning(Fore.CYAN + Style.DIM + "{} raised closing button: {}".format(type(f), f))
        finally:
            self._button = None
        return True

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
            super().disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

    def close(self):
        '''
        Permanently close the button and clean up GPIO resources.
        '''
        if not self.closed:
            try:
                super().close()
                self.clear_callbacks()
                if self._close_gpiozero():
                    Component.get_registry().deregister(self)
            except Exception as e:
                self._log.error('{} raised closing button: {}'.format(type(e), e))
                raise
            finally:
                self._log.info('closed.')
        else:
            self._log.warning('already closed.')

#EOF
