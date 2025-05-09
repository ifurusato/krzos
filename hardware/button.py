#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-22
# modified: 2025-04-27
#

import sys, traceback, time
import itertools
import threading
import RPi.GPIO as GPIO
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from hardware.player import Player
from hardware.sound import Sound

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Button(Component):
    '''
    A simple button that can be configured to either use a GPIO pin or a pin
    on the IOExpander as input.

    Uses BCM mode, i.e., GPIO numbering, not pin numbers. When using the
    IOExpander the pin is on the IOExpander.

    This introduces a slight delay on returning a value in order
    to debounce the switch.

    This supports RPi.GPIO, gpiod, gpiozero, or IO Expander implementations:

      * https://pypi.org/project/RPi.GPIO/
            or: https://pypi.org/project/rpi-lgpio/
      * https://libgpiod.readthedocs.io/en/latest/
      * https://gpiozero.readthedocs.io/en/latest/
      * https://github.com/pimoroni/ioe-python

    :param config:        the application configuration
    :param pin:           the optional pin number (overrides config)
    :param impl:          the chosen implementation for GPIO handling
    :param waitable:      if True support wait()
    :param level:         the log level
    '''
    def __init__(self, config, name='', pin=None, impl=None, waitable=False, momentary=False, level=Level.INFO):
        _cfg = config['krzos'].get('hardware').get('button')
        self._pin = _cfg.get('pin') if pin is None else pin
        self._log = Logger('btn:{}-{}'.format(self._pin, name), level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._impl = impl if impl is not None else _cfg.get('impl') # either 'gpio', 'gpiod', 'gpiozero' or 'ioe'
        self._ioe = None
        self._pi  = None
        self._stop_event = None
        self._momentary            = momentary
        self._pigpio_callback      = None
        self._gpiod_monitor_thread = None
        self._callbacks = [] # list of callback functions
        # configuration:
        if waitable:
            import RPi.GPIO as GPIO

            # set up flashing LED
            self._led_pin = _cfg.get('led_pin')
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._led_pin, GPIO.OUT)
            self._counter = itertools.count()
            self._toggle = itertools.cycle([True, False])
            self._log.info('ready: waitable pushbutton on GPIO pin {:d} using RPi.GPIO'.format(self._pin))

        if self._impl == 'gpio':

            import RPi.GPIO as GPIO

            GPIO.setwarnings(False)
            GPIO.cleanup()
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            _edge_detect = _cfg.get('edge_detect')
            if _edge_detect == 'falling':
                self._edge_detect = GPIO.FALLING # triggered when turned on
            elif _edge_detect == 'rising':
                self._edge_detect = GPIO.RISING # triggered when turned off
            elif _edge_detect == 'both':
                self._edge_detect = GPIO.BOTH # triggered when turned on or off
            else:
                raise ValueError("unrecognised edge detect: '{}'".format(_edge_detect))
            self._log.info("ready: pushbutton on GPIO pin {:d} using RPi.GPIO with {} edge detect.".format(self._pin, _edge_detect))

        elif self._impl == 'gpiod':

            if not self._momentary:
                # if latching, we need to check that it's not already triggered
                if not self.gpiod_get_value(pin=self._pin):
                    Player.play(Sound.GWOLP)
                    raise RuntimeError('cannot start, toggle switch is not enabled.')

            # monitoring is established by add_callback()

        elif self._impl == 'ioe':

            import ioexpander as io

            _i2c_address = _cfg.get('i2c_address')
            self._ioe = io.IOE(i2c_addr=_i2c_address)
            self._ioe.set_mode(self._pin, io.IN_PU)
            self._log.info('ready: pushbutton on IO Expander pin {:d}'.format(self._pin))

        elif self._impl == 'pigpio':

            import pigpio
            from hardware.pigpiod_util import PigpiodUtility

            if not PigpiodUtility.is_pigpiod_running():
                _pigpiod_util = PigpiodUtility()
                _pigpiod_util.ensure_running()

            self._pi = pigpio.pi()  # initialize pigpio
            if self._pi is None:
                raise Exception('unable to instantiate pigpio.pi().')
            elif self._pi._notify is None:
                raise Exception('can\'t connect to pigpio daemon; did you start it?')
            if not self._pi.connected:
                raise RuntimeError("Failed to connect to pigpio daemon")
            # set the GPIO pin mode
            self._pi.set_mode(self._pin, pigpio.INPUT)
            self._pi.set_pull_up_down(self._pin, pigpio.PUD_UP)
            self._log.info('ready: pushbutton on GPIO pin {:d} using pigpio.'.format(self._pin))

        elif self._impl == 'gpiozero':

            from gpiozero import Button

            self._button = Button(self._pin, pull_up=True)  # create a Button object for the specified pin
            self._button.when_pressed = self.__gpiozero_button_pressed  # set the internal callback
            self._log.info('ready: pushbutton on GPIO pin {:d} using gpiozero.'.format(self._pin))

        else:
            raise Exception('unrecognised source: {}'.format(self._impl))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def add_callback(self, callback_method, bouncetime_ms=300):
        '''
        Set up a callback on the button's pin. This only works with the GPIO,
        gpiod, or gpiozero implementations. The 'bouncetime_ms ' argument is
        only used with GPIO.
        '''
        if not callable(callback_method):
            raise TypeError('provided callback was not callable.')
        if self._impl == 'gpio':
            try:
                # set up the event detection on pin
                GPIO.add_event_detect(self._pin, self._edge_detect, callback=callback_method, bouncetime=bouncetime_ms)
                self._log.info('added callback on GPIO pin {:d}'.format(self._pin))
            except Exception as e:
                self._log.error('{} error adding callback: {}\n{}'.format(type(e), e, traceback.format_exc()))
        elif self._impl == 'gpiod':
            _debounce_time_ms = 300
            self._stop_event = threading.Event()
            _is_daemon = True
            self._gpiod_monitor_thread = threading.Thread(target=self._monitor_gpiod, args=(self._pin, _debounce_time_ms, callback_method, self._stop_event), daemon=_is_daemon)
            self._gpiod_monitor_thread.start()
        elif self._impl == 'pigpio':
            self._pigpio_callback = self._pi.callback(self._pin, pigpio.RISING_EDGE, self.__pigpio_callback)
            self._callbacks.append(callback_method)
        elif self._impl == 'gpiozero':
            self._callbacks.append(callback_method)
        else:
            raise Exception('{} implementation does not support callbacks.'.format(self._impl))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __pigpio_callback(self, gpio, level, tick):
        for callback in self._callbacks:  # execute registered callbacks
            callback()
#       self._callbacks.clear() # only if one-shot

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __gpiozero_button_pressed(self):
        '''
        Internal method called when the button is pressed.
        '''
        self._log.info(Fore.MAGENTA + "button pressed!")
        self._close_gpzio()
        for callback in self._callbacks:  # execute registered callbacks
            callback()
#       self._callbacks.clear() # only if one-shot

    def _close_gpzio(self):
        if self._button:
            try:
                self._log.info("performing gpiozero cleanup…")
                # cancel subsequent callbacks
                self._button.when_pressed = None
                self._button.close()
            except Exception as e:
                self._log.error("error during gpiozero cleanup: {e}".format(e))
            finally:
                self._button = None

    # gpiod ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def cancel(self):
        '''
        Sets the stop event, cancelling the gpiod callback thread.
        If not in gpiod mode this does nothing.
        '''
        if self._stop_event:
            self._stop_event.set()
            self._log.info('cancelled gpiod callback.')

    def gpiod_get_value(self, pin=None):
        '''
        Returns the value of the GPIO pin using gpiod, True is HIGH, False is LOW.
        '''
        import gpiod
        from gpiod.line import Bias, Edge, Value
        line_offset = pin
        _config = {
            line_offset: gpiod.LineSettings(
                direction=gpiod.line.Direction.INPUT,
                bias=Bias.PULL_UP, # set pull-up or pull-down
                edge_detection=Edge.FALLING
            )   
        }   
        _chip_path = "/dev/gpiochip0"
        with gpiod.request_lines(_chip_path, consumer="gpiod-callback-monitor", config=_config) as request:
            for value in request.get_values():
                if value is Value.ACTIVE:
                    return False
                elif value is Value.INACTIVE:
                    return True
                else:
                    raise Exception("unrecognised return value from gpiod '{}'".format(value))

    def _monitor_gpiod(self, line_offset, debounce_time_ms, callback, stop_event):
        '''
        Monitor GPIO for rising edges and trigger a callback.
        '''
        import gpiod
        from gpiod.line import Bias, Edge, Value

        _config = {
            line_offset: gpiod.LineSettings(
                direction=gpiod.line.Direction.INPUT,
                bias=Bias.PULL_UP, # set pull-up or pull-down
                edge_detection=Edge.RISING
            )
        }
        _chip_path = "/dev/gpiochip0"
        _last_event_time = 0
        self._log.info('monitoring pin {} using gpiod…'.format(line_offset))
        with gpiod.request_lines(_chip_path, consumer="gpiod-callback-monitor", config=_config) as request:
            while not stop_event.is_set():
                for event in request.read_edge_events():
                    event_time_ms = event.timestamp_ns // 1_000_000
                    if event_time_ms - _last_event_time >= debounce_time_ms:
                        for value in request.get_values():
                            if value is Value.ACTIVE:
                                self._log.info(Style.DIM + "valid rising edge ACTIVE on line {} at {}ms".format(event.line_offset, event_time_ms))
                                callback(event)
                            elif value is Value.INACTIVE:
                                if self._momentary:
                                    callback(event)
                                else:
                                    self._log.info(Style.DIM + "valid rising edge INACTIVE on line {} at {}ms".format(event.line_offset, event_time_ms))
                            else:
                                raise Exception("unrecognised return value from gpiod '{}'".format(value))
                        _last_event_time = event_time_ms
                time.sleep(0.5)
        self._log.debug('exited gpiod monitor thread.')

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
        if self._impl == 'gpio':
            GPIO.cleanup(self._pin)
        elif self._impl == 'gpiozero':
            self._close_gpzio()
        elif self._impl == 'gpiod':
            self._log.debug('cancelling gpiod callback…')
            if self._gpiod_monitor_thread:
                self.cancel()
        elif self._impl == 'pigpio':
            if self._pigpio_callback:
                self._log.debug('cancelling pigpio callback…')
                self._pigpio_callback.cancel()
            if self._pi:
                self._log.debug('stopping pigpio pi…')
                self._pi.stop()
        self._callbacks.clear()
        Component.close(self) # calls disable

#EOF
