#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-16
# modified: 2025-10-16

import time
import warnings
from collections import deque
import RPi.GPIO as GPIO
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.orientation import Orientation
from core.util import Util

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DistanceSensor(Component):
    '''
    Provides distance information in millimeters from a Pololu PWM-based
    infrared proximity sensor. The maximum range seems to be around 270mm,
    reliably to about 250mm, and the returned value is quite accurate.
    '''
    def __init__(self, config, orientation=None, level=Level.INFO):
        '''
        Initializes the DistanceSensor.

        :param config:        the application configuration
        :param orientation:   the orientation of the sensor (PSID, SSID or FWD)
        :param level:         the logging Level
        '''
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['kros'].get('hardware').get('distance_sensor')
        match orientation:
            case Orientation.FWD: # forward
                self._pin = _cfg.get('pin-fwd') # pin connected to the forward sensor
            case Orientation.PSID: # port side
                self._pin = _cfg.get('pin-psid') # pin connected to the port side sensor
            case Orientation.SSID: # starboard side
                self._pin = _cfg.get('pin-ssid') # pin connected to the starboard side sensor
            case _:
                raise ValueError('unsupported orientation: {}'.format(orientation.label))
        self._orientation     = orientation
        self._name = 'distance={}'.format(orientation.label)
        self._log = Logger(self._name, level=Level.INFO)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._timeout         = _cfg.get('timeout')     # time in seconds to consider sensor as timed out
        self._smoothing       = _cfg.get('smoothing')   # enable smoothing of distance readings
        _smoothing_window     = _cfg.get('smoothing_window')
        self._window = deque(maxlen=_smoothing_window) if self._smoothing else None
        self._loop_interval   = _cfg.get('loop_interval') # interval between distance polling, in seconds
        _use_ext_clock   = _cfg.get('use_ext_clock') # poll on callback from external clock if available
        self._external_clock  = None
        if _use_ext_clock:
            _component_registry = Component.get_registry()
            self._external_clock = _component_registry.get('irq-clock')
            if self._external_clock:
                self._log.warning(Fore.WHITE + 'external clock available.')
                self._external_clock.add_callback(self._external_callback_method)
            else:
                self._log.warning('no external clock available.')
        self._distance        = -1
        self._last_read_time  = time.time()
        GPIO.setmode(GPIO.BCM)
        self._log.info('{} distance sensor ready on pin {}.'.format(self._orientation.label, self._pin))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def orientation(self):
        return self._orientation

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _measure_pulse_width(self):
        '''
        Polls the GPIO pin to measure pulse width in microseconds.
        Returns None if no valid pulse found within timeout.
        '''
        timeout = time.perf_counter() + 0.05
        # wait for rising edge (start of pulse)
        while GPIO.input(self._pin) == 0:
            if time.perf_counter() > timeout:
                return None
        start = time.perf_counter()
        # wait for falling edge (end of pulse)
        timeout = time.perf_counter() + 0.05
        while GPIO.input(self._pin) == 1:
            if time.perf_counter() > timeout:
                return None
        end = time.perf_counter()
        pulse_width_us = (end - start) * 1e6  # microseconds
        self._last_read_time = time.time()
        self._log.debug("measured pulse width: {:.1f} us".format(pulse_width_us))
        return pulse_width_us

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _compute_distance(self):
        '''
        Compute and update the distance based on the current pulse width,
        returning the distance or None if out of range.
        '''
        pulse_width_us = self._measure_pulse_width()
        distance = None
        if pulse_width_us is not None and 1000 <= pulse_width_us <= 1850:
            distance_mm = (pulse_width_us - 1000) * 3 / 4
            self._last_read_time = time.time()
            if self._smoothing:
#               self._log.debug("\nappend distance: {:4.2f}mm".format(distance_mm))
                self._window.append(distance_mm)
                distance = int(sum(self._window) / len(self._window))
            else:
                distance = int(distance_mm)
        else:
            self._log.debug("pulse width {} out of expected sensor range.".format(pulse_width_us))
        return distance

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _external_callback_method(self):
        ''' 
        The callback called by the external clock as an alternative to the
        asyncio _loop() method.
        '''
        _distance_mm = self._compute_distance()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_distance(self):
        '''
        Returns the current distance value in millimeters, None if
        out of range.
        '''
        return self._compute_distance()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def check_timeout(self):
        '''
        Check if the sensor has timed out (no pulse received recently).
        '''
        return time.time() - self._last_read_time > self._timeout

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enable the sensor, setting up the GPIO pin.
        '''
        if not self.enabled:
            Component.enable(self)
            GPIO.setup(self._pin, GPIO.IN)
        else:
            self._log.warning('already enabled distance sensor.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable the sensor and clean up resources.
        '''
        # we know of this warning so make it pretty
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always", RuntimeWarning)
            GPIO.cleanup(self._pin)
            for warning in w:
                msg = Util.ellipsis('{}'.format(warning.message), 33)
                self._log.info(Style.DIM + 'warning on GPIO cleanup: {}'.format(msg))
        Component.disable(self)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Stop the loop if running, then close the sensor.
        '''
        self.disable()
        Component.close(self)

#EOF
