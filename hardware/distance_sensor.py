#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-16
# modified: 2025-10-18

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
            case Orientation.PORT: # port side
                self._pin = _cfg.get('pin-port') # pin connected to the port side sensor
            case Orientation.STBD: # starboard side
                self._pin = _cfg.get('pin-stbd') # pin connected to the starboard side sensor
            case _:
                raise ValueError('unsupported orientation: {}'.format(orientation.label))
        self._orientation     = orientation
        self._name = 'distance={}'.format(orientation.label)
        self._log = Logger(self._name, level=Level.INFO)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._timeout         = _cfg.get('timeout')     # time in seconds to consider sensor as timed out
        self._min_valid_pulse_us = _cfg.get('min_valid_pulse_us', 1000) # anything <1000us is invalid
        self._max_valid_pulse_us = _cfg.get('max_valid_pulse_us', 1850) # anything >1850us is invalid
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
                self._log.info('external clock available.')
                self._external_clock.add_callback(self._external_callback_method)
            else:
                self._log.warning('no external clock available.')
        self._distance        = -1
        self._last_read_time  = time.time()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin, GPIO.IN)
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
        Returns None on timeout or if the measured pulse is too short (glitch).
        '''
        idle = GPIO.input(self._pin)
        # wait for start edge
        deadline = time.perf_counter() + 0.05
        while GPIO.input(self._pin) == idle:
            if time.perf_counter() > deadline:
                return None
        start = time.perf_counter()
        # wait for end edge
        deadline = start + 0.05
        while GPIO.input(self._pin) != idle:
            if time.perf_counter() > deadline:
                return None
        end = time.perf_counter()
        pulse_width_us = (end - start) * 1e6
        if pulse_width_us < self._min_valid_pulse_us:
            self._log.debug("measured pulse too short: {:.1f} us".format(pulse_width_us))
            return None
        self._last_read_time = time.time()
        self._log.debug("measured pulse width: {:.1f} us".format(pulse_width_us))
        return pulse_width_us

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _compute_distance(self):
        '''
        Compute and update the distance based on the current pulse width,
        returning the distance or None if out of range or if we do not have
        enough recent consistent readings. Requires 3 consecutive readings
        within a tolerance of their median to report a distance.
        '''
        pulse_us = self._measure_pulse_width()
        if pulse_us is None:
            self._log.debug("pulse width reading None.")
            return None
        if not (self._min_valid_pulse_us <= pulse_us <= self._max_valid_pulse_us):
#           self._log.debug("pulse width {} out of expected sensor range.".format(pulse_us))
            return None
        distance_mm = (pulse_us - self._min_valid_pulse_us) * 3 / 4
        self._last_read_time = time.time()
        if self._smoothing:
            required_consistent_count = 3     # consecutive recent readings required to agree before reporting
            tolerance_fraction        = 0.25  # allowed fractional deviation from the median (0.25 == ±25%)
            # append the new reading to the existing window
            self._window.append(distance_mm)
            if len(self._window) < required_consistent_count:
                return None
            recent_readings = list(self._window)[-required_consistent_count:]
            sorted_recent = sorted(recent_readings)
            median_index = required_consistent_count // 2
            median_of_recent = sorted_recent[median_index] if required_consistent_count % 2 else \
                (sorted_recent[median_index - 1] + sorted_recent[median_index]) / 2.0
            # all recent readings must be within tolerance_fraction of the median
            if all(abs(v - median_of_recent) <= max(1.0, median_of_recent * tolerance_fraction) for v in recent_readings):
                return int(median_of_recent)
            return None
        return int(distance_mm)

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
