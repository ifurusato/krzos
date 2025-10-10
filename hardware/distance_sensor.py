#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-16
# modified: 2025-10-07

import time
import warnings
from threading import Thread
from collections import deque
import RPi.GPIO as GPIO
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.util import Util

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DistanceSensor(Component):
    NAME = 'distance'
    '''
    Provides distance information in millimeters from a Pololu PWM-based
    infrared proximity sensor. The maximum range seems to be around 270mm,
    reliably to about 250mm, and the returned value is quite accurate.

    Previous versions of this sensor included Orientation as it supported
    multiple sensors; this supports only a single sensor. It also supported
    use with the MessageBus; this has been eliminated to simplify the class.

    This can run either in a Thread, or by simply calling get_distance()
    directly.
    '''
    def __init__(self, config, message_bus=None, level=Level.INFO):
        '''
        Initializes the DistanceSensor.

        :param config:        the application configuration
        :param level:         the logging Level
        '''
        self._log = Logger(DistanceSensor.NAME, level)
        if config is None:
            raise ValueError('no configuration provided.')
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        _cfg = config['kros'].get('hardware').get('distance_sensor')
        self._pin = _cfg.get('pin') # pin connected to the center sensor
        self._task_name = '__distance-sensor-loop'
        self._timeout         = _cfg.get('timeout')     # time in seconds to consider sensor as timed out
        self._smoothing       = _cfg.get('smoothing')   # enable smoothing of distance readings
        _smoothing_window     = _cfg.get('smoothing_window')
        self._window = deque(maxlen=_smoothing_window) if self._smoothing else None
        self._loop_interval   = _cfg.get('loop_interval') # interval between distance polling, in seconds
        self._distance        = -1
        self._thread          = None
        self._running         = False
        self._use_message_bus = False
        self._last_read_time  = time.time()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin, GPIO.IN)
        self._log.info('distance sensor ready on pin {}.'.format(self._pin))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return DistanceSensor.NAME

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _measure_pulse_width(self):
        '''
        Polls the GPIO pin to measure pulse width in microseconds.
        Returns None if no valid pulse found within timeout.
        '''
        timeout = time.perf_counter() + 0.05
        # Wait for rising edge (start of pulse)
        while GPIO.input(self._pin) == 0:
            if time.perf_counter() > timeout:
                return None
        start = time.perf_counter()
        # Wait for falling edge (end of pulse)
        timeout = time.perf_counter() + 0.05
        while GPIO.input(self._pin) == 1:
            if time.perf_counter() > timeout:
                return None
        end = time.perf_counter()
        pulse_width_us = (end - start) * 1e6  # microseconds
        self._last_read_time = time.time()
        self._log.debug(f"Measured pulse width: {pulse_width_us:.1f} us")
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
                self._window.append(distance_mm)
                distance = int(sum(self._window) / len(self._window))
            else:
                distance = int(distance_mm)
        else:
            self._log.debug(f"Pulse width {pulse_width_us} out of expected sensor range.")
        return distance

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def distance(self):
        '''
        Get the last computed distance in millimeters as a property.
        Note that this does not return the current value.
        '''
        return self._distance

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
    def _sensor_loop(self):
        '''
        Loop to continuously compute distances.
        '''
        while self._running:
            self._distance = self._compute_distance()
            time.sleep(self._loop_interval)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        '''
        Stop the sensor's loop and clean up resources.
        '''
        self._running = False
        if self._thread:
            try:
                self._thread.join(timeout=0.2)
            except Exception as e:
                self._log.warning('{} raised joining thread: {}.'.format(type(e), e))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enable the sensor, starting the polling loop.
        '''
        Component.enable(self)
        if self.enabled:
            self._running = True
            self._thread = Thread(name='sensor-loop', target=self._sensor_loop)
            self._thread.start()
        else:
            self._log.warning('failed to enable distance sensor.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable the sensor and clean up resources.
        '''
        self.stop()
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
