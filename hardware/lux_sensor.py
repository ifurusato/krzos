#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-09-29
# modified: 2025-09-29

import time
from ltr559 import LTR559
from threading import Thread, Event
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.component import Component

class LuxSensor(Component):
    '''
    Represents a light sensor component using the LTR559 sensor.

    This class provides an interface for retrieving the current lux level and monitoring
    changes in ambient light relative to a configurable threshold. It supports asynchronous
    monitoring via a background thread, and allows user-supplied callback functions to be
    invoked when the lux value crosses the threshold (both rising and falling edges).
    '''
    def __init__(self, config=None, level=Level.INFO):
        self._log = Logger('lux-sensor', level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        _cfg = config['kros'].get('hardware').get('lux_sensor')
        self._lux_threshold = _cfg.get('lux_threshold')
        self._log.info('lux threshold set to: {:.2f}.'.format(self._lux_threshold))
        self._ltr559 = LTR559()
        self._stop_event          = Event()
        self._monitor_thread      = None
        self._rising_callback     = None
        self._falling_callback    = None
        self._was_above_threshold = None # last known state
        self._log.info('ready.')

    @property
    def lux(self):
        '''
        Returns the current measured lux value.
        '''
        self._ltr559.update_sensor()
        return self._ltr559.get_lux()

    def get_lux_threshold(self):
        '''
        Returns the configured lux threshold.
        '''
        return self._lux_threshold

    def start_monitoring(self, rising_callback=None, falling_callback=None, poll_interval=0.05):
        '''
        Starts background monitoring and registers callbacks.
        '''
        self._rising_callback  = rising_callback
        self._falling_callback = falling_callback
        self._stop_event.clear()
        self._monitor_thread = Thread(target=self._monitor, args=(poll_interval,), daemon=True)
        self._monitor_thread.start()
        self._log.info('started monitoring thread.')

    def stop_monitoring(self):
        '''
        Stops background monitoring.
        '''
        self._stop_event.set()
        if self._monitor_thread:
            self._monitor_thread.join()
        self._log.info('stopped monitoring thread.')

    def _monitor(self, poll_interval):
        while not self._stop_event.is_set():
            lux = self.lux
            above = lux > self._lux_threshold
            if self._was_above_threshold is None:
                # first run: just set state
                self._was_above_threshold = above
            elif above and not self._was_above_threshold:
                # rising edge: crossed threshold upwards
                self._was_above_threshold = True
                if self._rising_callback:
                    self._rising_callback(lux)
            elif not above and self._was_above_threshold:
                # falling edge: crossed threshold downwards
                self._was_above_threshold = False
                if self._falling_callback:
                    self._falling_callback(lux)
            # else, no state change
            time.sleep(poll_interval)

    def disable(self):
        self.stop_monitoring()
        return super().disable()

    def close(self):
        return super().close()

#EOF
