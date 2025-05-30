#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-08-23
# modified: 2024-08-07 - using time.perf_counter()
#

import time
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Rate():
    '''
    By calling wait() faster than the set frequency, results in looping at a
    fixed rate, specified in hertz (Hz).

    :param hertz:   the frequency of the loop in Hertz
    :param level:   the log level
    :param use_ns:  (optional) if True use a nanosecond counter instead of milliseconds
    '''
    def __init__(self, hertz, level=Level.INFO, use_ns=False):
        self._log = Logger('rate', level)
        self._last_ns   = time.perf_counter_ns()
        self._last_time = time.perf_counter()
        self._dt_s = 1/hertz
        self._dt_ms = self._dt_s * 1000
        self._dt_ns = self._dt_ms * 1000000
        self._use_ns = use_ns
        if self._use_ns:
            self._log.info('nanosecond rate set for {:d}Hz (period: {:>6.4f}sec/{:d}ms)'.format(hertz, self.get_period_sec(), self.get_period_ms()))
        elif isinstance(hertz, int):
            self._log.info('millisecond rate set for {:d}Hz (period: {:>6.4f}sec/{:d}ms)'.format(hertz, self.get_period_sec(), self.get_period_ms()))
        else:
            self._log.info('millisecond rate set for {:5.2f}Hz (period: {:>6.4f}sec/{:d}ms)'.format(hertz, self.get_period_sec(), self.get_period_ms()))
        self._trim = 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_period_sec(self):
        '''
        Returns the period in seconds, as a float.
        '''
        return self._dt_s

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def trim(self):
        '''
        Returns the loop trim value in milliseconds. Default is zero.
        Note that the trim feature is only used in the millisecond timer
        mode, not the nanosecond timer.
        '''
        return self._trim

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @trim.setter
    def trim(self, trim_s):
        '''
        Permits auto-adjustment of loop trim should this be needed.
        Note that the trim feature is only used in the millisecond timer
        mode, not the nanosecond timer. Note that the argument's units
        are seconds, not milliseconds.
        '''
        if trim_s < self._dt_s:
            self._trim = trim_s
            self._log.debug('trim set to: {:9.7f}s'.format(self._trim))
        else:
            self._log.warning('trim argument {:8.5f}s larger than dt ({:8.5f}s): ignored.'.format(trim_s, self._dt_s))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def dt_ms(self):
        '''
        Returns the time loop delay in milliseconds.
        This is set by configuration.
        '''
        return self._dt_ms

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_period_ms(self):
        '''
        Returns the period in milliseconds, rounded to an int.
        This is set by configuration.
        '''
        return round(self._dt_s * 1000)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def waiting(self):
        '''
        Return True if still waiting for the current loop to complete.
        '''
        return self._dt_s < ( time.perf_counter() - self._last_time )

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def wait(self):
        '''
        If called before the allotted period will wait the remaining time
        so that the loop is no faster than the specified frequency. If
        called after the allotted period has passed, no waiting takes place.

        E.g.:

            # execute loop at 60Hz
            rate = Rate(60)

            while True:
                # do something...
                rate.wait()
        '''
        if self._use_ns:
            _ns_diff = time.perf_counter_ns() - self._last_ns
            _delay_sec = ( self._dt_ns - _ns_diff ) / ( 1000 * 1000000 )
            if self._dt_ns > _ns_diff:
                time.sleep(_delay_sec)
            self._last_ns = time.perf_counter_ns()
        else:
            _diff = time.perf_counter() - self._last_time
            _delay_sec = self._dt_s - _diff
            # adjust for error
            if _delay_sec + self._trim > 0.0:
                _delay_sec += self._trim
            if self._dt_s > _diff:
                time.sleep(_delay_sec)
            else:
                self._log.debug('no additional delay in rate loop (diff: {:7.4f}ms)'.format(_diff * 1000.0))
            self._last_time = time.perf_counter()

#EOF
