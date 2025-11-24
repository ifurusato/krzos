#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-24
# modified: 2025-11-24

import time
from machine import RTC

class TimeStamp:
    '''
    Utility class to store and format event timestamps using RTC.
    '''
    def __init__(self):
        self._rtc_tuple = None   # for formatted output
        self._tick_mark = None   # for elapsed time

    @property
    def marked(self):
        '''
        Returns True if the mark has been set.
        '''
        return self._rtc_tuple is not None

    def mark(self):
        '''
        Mark the current time as the event time.
        '''
        self._rtc_tuple = RTC().datetime()
        self._tick_mark = time.ticks_ms()

    def clear(self):
        '''
        Clear the marker.
        '''
        self._rtc_tuple = None
        self._tick_mark = None

    def raw(self):
        '''
        Return raw RTC tuple, or None if never marked.
        '''
        return self._rtc_tuple

    def iso(self):
        '''
        Return ISO-formatted string, or None if never marked.
        '''
        if self._rtc_tuple is None:
            return None
        ts = self._rtc_tuple
        return "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}".format(*ts[0:6])

    def elapsed(self):
        '''
        Return seconds since last mark, or None if never marked.
        '''
        if self._tick_mark is None:
            return None
        return time.ticks_diff(time.ticks_ms(), self._tick_mark) / 1000

#EOF
