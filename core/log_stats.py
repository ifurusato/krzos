#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2020-01-14
# modified: 2025-11-11

class LogStats:
    '''
    Provides a simple count for each call to the Logger.
    '''
    def __init__(self):
        self._data_count     = 0
        self._debug_count    = 0
        self._info_count     = 0
        self._warn_count     = 0
        self._error_count    = 0
        self._critical_count = 0
        pass

    def data_count(self):
        self._data_count += 1

    def debug_count(self):
        self._debug_count += 1

    def info_count(self):
        self._info_count += 1

    def warn_count(self):
        self._warn_count += 1

    def error_count(self):
        self._error_count += 1

    def critical_count(self):
        self._critical_count += 1

    @property
    def counts(self):
        return ( self._data_count, self._debug_count, self._info_count,
                self._warn_count, self._error_count, self._critical_count )

#EOF
