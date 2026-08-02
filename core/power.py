#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-03-13
# modified: 2026-03-13
#
# An enum for expressing the Power levels.

from enum import Enum
from colorama import init, Fore, Style
init()

class Power(Enum):

    POWER_EXTERNAL      = ( 0, 5, 'external',      Fore.BLUE   + 'external supply',       False )
    POWER_4_BARS_GREEN  = ( 1, 4, '4 bars green',  Fore.GREEN  + '4 bars',                False )
    POWER_4_BARS_YELLOW = ( 2, 4, '4 bars yellow', Fore.YELLOW + '4 bars',                False )
    POWER_3_BARS_AMBER  = ( 3, 3, '3 bars amber',  Fore.YELLOW + '3 bars',                False )
    POWER_2_BARS_ORANGE = ( 4, 2, '2 bars orange', Fore.YELLOW + Style.BRIGHT + '2 bars', True )
    POWER_1_BAR_RED     = ( 5, 1, '1 bar red',     Fore.RED    + '1 bar',                 True )
    POWER_UNKNOWN       = ( 5, 0, 'unknown',       Fore.RED    + 'unknown',               True )

    # ignore the first param since it's already set by __new__
    def __init__(self, num, bars, label, message, alerting):
        self._id       = num
        self._bars     = bars
        self._label    = label
        self._message  = message
        self._alerting = alerting

    @property
    def id(self):
        return self._id

    @property
    def bars(self):
        return self._bars

    @property
    def label(self):
        return self._label

    @property
    def message(self):
        return self._message

    @property
    def is_alerting(self):
        '''
        Return True if the level should raise an alert.
        '''
        return self._alerting

    @staticmethod
    def from_index(index):
        '''
        Return the Power enumeration by its index number, raising an exception if no match.
        '''
        if not isinstance(index, int):
            raise TypeError('from_index: expected int argument not {}'.format(type(index)))
        for power in Power:
            if power.id == index:
                return power
        raise ValueError('no match on power index.')

#EOF
