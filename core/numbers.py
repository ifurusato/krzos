#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2021-07-01
# modified: 2025-12-17
#
# An enumeration of the names of numbers through twenty, including ordinals.

from enum import Enum

class Numbers(Enum):
    ZERO      = ( 0,  'zero',      '0' )
    ONE       = ( 1,  'one',       '1st' )
    TWO       = ( 2,  'two',       '2nd' )
    THREE     = ( 3,  'three',     '3rd' )
    FOUR      = ( 4,  'four',      '4th' )
    FIVE      = ( 5,  'five',      '5th' )
    SIX       = ( 6,  'six',       '6th' )
    SEVEN     = ( 7,  'seven',     '7th' )
    EIGHT     = ( 8,  'eight',     '8th' )
    NINE      = ( 9,  'nine',      '9th' )
    TEN       = ( 10, 'ten',       '10th' )
    ELEVEN    = ( 11, 'eleven',    '11th' )
    TWELVE    = ( 12, 'twelve',    '12th' )
    THIRTEEN  = ( 13, 'thirteen',  '13th' )
    FOURTEEN  = ( 14, 'fourteen',  '14th' )
    FIFTEEN   = ( 15, 'fifteen',   '15th' )
    SIXTEEN   = ( 16, 'sixteen',   '16th' )
    SEVENTEEN = ( 17, 'seventeen', '17th' )
    EIGHTEEN  = ( 18, 'eighteen',  '18th' )
    NINETEEN  = ( 19, 'nineteen',  '19th' )
    TWENTY    = ( 20, 'twenty',    '20th' )

    def __new__(cls, *args, **kwds):
        obj = object.__new__(cls)
        obj._value_ = args[0]
        return obj

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, ordinal):
        self._name = name
        self._ordinal = ordinal

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def name(self):
        return self._name

    @property
    def ordinal(self):
        return self._ordinal

    @staticmethod
    def from_number(num, prefer_ordinal=False):
        '''
        Returns 'zero' through 'twenty', then simply a string
        version of the number.
        '''
        for n in Numbers:
            if n.value == num:
                if prefer_ordinal:
                    return n._ordinal
                else:
                    return n._name
        return str(num)

    def __eq__(self, obj):
        return isinstance(obj, Number) and obj.value == self.value

#EOF
