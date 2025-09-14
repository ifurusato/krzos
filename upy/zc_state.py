#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-07-09
# modified: 2025-07-20

class ZeroCrossingState:
    '''
    Pseudo-Enum for Zero Crossing States.
    '''
    _instances = []

    def __init__(self, index, code, description):
        self._index = index
        self._code = code
        self._description = description
        ZeroCrossingState._instances.append(self)

    @property
    def index(self):
        return self._index

    @property
    def code(self):
        return self._code

    @property
    def description(self):
        return self._description

    def __str__(self):
        return self._code

    def __repr__(self):
        return 'ZeroCrossingState({}, {})'.format(self._index, self._code)

    @classmethod
    def all(cls):
        return cls._instances

    @classmethod
    def from_index(cls, index):
        for inst in cls._instances:
            if inst.index == index:
                return inst
        raise ValueError('No ZeroCrossingState with index "{}"'.format(index))

    @classmethod
    def from_code(cls, code):
        for inst in cls._instances:
            if inst.code == code:
                return inst
        raise ValueError('No ZeroCrossingState with code "{}"'.format(code))

# Instantiate your ZC States
ZeroCrossingState.IDLE            = ZeroCrossingState(0, 'IDLE',            'zero-crossing handler is idle')
ZeroCrossingState.DECELERATING    = ZeroCrossingState(1, 'DECELERATING',    'decelerating to zero RPM')
ZeroCrossingState.CONFIRMING_STOP = ZeroCrossingState(2, 'CONFIRMING_STOP', 'confirming motor has stopped physically')
ZeroCrossingState.STOPPED         = ZeroCrossingState(3, 'STOPPED',         'motor is stopped.')
ZeroCrossingState.ACCELERATING    = ZeroCrossingState(4, 'ACCELERATING',    'accelerating to final target RPM')
ZeroCrossingState.COMPLETE        = ZeroCrossingState(5, 'COMPLETE',        'zero-crossing process completed')
ZeroCrossingState.ERROR           = ZeroCrossingState(6, 'ERROR',           'zero-crossing aborted')

#EOF
