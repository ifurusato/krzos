#}!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-07-29
# modified: 2025-11-20

from enum import Enum

class Direction(Enum):
    STOPPED           = ( 0, 'stopped',           'stop',  0.0,  0.0)
    AHEAD             = ( 1, 'ahead',             'ahed',  0.0,  1.0)
    ASTERN            = ( 2, 'astern',            'astn',  0.0, -1.0)
    PORT              = ( 3, 'port',              'port', -1.0,  0.0)
    STARBOARD         = ( 4, 'stbd',              'stbd',  1.0,  0.0)
#   CLOCKWISE         = ( 5, 'clockwise',         'clws',  0.0,  0.0)
#   COUNTER_CLOCKWISE = ( 6, 'counter-clockwise', 'ccwz',  0.0,  0.0)
#   UNKNOWN           = ( 7, 'unknown',           'unkn',  0.0,  0.0) # n/a or indeterminate

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, label, vx_direction, vy_direction):
        self._name  = name
        self._label = label
        self._vx_direction = vx_direction
        self._vy_direction = vy_direction

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def name(self):
        return self._name

    @property
    def label(self):
        return self._label

    @property
    def vx_direction(self):
        '''
        Returns the x-component direction multiplier (-1.0, 0.0, or 1.0)
        '''
        return self._vx_direction

    @property
    def vy_direction(self):
        '''
        Returns the y-component direction multiplier (-1.0, 0.0, or 1.0)
        '''
        return self._vy_direction

    @staticmethod
    def get_direction_for(port_velocity, stbd_velocity):
        if port_velocity and stbd_velocity:
            if port_velocity == 0.0 and stbd_velocity == 0.0:
                return Direction.STOPPED
            elif port_velocity > 0.0 and stbd_velocity > 0.0:
                return Direction.AHEAD
            elif port_velocity < 0.0 and stbd_velocity < 0.0:
                return Direction.ASTERN
            elif port_velocity > 0.0 and stbd_velocity <= 0.0:
                return Direction.CLOCKWISE
            elif port_velocity <= 0.0 and stbd_velocity > 0.0:
                return Direction.COUNTER_CLOCKWISE
            else:
                raise TypeError('unable to discern direction for port: {}; stbd: {}'.format(port_velocity, stbd_velocity))
        else:
            return Direction.UNKNOWN

#EOF
