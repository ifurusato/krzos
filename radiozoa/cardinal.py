#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-14
# modified: 2026-01-29

from math import pi as π

class Cardinal:
    _registry = []

    def __init__(self, num, name, abbrev, degrees, radians):
        self._id      = num
        self._name    = name
        self._abbrev  = abbrev
        self._degrees = degrees
        self._radians = radians
        Cardinal._registry.append(self)

    @property
    def id(self):
        return self._id

    @property
    def name(self):
        return self._name

    @property
    def abbrev(self):
        return self._abbrev

    @property
    def degrees(self):
        return self._degrees

    @property
    def radian(self):
        return self._radian

    def __eq__(self, other):
        if isinstance(other, Cardinal):
            return self._name == other._name
        return self == other

    def __hash__(self):
        return hash(self._name)

    def __repr__(self):
        return self._name

NORTH     = (0, 'N',  'north',        0, 0.0 )
NORTHEAST = (1, 'NE', 'north-east',  45, π * 0.25 )
EAST      = (2, 'E',  'east',        90, π * 0.50 )
SOUTHEAST = (3, 'SE', 'south-east', 135, π * 0.75 )
SOUTH     = (4, 'S',  'south',      180, π )
SOUTHWEST = (5, 'SW', 'south-west', 225, π * 1.25 )
WEST      = (6, 'W',  'west',       270, π * 1.50 )
NORTHWEST = (7, 'NW', 'north-west', 315, π * 1.75 )
UNKNOWN   = (8, 'U',  'unknown',     -1, -1.0 )

#EOF
