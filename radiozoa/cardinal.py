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
    _by_id    = {}

    def __init__(self, num, abbrev, name, pixel, degrees, radians):
        self._id      = num
        self._abbrev  = abbrev
        self._name    = name
        self._pixel   = pixel
        self._degrees = degrees
        self._radians = radians
        Cardinal._registry.append(self)
        Cardinal._by_id[num] = self

    @property
    def id(self):
        return self._id

    @property
    def name(self):
        return self._name

    @property
    def pixel(self):
        return self._pixel

    @property
    def abbrev(self):
        return self._abbrev

    @property
    def degrees(self):
        return self._degrees

    @property
    def radians(self):
        return self._radians

    @classmethod
    def from_id(cls, num):
        return cls._by_id.get(num, None)

    def __eq__(self, other):
        if isinstance(other, Cardinal):
            return self._name == other._name
        return False

    def __hash__(self):
        return hash(self._name)

    def __repr__(self):
        return self._name

NORTH     = Cardinal(0, 'N',  'north',      13,   0, 0.0 )
NORTHEAST = Cardinal(1, 'NE', 'north-east', 16,  45, π * 0.25 )
EAST      = Cardinal(2, 'E',  'east',       19,  90, π * 0.50 )
SOUTHEAST = Cardinal(3, 'SE', 'south-east', 22, 135, π * 0.75 )
SOUTH     = Cardinal(4, 'S',  'south',       1, 180, π )
SOUTHWEST = Cardinal(5, 'SW', 'south-west',  4, 225, π * 1.25 )
WEST      = Cardinal(6, 'W',  'west',        7, 270, π * 1.50 )
NORTHWEST = Cardinal(7, 'NW', 'north-west', 10, 315, π * 1.75 )

#EOF
