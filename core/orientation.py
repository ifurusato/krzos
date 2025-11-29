#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2019-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2019-12-23
# modified: 2025-09-11
#
# An enum for expressing different orientations.

from enum import Enum

class Orientation(Enum):
    NONE  = (  0, "none",          'NONE',  "none")
    PORT  = (  1, "port",          'PORT',  "port")
    STBD  = (  2, "starboard",     'STBD',  "stbd")
    CNTR  = (  3, "center",        'CNTR',  "cntr")
    FWD   = (  4, "fwd",           'NONE',  "fwd")
    AFT   = (  5, "aft",           'NONE',  "aft")
    PSID  = (  6, "port-side",     'PORT',  "psid")
    SSID  = (  7, "stbd-side",     'STBD',  "ssid")
    PFWD  = (  8, "port-fwd",      'PORT',  "pfwd")
    SFWD  = (  9, "starboard-fwd", 'STBD',  "sfwd")
    PAFT  = ( 10, "port-aft",      'PORT',  "paft")
    SAFT  = ( 11, "starboard-aft", 'STBD',  "saft")
    MAST  = ( 12, "mast",          'NONE',  "mast")
    PIR   = ( 13, "pir",           'NONE',  "pir")
    ALL   = ( 14, "all",           'NONE',  "all") # all extant orientations

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, side, label):
        self._name  = name
        self._side  = side
        self._label = label

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def name(self):
        '''
        Return the name. This makes sure the name is read-only.
        '''
        return self._name

    @property
    def label(self):
        '''
        Return the label. This makes sure the label is read-only.
        '''
        return self._label

    @property
    def side(self):
        '''
        Return the PORT or STBD side of this orientation, NONE if it does not apply.
        '''
        if self._side == 'PORT':
            return Orientation.PORT
        if self._side == 'STBD':
            return Orientation.STBD
        else:
            return Orientation.NONE

    @staticmethod
    def from_label(label):
        '''
        Returns the Orientation matching the label or None.
        '''
        for o in Orientation:
            if label == o.label:
                return o
        return None

#EOF
