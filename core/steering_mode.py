#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-22
# modified: 2024-06-03
#
# The SteeringMode enumerates the various modes for steering the robot. They
# generally assume a start from a zero/centered position, with a gradually
# decreasing turn radius. This is true for ACKERMANN, AFRS and OMNIDIRECTIONAL.
# For SIDEWAYS, ROTATE, FORWARD_PIVOT and AFT_PIVOT, either the robot should
# be stopped before changing to these modes, or they need to be gradually
# implemented.
#

from enum import Enum

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SteeringMode(Enum):
    NONE              = (  0, 'none',                     'none') # used only for reset
    ACKERMANN         = (  1, 'ackermann',                'ackm')
    SKID              = (  2, 'skid',                     'skid') # AKA tank steering
    AFRS              = (  3, 'afrs',                     'afrs') # synchro front and rear angles
    CRAB_PORT         = (  4, 'crab port',                'crbp')
    CRAB_STBD         = (  5, 'crab starboard',           'crbs')
    SIDEWAYS          = (  6, 'sideways',                 'side')
    ROTATE            = (  7, 'rotate',                   'rota')
    ROTATE_CW         = (  8, 'rotate clockwise',         'rotc')
    ROTATE_CCW        = (  9, 'rotate counter-clockwise', 'rotw') # widdershins
    FORWARD_PIVOT     = ( 10, 'forward-pivot',            'fpvt')
    AFT_PIVOT         = ( 11, 'aft-pivot',                'apvt')

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, mnemonic):
        self._name  = name
        self._mnemonic = mnemonic

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def mnemonic(self):
        return self._mnemonic

#EOF
