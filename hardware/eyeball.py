#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-10-23
# modified: 2025-11-09
#
# The basic idea:
#
#        XXX
#       X   X
#       X   X
#        XXX
#     
#         X
#        X X
#       X   X
#            
#       X
#         X
#           X
#         X
#       X
#           
#       XXXXX
#         X
#         X
#         X
#              
#       X   X
#        X X
#         X
#        X X
#       X   X

from enum import Enum

from hardware.color import Color

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Eyeball(Enum):
    NORMAL    = ( 0, 'normal', Color.GREEN, 
            [
                [ 0, 1, 1, 1, 0 ],
                [ 1, 0, 0, 0, 1 ],
                [ 1, 0, 0, 0, 1 ],
                [ 1, 0, 0, 0, 1 ],
                [ 0, 1, 1, 1, 0 ]
            ])
    HAPPY     = ( 1, 'happy', Color.YELLOW_GREEN,
            [
                [ 0, 0, 0, 0, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 1, 0, 1, 0 ],
                [ 1, 0, 0, 0, 1 ],
                [ 0, 0, 0, 0, 0 ]
            ])
    WINK_PORT = ( 2, 'wink-port', Color.YELLOW_GREEN,
            [
                [ 0, 1, 0, 0, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 0, 0, 1, 0 ],
                [ 1, 1, 1, 1, 1 ],
                [ 0, 0, 0, 0, 0 ]
            ])
    WINK_STBD = ( 3, 'wink-stbd', Color.YELLOW_GREEN,
            [
                [ 0, 1, 1, 1, 0 ],
                [ 1, 0, 0, 0, 1 ],
                [ 1, 0, 0, 0, 1 ],
                [ 1, 0, 0, 0, 1 ],
                [ 0, 1, 1, 1, 0 ]
            ])
    BLUSH     = ( 4, 'blush', Color.PINK,
            [
                [ 0, 0, 0, 0, 0 ],
                [ 1, 0, 0, 0, 1 ],
                [ 1, 0, 0, 0, 1 ],
                [ 0, 1, 1, 1, 0 ],
                [ 0, 0, 0, 0, 0 ]
            ])
    LOOK_STBD = ( 5, 'look-stbd', Color.ORANGE,
            [
                [ 0, 0, 0, 1, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 1, 0, 0, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 0, 0, 1, 0 ]
            ])
    LOOK_PORT = ( 6, 'look-port', Color.ORANGE,
            [
                [ 0, 1, 0, 0, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 0, 0, 1, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 1, 0, 0, 0 ]
            ])
    LOOK_UP   = ( 7, 'look-up', Color.ORANGE,
            [
                [ 0, 0, 0, 0, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 1, 0, 1, 0 ],
                [ 1, 0, 0, 0, 1 ],
                [ 0, 0, 0, 0, 0 ]
            ])
    LOOK_DOWN = ( 8, 'look-down', Color.ORANGE,
            [
                [ 0, 0, 0, 0, 0 ],
                [ 1, 0, 0, 0, 1 ],
                [ 0, 1, 0, 1, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 0, 0, 0, 0 ]
            ])
    CONFUSED_PORT = ( 9, 'confused-port', Color.CORAL,
            [
                [ 0, 1, 0, 0, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 0, 0, 1, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 1, 0, 0, 0 ]
            ])
    CONFUSED_STBD = ( 10, 'confused-stbd', Color.CORAL,
            [
                [ 0, 0, 0, 1, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 1, 0, 0, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 0, 0, 1, 0 ]
            ])
    BORED     = ( 11, 'bored', Color.GREY_ORANGE,
            [
                [ 0, 0, 0, 0, 0 ],
                [ 0, 0, 0, 0, 0 ],
                [ 1, 0, 0, 0, 1 ],
                [ 0, 1, 1, 1, 0 ],
                [ 0, 0, 0, 0, 0 ]
            ])
    SLEEPY    = ( 12, 'sleepy', Color.DARK_GREY,
            [
                [ 0, 0, 0, 0, 0 ],
                [ 0, 0, 0, 0, 0 ],
                [ 1, 0, 0, 0, 1 ],
                [ 1, 0, 0, 0, 1 ],
                [ 0, 1, 1, 1, 0 ]
            ])
    WOW1      = ( 13, 'wow-1', Color.YELLOW,
            [
                [ 0, 0, 0, 0, 0 ],
                [ 0, 0, 0, 0, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 0, 0, 0, 0 ],
                [ 0, 0, 0, 0, 0 ]
            ])
    WOW2      = ( 14, 'wow-2', Color.YELLOW,
            [
                [ 0, 0, 0, 0, 0 ],
                [ 0, 1, 1, 1, 0 ],
                [ 0, 1, 0, 1, 0 ],
                [ 0, 1, 1, 1, 0 ],
                [ 0, 0, 0, 0, 0 ]
            ])
    WOW3      = ( 15, 'wow-3', Color.YELLOW,
            [
                [ 0, 1, 1, 1, 0 ],
                [ 1, 0, 0, 0, 1 ],
                [ 1, 0, 0, 0, 1 ],
                [ 1, 0, 0, 0, 1 ],
                [ 0, 1, 1, 1, 0 ]
            ])
    BLANK     = ( 16, 'blank', Color.ORANGE, # was LIGHT_GREY
            [
                [ 0, 0, 0, 0, 0 ],
                [ 0, 0, 0, 0, 0 ],
                [ 1, 1, 1, 1, 1 ],
                [ 0, 0, 0, 0, 0 ],
                [ 0, 0, 0, 0, 0 ]
            ])
    SAD       = ( 17, 'sad', Color.BLUE_VIOLET,
            [
                [ 0, 0, 0, 0, 0 ],
                [ 1, 1, 1, 1, 1 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 0, 1, 0, 0 ]
            ])
    DEAD      = ( 18, 'dead', Color.GREY,
            [
                [ 1, 0, 0, 0, 1 ],
                [ 0, 1, 0, 1, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 1, 0, 1, 0 ],
                [ 1, 0, 0, 0, 1 ]
            ])
    # additional navigation indicators .....................

    LOOK_STBD_FWD = ( 19, 'look-stbd-fwd', Color.BLUE,
            [
                [ 0, 0, 0, 1, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 1, 0, 0, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 0, 0, 1, 0 ]
            ])
    LOOK_PORT_FWD = ( 20, 'look-port-fwd', Color.BLUE,
            [
                [ 0, 1, 0, 0, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 0, 0, 1, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 1, 0, 0, 0 ]
            ])
    LOOK_STBD_AFT = ( 21, 'look-stbd-aft', Color.YELLOW,
            [
                [ 0, 0, 0, 1, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 1, 0, 0, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 0, 0, 1, 0 ]
            ])
    LOOK_PORT_AFT = ( 22, 'look-port-aft', Color.YELLOW,
            [
                [ 0, 1, 0, 0, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 0, 0, 1, 0 ],
                [ 0, 0, 1, 0, 0 ],
                [ 0, 1, 0, 0, 0 ]
            ])
    
    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, color, array):
        self._name  = name
        self._color = color
        self._array = array

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def color(self):
        return self._color

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def array(self):
        return self._array
    
#EOF
