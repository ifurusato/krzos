#}!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-07-29
# modified: 2024-11-15
#

from enum import Enum

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Directive(Enum):
    NO_CHANGE         = (  0, 'no-change',         'noch', 'n')
    WAIT              = (  1, 'wait',              'wait', 'z')
    STOP              = (  2, 'stop',              'stop', 's')
    BRAKE             = (  3, 'brake',             'brak', 'b')
    COAST             = (  4, 'coast',             'cost', 'c')
    AHEAD             = (  5, 'ahead',             'ahed', 'w')
    ASTERN            = (  6, 'astern',            'astn', 'z')
    ROTATE_CW         = (  7, 'rotate-cw',         'rtcw', 'r')
    ROTATE_CCW        = (  8, 'rotate-ccw',        'rtcc', 't')
    CRAB_PORT         = (  9, 'crab-port',         'crap', '<')
    CRAB_STBD         = ( 10, 'crab-stbd',         'cras', '>')
    DIAGONAL_PORT     = ( 11, 'diagonal-port',     'diap', 'd')
    DIAGONAL_STBD     = ( 12, 'diagonal-stbd',     'dias', 'f')
    PIVOT_FWD_CW      = ( 13, 'pivot-fwd-cw',      'pfcw', 'p')
    PIVOT_FWD_CCW     = ( 14, 'pivot-fwd-ccw',     'pfcc', 'o')
    PIVOT_AFT_CW      = ( 15, 'pivot-aft-cw',      'pacw', 'l')
    PIVOT_AFT_CCW     = ( 16, 'pivot-aft-ccw',     'pacc', 'k')
    PIVOT_PORT_CW     = ( 17, 'pivot-port-cw',     'ppcw', 'i')
    PIVOT_PORT_CCW    = ( 18, 'pivot-port-ccw',    'ppcc', 'u')
    PIVOT_STBD_CW     = ( 19, 'pivot-stbd-cw',     'pscw', 'j')
    PIVOT_STBD_CCW    = ( 20, 'pivot-stbd-ccw',    'pscc', 'h')
    PFWD_ONLY         = ( 21, 'pfwd-only',         'pfwd', '1')
    SFWD_ONLY         = ( 22, 'sfwd-only',         'sfwd', '2')
    PAFT_ONLY         = ( 23, 'paft-only',         'paft', '3')
    SAFT_ONLY         = ( 24, 'saft-only',         'saft', '4')
    UNKNOWN           = ( 99, 'unknown',           'unkn', '?') # n/a or indeterminate

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, label, key):
        self._name  = name
        self._label = label
        self._key   = key

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def label(self):
        return self._label

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def key(self):
        return self._key

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_directive_for(port_velocity, stbd_velocity):
        '''
        NOTE: Not yet implemented for enums > 4.
        '''
        if port_velocity and stbd_velocity:
            if port_velocity == 0.0 and stbd_velocity == 0.0:
                return Directive.STOPPED
            elif port_velocity > 0.0 and stbd_velocity > 0.0:
                return Directive.AHEAD
            elif port_velocity < 0.0 and stbd_velocity < 0.0:
                return Directive.ASTERN
            elif port_velocity > 0.0 and stbd_velocity <= 0.0:
                return Directive.CLOCKWISE
            elif port_velocity <= 0.0 and stbd_velocity > 0.0:
                return Directive.COUNTER_CLOCKWISE
            else:
                raise TypeError('unable to discern directive for port: {}; stbd: {}'.format(port_velocity, stbd_velocity))
        else:
            return Directive.UNKNOWN

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_directive_for_key(key):
        '''
        NOTE: Not yet implemented for all enums.
        '''
        for _directive in Directive:
            if _directive.key == key:
                return _directive
        return Directive.UNKNOWN

#EOF
