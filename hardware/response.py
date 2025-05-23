#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-13
# modified: 2025-05-13
#

import sys, traceback
from smbus import SMBus
from enum import Enum

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Response(Enum):
    # this variable must include all entries, whitespace-delimited
    __order__ = " INIT OKAY BAD_ADDRESS BAD_REQUEST OUT_OF_SYNC INVALID_CHAR SOURCE_TOO_LARGE UNVALIDATED EMPTY_PAYLOAD PAYLOAD_TOO_LARGE BUSY SKIPPED CONNECTION_ERROR RUNTIME_ERROR UNKNOWN_ERROR PIR_ACTIVE PIR_IDLE VALUE_OFF VALUE_ON "
    '''
    Provides an enumeration of response codes from the I2C Slave.
    These match the hard-coded values in the MicroPython file.
    '''
    INIT              = (  0, 'init',              0x10 )
    OKAY              = (  1, 'okay',              0x4F )
    BAD_ADDRESS       = (  2, 'bad address',       0x71 )
    BAD_REQUEST       = (  3, 'bad request',       0x72 )
    OUT_OF_SYNC       = (  4, 'out of sync',       0x73 )
    INVALID_CHAR      = (  5, 'invalid character', 0x74 )
    SOURCE_TOO_LARGE  = (  6, 'source too large',  0x75 )
    UNVALIDATED       = (  7, 'unvalidated',       0x76 )
    EMPTY_PAYLOAD     = (  8, 'empty payload',     0x77 )
    PAYLOAD_TOO_LARGE = (  9, 'payload too large', 0x78 )
    BUSY              = ( 10, 'busy',              0x79 )
    SKIPPED           = ( 11, 'busy',              0x80 )
    CONNECTION_ERROR  = ( 12, 'connection error',  0x81 )
    RUNTIME_ERROR     = ( 13, 'runtime error',     0x82 )
    UNKNOWN_ERROR     = ( 14, 'unknown error',     0x83 )

    PIR_ACTIVE        = ( 15, 'off',               0x30 )
    PIR_IDLE          = ( 16, 'on',                0x31 )

    # example extension
    VALUE_OFF         = ( 17, 'off',               0x20 )
    VALUE_ON          = ( 18, 'on',                0x21 )

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, value):
        self._num   = num
        self._name  = name
        self._value = value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def num(self):
        '''
        Returns the original enum numerical value.
        '''
        return self._num

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def value(self):
        return self._value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def from_value(value):
        for r in Response:
            if value == r.value:
                return r
        return Response.UNKNOWN_ERROR

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __str__(self):
        return 'Response.{}; value={:4.2f}'.format(self.name, self._value )

#EOF
