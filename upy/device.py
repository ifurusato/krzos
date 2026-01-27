#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2026-01-27
# modified: 2026-01-27

class Device:
    _registry = []
    '''
    A pseudo-enum for VL53L0X sensor configuration.

    Iterate like an enum

        for dev in Device.all():
            print(dev.label, hex(dev.i2c_address), dev.xshut)

    Lookup by label

        d = Device.by_label("sw5")
        print(d.i2c_address)

    Lookup by index

        d = Device.by_index(3)
        print(d.label)

    '''
    def __init__(self, index, label, i2c_address, xshut):
        self._index = index
        self._label = label
        self._i2c_address = i2c_address
        self._xshut = xshut
        Device._registry.append(self)

    @property
    def index(self):
        return self._index

    @property
    def label(self):
        return self._label

    @property
    def i2c_address(self):
        return self._i2c_address

    @property
    def xshut(self):
        return self._xshut

    def __int__(self):
        return self._index

    def __repr__(self):
        return "Device({}, {}, 0x{:02X}, xshut={})".format(
            self._index, self._label, self.i2c_address, self._xshut
        )

    def __eq__(self, other):
        if isinstance(other, Device):
            return self._index == other._index
        return False

    def __hash__(self):
        return hash(self._index)

    # ---- class helpers ----

    @classmethod
    def all(cls):
        return cls._registry

    @classmethod
    def by_index(cls, index):
        for d in cls._registry:
            if d._index == index:
                return d
        return None

    @classmethod
    def by_label(cls, label):
        key = label.upper()
        for d in cls._registry:
            if d._label.upper() == key:
                return d
        return None

    @classmethod
    def by_i2c(cls, address):
        for d in cls._registry:
            if d._i2c_address == address:
                return d
        return None

'''
STM32F405 Radiozoa Pinout

    dir     xshut     pin    wire color
    N0      XS0       C4     red/grey
    NE1     XS1       B2     red/white
    E2      XS2       A7     green/grey
    SE3     XS3       B1     green/white
    S4      XS4       A6     blue/grey
    SW5     XS5       B0     blue/white
    W6      XS6       A5     grey
    NW7     XS7       C5     white

'''
N0  = Device(0, 'N0',  0x30, 0)
NE1 = Device(1, 'NE1', 0x31, 1)
E2  = Device(2, 'E2',  0x32, 2)
SE3 = Device(3, 'SE3', 0x33, 3)
S4  = Device(4, 'S4',  0x34, 4)
SW5 = Device(5, 'SW5', 0x35, 5)
W6  = Device(6, 'W6',  0x36, 6)
NW7 = Device(7, 'NW7', 0x37, 7)

