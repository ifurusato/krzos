#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2026-01-27
# modified: 2026-02-17
#
# Instances defined at bottom.

class Device:
    _registry = []
    '''
    A pseudo-enum for VL53L0X or VL53L1X sensor configuration.

    The value for impl() will either be 'VL53L0X', 'VL53L1X' or None (if no sensor is available in that slot).

    Usage:

      Iterate like an enum:

          for dev in Device.all():
              print(dev.label, hex(dev.i2c_address), dev.xshut)

      Lookup by label:

          d = Device.by_label("sw5")
          print(d.i2c_address)

      Lookup by index:

          d = Device.by_index(3)
          print(d.label)
    '''
    def __init__(self, index, impl, label, i2c_address, xshut):
        self._index = index
        self._impl  = impl
        self._label = label
        self._i2c_address = i2c_address
        self._xshut = xshut
        Device._registry.append(self)

    @property
    def index(self):
        return self._index

    @property
    def impl(self):
        return self._impl

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

#            IDX  IMPL       DIR   ADDR   PIN    WIRE COLOR
N0  = Device( 0, 'VL53L1X', 'N0',  0x30,  3) # red/grey
NE1 = Device( 1, 'VL53L1X', 'NE1', 0x31, 35) # red/white
E2  = Device( 2, 'VL53L1X', 'E2',  0x32,  2) # green/grey
SE3 = Device( 3, 'VL53L1X', 'SE3', 0x33, 36) # green/white
S4  = Device( 4, 'VL53L1X', 'S4',  0x34,  1) # blue/grey
SW5 = Device( 5, 'VL53L1X', 'SW5', 0x35, 37) # blue/white
W6  = Device( 6, 'VL53L1X', 'W6',  0x36,  0) # grey
NW7 = Device( 7, 'VL53L1X', 'NW7', 0x37, 43) # white

#EOF
