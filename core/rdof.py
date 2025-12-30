#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-12-19
# modified: 2025-12-19

from enum import Enum

class RDoF(Enum):
    ROLL  = ( 0, 'roll' )
    PITCH = ( 1, 'pitch' )
    YAW   = ( 2, 'yaw' )

    def __init__(self, num, label):
        self._label = label

    @property
    def label(self):
        return self._label

#EOF
