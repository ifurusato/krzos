#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-20
# modified: 2025-05-20
#

import math
from enum import Enum

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Easing(Enum):
    LINEAR      = 'linear'       # direct proportional mapping 
    QUADRATIC   = 'quadratic'    # values decrease quickly near max, then flatten near min 
    CUBIC       = 'cubic'        # decreases very fast initially, then barely changes close to obstacles
    SQUARE_ROOT = 'square_root'  # maintains higher values longer, drops more gradually near obstacles
    LOGARITHMIC = 'logarithmic'  # keeps values high over a wide range, drops off sharply only near minimum
    SIGMOID     = 'sigmoid'      # smooth, balanced transition; avoids abrupt changes

    @property
    def name(self):
        return self.value

    def apply(self, normalised: float) -> float:
        '''
        Apply the selected easing function to the normalised value.
        '''
        match self:
            case Easing.LINEAR:
                return normalised
            case Easing.QUADRATIC:
                return normalised ** 2
            case Easing.CUBIC:
                return normalised ** 3
            case Easing.SQUARE_ROOT:
                return math.sqrt(normalised)
            case Easing.LOGARITHMIC:
                '''
                log1p(9x)/log1p(9) keeps range [0,1]

                Value  Behaviour       Effect on Curve                            Responsiveness
                1      linear-ish      very shallow log curve, close to linear    fast response — not much easing
                3      mild easing     starts higher, drops gently                medium responsiveness
                5      moderate        flatter curve near 0                       less responsive until mid-range
                9      strong          very flat early, sharp drop at end         conservative; waits to respond
                20     very strong     stays almost flat until ~80%               very late response; very cautious
                '''
                log_scaling_factor = 7
                return math.log1p(normalised * log_scaling_factor) / math.log1p(log_scaling_factor)
            case Easing.SIGMOID:
                sigmoid_sharpness = 6   # reduce sharpness to make the deceleration more gradual (was 4)
                midpoint          = 0.4 # shift the midpoint of the sigmoid to start decelerating earlier (was 0.22)
                return 1 / (1 + math.exp(-sigmoid_sharpness * (normalised - midpoint)))
            case _:
                raise ValueError("Unknown easing type: {}".format(self))

    @classmethod
    def from_string(cls, name: str):
        '''
        Convert a string to an Easing enum member.
        Case-insensitive. Raises ValueError on invalid name.
        '''
        name = name.strip().lower()
        for member in cls:
            if member.value == name:
                return member
        raise ValueError("'{}' is not a valid Easing type. Available options: {}".format(name, [e.value for e in cls]))

