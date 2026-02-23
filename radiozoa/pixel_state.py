#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License.
#
# author:   Ichiro Furusato
# created:  2026-02-09
# modified: 2026-02-11

from colors import *

class PixelState:
    def __init__(self, color=COLOR_BLACK, phase=0.0):
        self.base_color = color
        self.color = color.rgb
        self.phase = phase
        self._pixel_off_pending = False

    def is_active(self):
        return self.base_color != COLOR_BLACK

    def reset(self):
        self.base_color = COLOR_BLACK
        self.color = self.base_color.rgb
        self.phase = 0.0

#EOF
