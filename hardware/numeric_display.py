#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-12-23
# modified: 2025-12-23

from matrix11x7 import Matrix11x7
from matrix11x7.fonts import font3x5

from core.logger import Logger, Level
from core.component import Component

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class NumericDisplay(Component):
    NAME = 'irq-clock'

    LOW_BRIGHTNESS    = 0.15
    MEDIUM_BRIGHTNESS = 0.25
    HIGH_BRIGHTNESS   = 0.45

    def __init__(self, brightness=0.25):
        self._log = Logger(NumericDisplay.NAME, Level.INFO)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._matrix = Matrix11x7()
        self._matrix.set_brightness(brightness)
        self._log.info('ready.')

    def show_int(self, n: int) -> None:
        self._matrix.clear()
        self._matrix.write_string("{:3d}".format(n), y=1, font=font3x5)
        self._matrix.show()

    def show_string(self, text: str) -> None:
        if not 1 <= len(text) <= 3:
            raise ValueError("text must be 1 to 3 characters long.")
        self.clear()
        self._matrix.write_string('{:>3}'.format(text), y=1, font=font3x5)
        self._matrix.show()

    def set_brightness(self, brightness):
        self._matrix.set_brightness(brightness)

    def clear(self):
        self._matrix.clear()

    def close(self):
        self.clear()
        self._matrix.show()

#EOF
