#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-27
# modified: 2025-05-27

from colorama import Fore, Style

from abstract_display import AbstractDisplay

class TinyFxDisplay(AbstractDisplay):

    def __init__(self, rgb_led=None):
        self._rgb_led = rgb_led
        self._verbose = False

    def show_color(self, color):
        if self._rgb_led:
            self._rgb_led.set_rgb(*color)
        elif self._verbose:
            print(Fore.CYAN + "show color: {}".format(*color) + Style.RESET_ALL)

#EOF
