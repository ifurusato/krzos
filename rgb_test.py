#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-08-25
# modified: 2025-08-25

import time

from hardware.rgb_led import RGBLED
from hardware.color import Color

def main():
    try:
        print("Initializing…")
        led_control = RGBLED(0.667)
        print("\nsetting LED to RED…")
        led_control.set_color(Color.RED)
        time.sleep(2)
        print("\nsetting LED to GREEN…")
        led_control.set_color(Color.GREEN)
        time.sleep(2)
        print("\nsetting LED to BLUE…")
        led_control.set_color(Color.BLUE)
        time.sleep(2)
    finally:
        print("\nsetting LED off...")
        led_control.cleanup()
        print("\ncomplete.")

if __name__ == "__main__":
    main()

