#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-09-21
# modified: 2025-09-21
#

import os
import pytest
import time
from colorama import Fore, Style

from core.logger import Logger, Level
from hardware.thunderborg import ThunderBorg

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

__log = Logger('test-thunderborg', level=Level.INFO)

@pytest.mark.unit
def test_thunderborg():
    __log.info(Style.BRIGHT + "testing if motor controllers are available and functional…")
    TB1 = None
    TB2 = None

    try:
        # setup ThunderBorg 1
        TB1 = ThunderBorg()     # create new ThunderBorg object
        TB1.i2cAddress = 0x15   # set to altered I2C address
        TB1.Init()              # set the board up (checks the board is connected)
        assert TB1.foundChip
        # disable the colour by battery level
        TB1.SetLedShowBattery(False)
        TB1.SetLeds(0.0, 0.0, 0.0)

        # setup ThunderBorg 2
        TB2 = ThunderBorg()     # create new ThunderBorg object
        TB2.i2cAddress = 0x16   # set to altered I2C address
        TB2.Init()              # set the board up (checks the board is connected)
        assert TB2.foundChip
        # disable color by battery level
        TB2.SetLedShowBattery(False)
        TB2.SetLeds(0.0, 0.0, 0.0)

        for _ in range(1):
            for hue in range(300):
                # Get hue into the 0 to 3 range
                hue /= 100.0
                # decide which two channels we are between
                if hue < 1.0: # red to green
                    red   = 1.0 - hue
                    green = hue 
                    blue  = 0.0
                elif hue < 2.0: # green to blue
                    red   = 0.0
                    green = 2.0 - hue
                    blue  = hue - 1.0
                else: # blue to red
                    red   = hue - 2.0
                    green = 0.0 
                    blue  = 3.0 - hue
                TB1.SetLeds(red, green, blue)
                TB2.SetLeds(red, green, blue)
                time.sleep(0.005)

        __log.info(Fore.GREEN + "motor controllers are functional.")

    except Exception as e:
        pytest.fail("{} raised setting up thunderborg: {}".format(type(e), e))
    finally:
        if TB1:
            TB1.SetLeds(0.0, 0.0, 0.0)
        if TB2:
            TB2.SetLeds(0.0, 0.0, 0.0)
        TB2.SetLedShowBattery(True)

def main():
    global __log
    """
    Runs only the marked unit tests within this file when executed directly.
    """
    try:
        pytest.main([__file__, "-m", "unit", "-s"])
        __log.info("test execution complete.")
    except Exception as e:
        __log.error("an unexpected error occurred: {}".format(e))
    finally:
        __log = None

if __name__ == "__main__":
    main()

#EOF
