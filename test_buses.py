#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-09-05
# modified: 2025-09-05
#

import os
import pytest
from colorama import Fore, Style

from core.logger import Logger, Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

__log = Logger('test-buses', level=Level.INFO)

@pytest.mark.unit
def test_buses():
    __log.info("testing if I²C and SPI buses are enabled…")
    _i2c_enabled = os.path.exists("/dev/i2c-1")
    _spi_enabled = os.path.exists("/dev/spidev0.0") or os.path.exists("/dev/spidev0.1")
    if _i2c_enabled:
        __log.info(Fore.GREEN + "I²C is enabled.")
    else:
        __log.warning("I²C is not enabled.")
    if _spi_enabled:
        __log.info(Fore.GREEN + "SPI is enabled.")
    else:
        __log.warning("SPI is not enabled.")
    assert _i2c_enabled
    assert _spi_enabled
    __log.info(Fore.GREEN + "I²C and SPI buses are enabled.")

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
