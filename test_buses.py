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

@pytest.mark.unit
def test_buses():
    _log = Logger('test-buses', level=Level.INFO)
    _log.info("testing if I2C and SPI buses are enabled…")
    _i2c_enabled = os.path.exists("/dev/i2c-1")
    _spi_enabled = os.path.exists("/dev/spidev0.0") or os.path.exists("/dev/spidev0.1")
    if _i2c_enabled:
        _log.info(Fore.GREEN + "I2C is enabled.")
    else:
        _log.warning("I2C is not enabled.")
    if _spi_enabled:
        _log.info(Fore.GREEN + "SPI is enabled.")
    else:
        _log.warning("SPI is not enabled.")
    assert _i2c_enabled
    assert _spi_enabled

#EOF
