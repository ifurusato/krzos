#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-09-15
# modified: 2025-09-15
#

import pytest
import subprocess
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

__log = Logger('test-bluetooth', level=Level.INFO)

@pytest.mark.unit
def test_bluetooth_state():
    '''
    Checks if Bluetooth is enabled.
    '''
    __log.info("testing Bluetooth is enabled…")
    try:
        output = subprocess.check_output("sudo systemctl status bluetooth", shell=True).decode()
        assert "active (running)" in output
        __log.info(Fore.GREEN + "Bluetooth is enabled.")
    except subprocess.CalledProcessError:
        assert False

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def main():
    global __log
    """
    Runs only the marked unit tests within this file when executed directly.
    """
    try:
        pytest.main([__file__, "-m", "unit"])
        __log.info("test execution complete.")
    except Exception as e:
        __log.error("an unexpected error occurred: {}".format(e))
    finally:
        __log         = None

if __name__ == "__main__":
    main()

#EOF
