#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-01-16
# modified: 2026-01-16

import time
import pytest
from colorama import Fore, Style

from core.logger import Logger, Level
from hardware.stm32_controller import Stm32Controller

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

__log = Logger('test-stm', level=Level.INFO)

def get_response(master, user_msg, expected):
    expected.append(user_msg)
    for i in range(3):
        response = master.send_request(user_msg)
        if response in expected:
            __log.info("[{}] response for {}: ".format(i, user_msg) + Fore.GREEN + "'{}'".format(response))
            return True
        else:
            __log.info("[{}] response for {}: ".format(i, user_msg) + Fore.YELLOW + "'{}'".format(response))
        time.sleep(0.5)
    return False

@pytest.mark.unit
def test_stm():
    print('')
    __log.info("testing if STM32 is available…")

    master = Stm32Controller()
#   master.set_fail_on_exception(True)
    master.enable()

    response = get_response(master, 'odo led off', ['ACK'])
    assert response is True
    time.sleep(0.5)

    response = get_response(master, 'theme cool 13', ['ACK'])
    assert response is True
    time.sleep(1)

    response = get_response(master, 'ring clear', ['ACK'])
    assert response is True

    __log.info(Fore.GREEN + "STM32 is available.")

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
        raise
    finally:
        __log = None

if __name__ == "__main__":
    main()

#EOF
