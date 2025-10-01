#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-09-27
# modified: 2025-09-27

import depthai as dai
import pytest
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

__log = Logger('test-oakdlite', level=Level.INFO)

@pytest.mark.unit
def test_oakd_lite_available():
    '''
    Test to confirm that an OAK-D Lite depth camera is available.
    '''
    __log.info(Style.BRIGHT + "testing availability of OAK-D Lite depth camera…")
    try:
        # check for available devices
        devices = dai.Device.getAllAvailableDevices()
        assert len(devices) > 0, "no OAK-D Lite (DepthAI) devices found"

        __log.info("trying to connect to: {}…".format(devices[0]))
        device = dai.Device(devices[0])
        __log.info("device connected: {}".format(device))
        __log.info("available camera sensors: {}".format(device.getCameraSensorNames()))
        deviceInfo = device.getDeviceInfo()
        __log.info(Style.DIM + 'device id: {}'.format(deviceInfo.deviceId))
        __log.info(Style.DIM + 'device name: {}'.format(deviceInfo.name))
        __log.info(Style.DIM + 'device desc: {}'.format(deviceInfo.getXLinkDeviceDesc()))
        __log.info(Style.DIM + 'device platform: {}'.format(deviceInfo.platform))
        __log.info(Style.DIM + 'device state: {}'.format(deviceInfo.state)) # XLinkDeviceState.X_LINK_BOOTED
        __log.info(Style.DIM + 'device status: {}'.format(deviceInfo.status)) # XLinkError_t.X_LINK_SUCCESS
        assert deviceInfo.state == dai.XLinkDeviceState.X_LINK_BOOTED, 'expected BOOTED status, not: {}'.format(deviceInfo.state)
        assert deviceInfo.status == dai.XLinkError_t.X_LINK_SUCCESS, 'expected SUCCESS status, not: {}'.format(deviceInfo.status)
        __log.info(Fore.GREEN + "OAK-D Lite depth camera is ready.")
    except Exception as e:
        pytest.fail('unable to connect to OAK-D Lite depth camera: {}'.format(e))

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

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
