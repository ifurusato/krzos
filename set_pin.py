#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-18
# modified: 2024-05-18
#
# A handy script for setting a GPIO pin high or low.
#

import sys
import time
try:
    import RPi.GPIO as GPIO
except Exception:
    print('This script requires the RPi.GPIO module.\nInstall with: sudo pip3 install RPi.GPIO')
    sys.exit(1)

_pin = 12

try:

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(_pin, GPIO.OUT)

    while True:
        GPIO.output(_pin, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(_pin, GPIO.LOW)
        time.sleep(1)

except KeyboardInterrupt:
    print('caught Ctrl-C.')
finally:
    GPIO.cleanup(_pin)

#EOF
