#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-10
# modified: 2025-05-22
#
# Checks for the existence of a set of expected I2C devices on the KRZ03.
#

import os, sys, time, traceback
import subprocess
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

import RPi.GPIO as GPIO

from core.logger import Logger, Level
from core.util import Util
from core.config_loader import ConfigLoader
from hardware.i2c_scanner import I2CScanner, DeviceNotFound
from hardware.pigpiod_util import PigpiodUtility as PigUtil
#from hardware.tinyfx_driver import TinyFxDriver

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

#RUNNING_LIGHTS   = False
BLINK_ON_COMPLETE = True
MOTOR_2040        = True
TINY_FX           = True
CONFIRM_PIGPIOD   = False
driver            = None

_pin = None

try:

    _log = Logger('init', Level.INFO)

    _log.info("starting…")
    
    _devices='''
    0x0B   Rotary Encoder - stbd    (optional)
    0x0C   Digital Pot - port       (optional)
    0x0E   Digital Pot - stbd       (optional)
    0x0F   Rotary Encoder - port    (optional)
    0x10   GPS
    0x18   IO Expander
    0x1D   LSM303D
    0x29   VL53L1CX or VL53L5CX
    0x38   BH1745                   (optional)
    0x44   Motor 2040               (optional)
    0x45   TinyFX
    0x48   ADS1015
    0x74   RGB LED - stbd
    0x75   11x7 Matrix LED - stbd
    0x77   11x7 Matrix LED - port   (conflict)
    0x77   RGB LED - port
    0x69   ICM20948
'''
    _log.info("devices: \n{}".format(_devices))

    # check for expected devices ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    _config = ConfigLoader(Level.INFO).configure()
    _i2c_scanner = I2CScanner(_config, level=Level.INFO)

    #   0x0B   Rotary Encoder - stbd    (optional)
    #   0x0C   Digital Pot - port       (optional)
    if not _i2c_scanner.has_hex_address(['0x0C']):
        _log.info('Digital Potentiometer not found at address 0x0C.')
    else:
        _log.info(Fore.GREEN + 'Digital Potentiometer found at address 0x0C.')
    #   0x0E   Digital Pot - stbd       (optional)
    if not _i2c_scanner.has_hex_address(['0x0E']):
        _log.info('Digital Potentiometer not found at address 0x0E.')
    else:
        _log.info(Fore.GREEN + 'Digital Potentiometer found at address 0x0E.')
    #   0x0F   Rotary Encoder - port    (optional)
    if not _i2c_scanner.has_hex_address(['0x0F']):
        _log.info(Style.DIM + 'Rotary Encoder not found at address 0x0F.')
    else:
        _log.info(Fore.GREEN + 'Rotary Encoder found at address 0x0F.')
    # I²C address 0x10: PA1010D GPS
    if not _i2c_scanner.has_hex_address(['0x10']):
        _log.info(Style.DIM + 'PA1010D GPS not found at address 0x10.')
    else:
        _log.info(Fore.GREEN + 'PA1010D GPS found at address 0x10.')
    if TINY_FX:
        if not _i2c_scanner.has_hex_address(['0x44']):
            _log.warning('TinyFX not found at address 0x44.')
        else:
            _log.info(Fore.GREEN + 'TinyFX found at address 0x44.')
    if MOTOR_2040:
        if not _i2c_scanner.has_hex_address(['0x45']):
            _log.warning('Motor 2040 not found at address 0x45.')
        else:
            _log.info(Fore.GREEN + 'Motor 2040 found at address 0x45.')
    #   0x48   ADS1015
    if not _i2c_scanner.has_hex_address(['0x48']):
        _log.warning('ADS1015 not found at address 0x48.')
    else:
        _log.info(Fore.GREEN + 'ADS1015 found at address 0x48.')
    #   0x1D   LSM303D
    if not _i2c_scanner.has_hex_address(['0x1D']):
        _log.info(Style.DIM + 'LM303D not found at address 0x1D.')
    else:
        _log.info(Fore.GREEN + 'LM303D found at address 0x1D.')
    #   0x29   VL53L5CX
    if not _i2c_scanner.has_hex_address(['0x29']):
        _log.warning('VL53L1X or VL53L5X not found at address 0x29.')
    else:
        _log.info(Fore.GREEN + 'VL53L1X or VL53L5X found at address 0x29.')
    #   0x38   BH1745                   (optional)
    if not _i2c_scanner.has_hex_address(['0x38']):
        _log.info(Style.DIM + 'BH1745 not found at address 0x38.')
    else:
        _log.info(Fore.GREEN + 'BH1745 found at address 0x1D.')
    #   0x74   5x5 RGB LED - stbd
    if not _i2c_scanner.has_hex_address(['0x74']):
        _log.warning('Starboard 5x5 RGB Matrix not found at address 0x74.')
    else:
        _log.info(Fore.GREEN + 'Starboard 5x5 RGB Matrix found at address 0x74.')
    #   0x75   11x7 Matrix LED - stbd
    if not _i2c_scanner.has_hex_address(['0x75']):
        _log.warning('11x7 RGB Matrix not found at address 0x75.' + Style.DIM + ' (required by ICM20948)')
    else:
        _log.info(Fore.GREEN + '11x7 RGB Matrix found at address 0x75.')
    #   0x77   11x7 Matrix LED - port   (conflict)
    #   0x77   5x5 RGB LED - port
    if not _i2c_scanner.has_hex_address(['0x77']):
        _log.warning('Port 5x5 RGB Matrix not found at address 0x77.')
    else:
        _log.info(Fore.GREEN + 'Port 5x5 RGB Matrix found at address 0x77.')
    #   0x69   ICM20948
    if not _i2c_scanner.has_hex_address(['0x69']):
        _log.warning('ICM20948 not found at address 0x69.')
    else:
        _log.info(Fore.GREEN + 'ICM20948 found at address 0x69.')

    if CONFIRM_PIGPIOD:
        PigUtil.ensure_pigpiod_is_running()
    else:
        _log.info(Style.DIM + 'no check for pigpiod.')

    if BLINK_ON_COMPLETE:
        _pin = 13
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(_pin, GPIO.OUT)
        for _ in range(7):
            GPIO.output(_pin, GPIO.HIGH)
            time.sleep(0.05)
            GPIO.output(_pin, GPIO.LOW)
            time.sleep(0.3)

#   if RUNNING_LIGHTS:
#       driver = TinyFxDriver(_config)
#       driver.command('on')

    _log.info("done.")

except KeyboardInterrupt:
    _log.info('Ctrl-C caught; exiting…')
except Exception as e:
    _log.error('{} thrown by init: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    if _pin and BLINK_ON_COMPLETE:
        GPIO.cleanup(_pin)
#   if driver:
#       driver.close()

#EOF
