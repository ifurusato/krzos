#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-20
# modified: 2024-10-24
#

import time
import RPi.GPIO as GPIO
from colorama import init, Fore, Style
init()

from hardware.VL53L0X import VL53L0X, Vl53l0xAccuracyMode
from core.config_loader import ConfigLoader
from core.logger import Logger, Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class ProximitySensor(object):
    '''
    Encapsulates a single VL53L0X proximity sensor.
    '''
    def __init__(self, i2c_bus_number=1, sensor_id=None, label=None, i2c_address=None, xshut_pin=None, enabled=False, level=Level.INFO):
        '''
        Initializes the sensor object with configuration data.

        Args:
            sensor_id (str): The unique identifier for the sensor.
            i2c_address (int): The I2C address for the sensor.
            xshut_pin (int): The BCM GPIO pin number for the XSHUT control.
            enabled (bool): Whether the sensor is enabled in the configuration.
            level (Level): The logging level for this sensor's logger.
        '''
        self._log = Logger('proximity-{}'.format(sensor_id), level=level)
        self._id        = sensor_id
        self._label     = label
        self._i2c_bus_number = i2c_bus_number
        self._i2c_address  = i2c_address
        self._xshut_pin = xshut_pin
        self._tof       = None
        self._startup_delay_s = 1.0
        self._enabled   = False # set True by _setup()
        self._force_address_change = True
        # initial state of XSHUT pin is LOW (disabled)
        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._xshut_pin, GPIO.OUT)
            GPIO.output(self._xshut_pin, GPIO.LOW)
        except Exception as e:
            self._log.info('{} raised during pin setup: {}'.format(type(e).__name__, e))
            raise
        if enabled:
            self._enabled = self._setup()
        if self.enabled:
            self._log.info('{} ready.'.format(self._label))
        else:
            self._log.info(Style.DIM + '{} disabled.'.format(self._label))

    @property
    def id(self):
        '''
        Returns the sensor's unique identifier.
        '''
        return self._id

    @property
    def label(self):
        '''
        Returns the sensor's label.
        '''
        return self._label

    @property
    def i2c_address(self):
        '''
        Returns the sensor's I2C address.
        '''
        return self._i2c_address

    @property
    def enabled(self):
        '''
        Returns whether the sensor is enabled.
        '''
        return self._enabled

    @property
    def tof(self):
        '''
        Returns the VL53L0X object for the sensor.
        '''
        return self._tof

    def _setup(self):
        '''
        Creates and enables the VL53L0X sensor object, changing its I2C address.
        This sets the self._enabled flag.
        '''
        self._log.debug('setup sensor {} ({}) at 0x{:02X}…'.format(self._label, self._id, self._i2c_address))
        try:
            self._startup();
            self._tof = VL53L0X(i2c_bus=self._i2c_bus_number, i2c_address=0x29)
            self._log.debug('changing I2C address to 0x{:02X}…'.format(self._i2c_address))
            self._tof.change_address(self._i2c_address)
            self._log.debug('successfully changed I2C address to 0x{:02X}.'.format(self._i2c_address))
            return True
        except Exception as e:
            self._log.info('{} raised during setup: {}'.format(type(e).__name__, e))
            self.stop()
            return False

    def _startup(self):
        self._log.debug('enabling sensor {} ({}) on XSHUT pin {}.'.format(self._label, self._id, self._xshut_pin))
        GPIO.output(self._xshut_pin, GPIO.HIGH)
        time.sleep(self._startup_delay_s)

    def _shutdown(self):
        self._log.info('sensor {} ({}) shutting down.'.format(self._label, self._id))
        GPIO.output(self._xshut_pin, GPIO.LOW)

    def get_distance(self):
        '''
        Retrieves the distance reading from the sensor.

        Returns:
            int: The distance in millimeters, or -1 if an error occurred.
        '''
        return self._tof.get_distance()

    def start_ranging(self, mode=Vl53l0xAccuracyMode.BETTER):
        '''
        Starts the ranging process for the sensor.

        Args:
            mode: The accuracy mode for ranging.
        '''
        self._tof.start_ranging(mode)

    def open(self):
        '''
        Opens the sensor's ranging object.
        '''
        self._log.info(Fore.MAGENTA + 'opening sensor {} at 0x{:02X}…'.format(self._label, self._i2c_address))
        self._tof.open()

    def stop(self):
        '''
        Stops ranging and shuts down the sensor.
        '''
        self._log.info(Fore.MAGENTA + '⛔ shutting down sensor {} at 0x{:02X}…'.format(self._label, self._i2c_address))
        if self._tof:
            self._tof.stop_ranging()
            self._log.info(Fore.MAGENTA + '⛔ stop ranging called on sensor {} at 0x{:02X}…'.format(self._label, self._i2c_address))
            self._tof.close()
        self._shutdown()

    def __str__(self):
        return "Sensor(id={}, label={}, i2c_address=0x{:02X}, xshut_pin={}, enabled={})".format(
            self._id, self._label, self._i2c_address, self._xshut_pin, self._enabled
        )

#EOF
