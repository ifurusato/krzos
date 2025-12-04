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
import traceback
from datetime import datetime as dt
#import RPi.GPIO as GPIO
from colorama import init, Fore, Style
init()

import ioexpander as io

from hardware.vl53l0x import VL53L0X, Vl53l0xAccuracyMode
from core.component import Component
from core.illegal_state_error import IllegalStateError
from core.config_loader import ConfigLoader
from core.logger import Logger, Level

class ProximitySensor(Component):
    '''
    Encapsulates a single VL53L0X proximity sensor.
    Initializes the sensor with a configuration information.

    The previous implementation included use of the VL53L0X's XSHUT pin
    to enable/disable the sensor in order to change its I2C address.
    This has been removed from this class and the addesses are assumed
    to have been configured elsewhere.

    Args:

        cardinal (Cardinal):   The enumeration of the cardinal direction of this sensor.
        i2c_bus:               The I2C bus used for connection.
        i2c_address (int):     The I2C address used for this sensor.
        enabled (boolean):     Whether or not to activate this sensor.
        level (Level):         The logging level for this sensor's logger.
    '''
    def __init__(self, cardinal=None, i2c_bus=None, i2c_address=None, ioe=None, enabled=False, level=Level.INFO):
        self._cardinal    = cardinal
        self._id          = cardinal.id
        self._log = Logger('proximity-{}'.format(self._id), level=level)
        Component.__init__(self, self._log, suppressed=False, enabled=enabled)
        self._label       = cardinal.label
        self._abbrev      = cardinal.abbrev
        self._i2c_bus     = i2c_bus
        self._i2c_address = i2c_address
        self._ioe         = ioe
        self._is_ranging  = False
        self._active      = False # set True by connect()
        self._tof         = None
        self._change_state_delay_s = 1.0
        try:
            if self._ioe:
                self._log.debug('sensor {} ({}) ready using IO Expander.'.format(self._label, self._id))
            else:
                raise NotImplementedError('IO Expander required: GPIO implementation not supported.')
#               GPIO.setwarnings(False)
#               GPIO.setmode(GPIO.BCM)
#               self._log.debug('sensor {} ({}) ready using GPIO.'.format(self._label, self._id))
        except Exception as e:
            self._log.info('{} raised during pin setup: {}'.format(type(e).__name__, e))
            raise

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def id(self):
        '''
        Returns the sensor's unique identifier, an int between 0 and 7, which
        is also used as an index.
        '''
        return self._id

    @property
    def cardinal(self):
        '''
        Returns the sensor's cardinal value.
        '''
        return self._cardinal

    @property
    def abbrev(self):
        '''
        Returns the sensor's cardinal abbreviation, e.g., "NE".
        '''
        return self._abbrev

    @property
    def label(self):
        '''
        Returns the sensor's cardinal label, e.g., "north-east".
        '''
        return self._label

    @property
    def i2c_address(self):
        '''
        Returns the sensor's I2C address.
        '''
        return self._i2c_address

    @property
    def active(self):
        '''
        Returns whether the sensor is active (connected and ready to operate).
        '''
        return self._active

    @property
    def tof(self):
        '''
        Returns the VL53L0X object for the sensor.
        '''
        return self._tof

    def connect(self):
        '''
        Creates the VL53L0X sensor object, changing its I2C address.
        This sets the active flag to True if the sensor is enabled.

        Returns True if the sensor is active.
        '''
        if self._active:
            raise IllegalStateError('VL53L0X {} at 0x{:02X} is already active.'.format(self._label, self._i2c_address))
        try:
            self._log.debug('create VL53L0X {} at 0x{:02X}…'.format(self._label, self._i2c_address))
            self._tof = VL53L0X(i2c_bus=self._i2c_bus, i2c_address=self._i2c_address, label=self._label, accuracy=Vl53l0xAccuracyMode.HIGH_SPEED)
            self._active = True
            self._log.debug('sensor {} ready.'.format(self._label))
        except Exception as e:
            self._log.error('{} raised during setup of sensor {}: {}\n{}'.format(type(e).__name__, self._label, e, traceback.format_exc()))
            self.close()
        finally:
            self._log.debug('connect complete.')
        return self._active

    def get_distance(self):
        '''
        Retrieves the distance reading from the sensor.

        Returns:
            int: The distance in millimeters, or -1 if an error occurred.
        '''
        if not self.enabled:
            raise Exception('not enabled.')
        elif self._active:
            return self._tof.get_distance()
        else:
            raise Exception('cannot get distance: tof not active.')

    def is_ranging(self):
        return self._is_ranging

    def start_ranging(self):
        '''
        Starts the ranging process for the sensor. If the sensor is not already
        open, calls open() first.

        Args:
            mode: The optional accuracy mode for ranging.
        '''
        if self.enabled and self._active:
            if not self._is_ranging:
                if not self._tof.is_open():
                    self._tof.open()
                self._tof.start_ranging()
                self._is_ranging = True
                self._log.info('ranging started for sensor ' + Fore.GREEN + '{}'.format(self._label)
                        + Fore.CYAN + ' at 0x{:02X}…'.format(self._i2c_address))
            else:
                self._log.warning('sensor {} already ranging at 0x{:02X}…'.format(self._label, self._i2c_address))
        else:
            self._log.warning('cannot start ranging: sensor {} at 0x{:02X} not enabled or active.'.format(self._label, self._i2c_address))

    def stop_ranging(self):
        '''
        Stops ranging for the sensor.
        '''
        if self.enabled and self._active:
            self._log.info(Style.DIM + 'stop ranging sensor {} at 0x{:02X}…'.format(self._label, self._i2c_address))
            if not self._tof:
                raise Exception('cannot stop ranging: sensor {} at 0x{:02X} does not exist.'.format(self._label, self._i2c_address))
            if self._is_ranging:
                self._tof.stop_ranging()
                self._is_ranging = False
                self._log.info('ranging stopped on sensor {} at 0x{:02X}…'.format(self._label, self._i2c_address))
            else:
                self._log.warning('sensor {} was not ranging at 0x{:02X}…'.format(self._label, self._i2c_address))
        else:
            self._log.debug('cannot stop ranging: sensor {} at 0x{:02X} not enabled or active.'.format(self._label, self._i2c_address))

    def enable(self):
        '''
        Opens the sensor's ranging object.
        '''
        if not self.enabled:
            Component.enable(self)
#           super().enable()
            if self._tof:
                self._log.debug('opening sensor {} at 0x{:02X}…'.format(self._label, self._i2c_address))
                self._tof.open()
            else:
#               self._log.warning('cannot open: sensor {} at 0x{:02X} not available.'.format(self._label, self._i2c_address))
                raise Exception('cannot open: sensor {} at 0x{:02X} not available.'.format(self._label, self._i2c_address))
        else:
            self._log.warning('already enabled distance sensor.')

    def close(self):
        '''
        Stops ranging and shuts down the sensor.
        '''
        self._log.info('closing sensor {}…'.format(self._label))
        try:
            self.stop_ranging()
            if self._tof:
                self._tof.close()
        except Exception as e:
            self._log.error('{} raised closing the sensor {} at 0x{:02X}: {}'.format(type(e), self._label, self._i2c_address, e))
        finally:
#           Component.close(self)
            super().close()

    def __str__(self):
        return "Sensor(id={}, abbrev={}, i2c_address=0x{:02X}, enabled={})".format(
            self._id, self._label, self._i2c_address, self._enabled
        )

#EOF
