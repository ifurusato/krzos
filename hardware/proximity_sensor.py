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

import ioexpander as io

from hardware.VL53L0X import VL53L0X, Vl53l0xAccuracyMode
from core.config_loader import ConfigLoader
from core.logger import Logger, Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class ProximitySensor(object):
    '''
    Encapsulates a single VL53L0X proximity sensor.
    Initializes the sensor with a configuration information.

    If an IO Expander is supplied it is used to implement the XSHUT
    functionality, otherwise it will be implemented using GPIO pins.

    Args:

        cardinal (Cardinal):   The enumeration of the cardinal direction of this sensor.
        i2c_bus:               The I2C bus used for connection.
        i2c_address (int):     The I2C address used for this sensor.
        xshut_pin (int):       The pin (GPIO or IO Expander) used to shut down the sensor.
        enabled (boolean):     Whether or not to activate this sensor.
        level (Level):         The logging level for this sensor's logger.
    '''
    def __init__(self, cardinal=None, i2c_bus=None, i2c_address=None, xshut_pin=None, ioe=None, enabled=False, level=Level.INFO):
        self._cardinal    = cardinal
        self._id          = cardinal.id
        self._log = Logger('proximity-{}'.format(self._id), level=level)
        self._label       = cardinal.label 
        self._abbrev      = cardinal.abbrev
        self._i2c_bus     = i2c_bus 
        self._i2c_address = i2c_address
        self._xshut_pin   = xshut_pin 
        self._ioe         = ioe
        self._enabled     = enabled
        self._is_ranging  = False
        self._active      = False # set True by connect()
        # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._tof         = None
        self._change_state_delay_s = 1.0
        try:
            if self._ioe:
                self._ioe.set_mode(self._xshut_pin, io.OUT)
                self._log.info('sensor {} ({}) ready using IOE.'.format(self._abbrev, self._id))
            else:
                GPIO.setwarnings(False)
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self._xshut_pin, GPIO.OUT)
                self._log.info('sensor {} ({}) ready using GPIO.'.format(self._abbrev, self._id))
        except Exception as e:
            self._log.info('{} raised during pin setup: {}'.format(type(e).__name__, e))
            raise

    # properties ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def id(self):
        '''
        Returns the sensor's unique identifier, an int between 0 and 7, which
        is also used as an index.
        '''
        return self._id

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
    def xshut_pin(self):
        '''
        Returns the sensor's configured XSHUT pin.
        '''
        return self._xshut_pin

    @property
    def enabled(self):
        '''
        Returns whether the sensor is enabled.
        '''
        return self._enabled

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

    # connection ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def connect(self, i2c_scanner):
        '''
        Creates the VL53L0X sensor object, changing its I2C address.
        This sets the active flag to True if the sensor is enabled.

        Returns True if the sensor is active.
        '''
        self._log.debug('setup sensor {} ({}) at 0x{:02X}…'.format(self._abbrev, self._id, self._i2c_address))
        try:
            self.startup();
            self._tof = None
            if i2c_scanner.has_hex_address([self.i2c_address]):
                # the sensor can already be seen at the desired I2C address, so create it at that address
                self._log.info(Fore.GREEN + 'create VL53L0X {} at 0x{:02X}…'.format(self._abbrev, self._i2c_address))
                self._tof = VL53L0X(i2c_bus=self._i2c_bus, i2c_address=self.i2c_address)
                time.sleep(0.2)
            else:
                # otherwise instantiate at 0x29 and change the address.
                self._log.info(Fore.MAGENTA + 'create VL53L0X {} at default address 0x29…'.format(self._abbrev))
                self._tof = VL53L0X(i2c_bus=self._i2c_bus, i2c_address=0x29)
                self._log.info(Fore.MAGENTA + 'changing I2C address to 0x{:02X}…'.format(self._i2c_address))
                self._tof.change_address(self._i2c_address)
                self._log.info(Fore.MAGENTA + 'successfully changed I2C address to 0x{:02X}.'.format(self._i2c_address))
                time.sleep(1)
            if i2c_scanner.has_hex_address([self.i2c_address], force_scan=True):
                self._log.info(Fore.WHITE + Style.BRIGHT + "HAS VL53L0X {} at default address 0x29…".format(self._abbrev))
                if self._enabled:
                    self._active = True
            else:
                self._log.info(Fore.WHITE + Style.BRIGHT + "DOESN'T HAVE VL53L0X {} at default address 0x29…".format(self._abbrev))
                self._enabled = False # then just don't use it
            self._log.info('sensor {} ready.'.format(self._abbrev))
        except Exception as e:
            self._log.info('{} raised during setup of sensor {}: {}'.format(type(e).__name__, self._abbrev, e))
            self.stop_ranging()
            self.shutdown();
            self._log.info(Style.DIM + '{} disabled.'.format(self._abbrev))
        return self._active

    def startup(self):
        self._log.info(Fore.MAGENTA + 'enabling sensor {} ({}) on XSHUT pin {}…'.format(self._abbrev, self._id, self._xshut_pin))
        if self._ioe:
            self._log.info(Fore.MAGENTA + '😭 sensor {} ({}) STARTUP on XSHUT pin {}.'.format(self._abbrev, self._id, self._xshut_pin))
            self._ioe.output(self._xshut_pin, io.HIGH)
        else:
            GPIO.output(self._xshut_pin, GPIO.HIGH)
        time.sleep(self._change_state_delay_s)
        self._log.info(Fore.MAGENTA + 'sensor {} ({}) enabled on XSHUT pin {}.'.format(self._abbrev, self._id, self._xshut_pin))

    def shutdown(self):
        self._log.info(Fore.MAGENTA + 'shutting down sensor {} ({})…'.format(self._abbrev, self._id))
        if self._ioe:
            self._log.info(Fore.MAGENTA + '😭 sensor {} ({}) SHUTDOWN on XSHUT pin {}.'.format(self._abbrev, self._id, self._xshut_pin))
            self._ioe.output(self._xshut_pin, io.LOW)
        else:
            GPIO.output(self._xshut_pin, GPIO.LOW)
#       time.sleep(self._change_state_delay_s)
        self._log.info(Fore.MAGENTA + 'sensor {} ({}) shut down.'.format(self._abbrev, self._id))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def get_distance(self):
        '''
        Retrieves the distance reading from the sensor.

        Returns:
            int: The distance in millimeters, or -1 if an error occurred.
        '''
        if self._active:
            return self._tof.get_distance()
        raise Exception('cannot get distance: tof not active.')

    def start_ranging(self, mode=Vl53l0xAccuracyMode.BETTER):
        '''
        Starts the ranging process for the sensor.

        Args:
            mode: The optional accuracy mode for ranging.
        '''
        if self._enabled and self._active:
            if not self._is_ranging:
                self._tof.start_ranging(mode)
                self._is_ranging = True
            else:
                self._log.warning('sensor {} already ranging at 0x{:02X}…'.format(self._abbrev, self._i2c_address))
        else:
            self._log.warning('cannot start ranging: sensor {} at 0x{:02X} not enabled or active.'.format(self._abbrev, self._i2c_address))

    def stop_ranging(self):
        '''
        Stops ranging for the sensor.
        '''
        if self._enabled and self._active:
            self._log.info(Fore.MAGENTA + 'stop ranging sensor {} at 0x{:02X}…'.format(self._abbrev, self._i2c_address))
            if not self._tof:
                raise Exception('cannot stop ranging: sensor {} at 0x{:02X} does not exist.'.format(self._abbrev, self._i2c_address))
            if self._is_ranging:
                self._tof.stop_ranging()
                self._is_ranging = False
                self._log.info(Fore.MAGENTA + 'ranging stopped on sensor {} at 0x{:02X}…'.format(self._abbrev, self._i2c_address))
            else:
                self._log.warning('sensor {} was not ranging at 0x{:02X}…'.format(self._abbrev, self._i2c_address))
        else:
            self._log.debug('cannot stop ranging: sensor {} at 0x{:02X} not enabled or active.'.format(self._abbrev, self._i2c_address))

    def open(self):
        '''
        Opens the sensor's ranging object.
        '''
        if self._tof:
            self._log.info(Fore.MAGENTA + 'opening sensor {} at 0x{:02X}…'.format(self._abbrev, self._i2c_address))
            _ = self._tof.open()
        else:
            self._log.warning('cannot open: sensor {} at 0x{:02X} not available.'.format(self._abbrev, self._i2c_address))

    def close(self):
        '''
        Stops ranging and shuts down the sensor.
        '''
        self._log.info(Fore.MAGENTA + 'closing sensor {}…'.format(self._abbrev))
        try:
            self.stop()
            if self._tof:
                self._tof.close()
            self._enabled = False
        except Exception as e:
            self._log.error('{} raised closing the sensor {} at 0x{:02X}: {}'.format(type(e), self._abbrev, self._i2c_address, e))
            self.shutdown()

    def __str__(self):
        return "Sensor(id={}, abbrev={}, i2c_address=0x{:02X}, xshut_pin={}, enabled={})".format(
            self._id, self._abbrev, self._i2c_address, self._xshut_pin, self._enabled
        )

#EOF
