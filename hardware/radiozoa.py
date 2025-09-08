#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-09-08
# modified: 2025-09-08
#

import time
import RPi.GPIO as GPIO
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from hardware.i2c_scanner import I2CScanner
from hardware.proximity_sensor import ProximitySensor

class Radiozoa(object):
    '''
    Manages an array of VL53L0X proximity sensors as implemented on the Radiozoa board.
    '''
    def __init__(self, config, level=Level.INFO):
        '''
        Initializes the sensor array, setting up sensors.

        Args:
            config (dict): The configuration dictionary for the sensors.
            level (Level): The logging level.
        '''
        self._log = Logger('proximity', level=level)
        self._config = config
        self._cfg_vl53l0x = config.get('kros').get('hardware').get('vl53l0x')
        self._i2c_bus_number = self._cfg_vl53l0x.get('i2c_bus_number')
        self._cfg_devices = self._cfg_vl53l0x.get('devices')
        self._active_sensors = []
        self._enabled = False
        self._disable_delay_s = 0.5
        self._log.info('ready.')

    def enable(self):
        if self._enabled:
            self._log.warning('already enabled.')
        else:
            self.disable_all_sensors()
            self._enabled = self._setup()

    def disable_all_sensors(self):
        """
        Sets all XSHUT pins low to disable all VL53L0X devices.
        """
        self._log.info('Disabling all VL53L0X sensors (setting XSHUT pins LOW)...')
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        # iterate through all devices in config, regardless of 'enabled'
        _i2c_addresses = {}
        for sensor_config in self._cfg_devices.values():
            label = sensor_config.get('label')
            xshut_pin = sensor_config.get('xshut')
            i2c_address = sensor_config.get('i2c_address')
            _i2c_addresses[i2c_address] = xshut_pin
            if xshut_pin is None:
                raise Exception('no XSHUT pin configured for sensor {}.'.format(label))
            GPIO.setup(xshut_pin, GPIO.OUT)
            GPIO.output(xshut_pin, GPIO.LOW)
            self._log.info(Style.DIM + "disable sensor {} via XSHUT pin {}".format(label, xshut_pin))
        time.sleep(1.0) # wait to ensure sensors shut down
        self._confirm_clean_bus(_i2c_addresses)

    def _confirm_clean_bus(self, i2c_addresses):
        _i2c_scanner = I2CScanner(self._config, bus_number=self._i2c_bus_number, level=Level.INFO)
        for i2c_address, xshut_pin in i2c_addresses.items():
            _address_str = '0x{:02X}'.format(i2c_address)
            self._log.info('checking device at {}…'.format(_address_str))
            if _i2c_scanner.has_hex_address([_address_str]):
                self._log.warning('sensor at {} still online.'.format(_address_str))
                self._reset_xshut_pin(xshut_pin, i2c_address)
            time.sleep(0.5)
            if _i2c_scanner.has_hex_address([_address_str]):
                self._log.warning('device remains at {}.'.format(_address_str))
            else:
                self._log.info('device clean at {}.'.format(_address_str))
        self._log.info('all VL53L0X sensors disabled.')

    def _reset_xshut_pin(self, xshut_pin, i2c_address):
        try:
            self._log.info(Fore.WHITE + Style.BRIGHT + 'resetting xshut pin {} for device at 0x{:02X}.'.format(xshut_pin, i2c_address))
            GPIO.setup(xshut_pin, GPIO.OUT)
            GPIO.output(xshut_pin, GPIO.LOW)
            time.sleep(0.5)
        except Exception as e:
            self._log.error('{} raised resetting xshut pin: {}'.format(type(e).__name__, e))

    def _setup(self):
        '''
        Creates and enables each sensor sequentially using the ProximitySensor class.

        Returns:
            bool: True if setup was successful for all enabled sensors, False otherwise.
        '''
        self._log.info(Fore.WHITE + 'setting up sensors…')
        try:
            for _sensor_id, sensor_config in self._cfg_devices.items():
                _label       = sensor_config.get('label')
                _i2c_address = sensor_config.get('i2c_address')
                _xshut       = sensor_config.get('xshut')
                _enabled     = sensor_config.get('enabled')
                # the sensor instantiates as shut down, changes its I2C address and then becomes available if enabled
                sensor = ProximitySensor(
                        i2c_bus_number=self._i2c_bus_number,
                        sensor_id=_sensor_id,
                        label=_label,
                        i2c_address=_i2c_address,
                        xshut_pin=_xshut,
                        enabled=_enabled
                )
                if _enabled:
                    self._log.info('created sensor {} ({}) at I2C address 0x{:02X} with XSHUT pin {}…'.format(
                            _label, _sensor_id, _i2c_address, _xshut))
                    self._active_sensors.append(sensor)
                else:
                    self._log.info(Style.DIM + 'created disabled sensor {} ({}) at I2C address 0x{:02X} with XSHUT pin {}…'.format(
                            _label, _sensor_id, _i2c_address, _xshut))
            self._log.info('sensors ready.')
#           self.print_configuration()
            return True
        except Exception as e:
            self._log.error('{} raised setting up sensors: {}'.format(type(e).__name__, e))
            self.stop_ranging()
            return False

    def print_configuration(self):
        for sensor_id, sensor_config in self._cfg_devices.items():
            label = sensor_config.get('label')
            if sensor_config.get('enabled'):
                self._log.info("sensor {} ({}) enabled.".format(label, sensor_id))
            else:
                self._log.info("sensor {} ({}) disabled.".format(label, sensor_id))

    @property
    def enabled(self):
        '''
        Returns True if the sensor array was successfully enabled.
        '''
        return self._enabled

    def start_ranging(self):
        '''
        Starts the ranging process for all active sensors.
        '''
        self._log.info(Fore.GREEN + 'start ranging…')
        if self._active_sensors:
            for sensor in self._active_sensors:
                self._log.debug('opening sensor {}…'.format(sensor.id))
                sensor.open()
            for sensor in self._active_sensors:
                self._log.debug('start ranging sensor {}…'.format(sensor.id))
                sensor.start_ranging()

    def get_all_distances(self):
        '''
        Retrieves the distance from all active sensors.

        Returns:
            list: A list of dictionaries with sensor_id and distance.
        '''
#       self._log.debug('get all distances from {} active sensors…'.format(len(self._active_sensors)))
        distances = []
        for sensor in self._active_sensors:
            distances.append({
                'id': sensor.id,
                'label': sensor.label,
                'distance': sensor.get_distance()
            })
        return distances

    def _stop_ranging(self):
        '''
        Stops ranging and shuts down all active sensors.
        '''
        self._log.info('⛔ stop ranging…')
        for sensor in self._active_sensors:
            sensor.stop()

    def close(self):
        self._log.info('closing…')
        self._stop_ranging()
        self.disable_all_sensors()
        self._log.info('closed.')

#EOF
