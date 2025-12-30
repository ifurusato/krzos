#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-10-08
# modified: 2025-10-08

import sys
import time
import traceback
import ioexpander as io
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from hardware.i2c_scanner import I2CScanner

class RadiozoaConfig:
    '''
    Configures all VL53L0X sensors on the Radiozoa sensor board to their unique I2C
    addresses by toggling XSHUT pins and setting addresses as specified in the
    configuration.
    '''
    def __init__(self, config, level=Level.INFO):
        self._log = Logger('radiozoa-config', level=level)
        self._level = level
        self._config = config
        _cfg_radiozoa = config.get('kros').get('hardware').get('radiozoa')
        self._cfg_devices = _cfg_radiozoa.get('devices')
        self._i2c_bus_number = _cfg_radiozoa.get('i2c_bus_number')
        self._ioe_i2c_address = _cfg_radiozoa.get('ioe_i2c_address')
        self._default_i2c_address = 0x29
        self._sensor_count = 8
        self._i2c_bus = None
        self._ioe = None
        self._setup_i2c()
        self._setup_ioe()
        self._i2c_scanner = I2CScanner(config=self._config, i2c_bus_number=self._i2c_bus_number, i2c_bus=self._i2c_bus, level=Level.INFO)
        self._log.info('ready.')

    @property
    def ioe(self):
        '''
        Return the instance of the IOExpander.
        '''
        return self._ioe

    def _setup_i2c(self):
        import pkg_resources
        SMBUS = 'smbus2'
        for dist in pkg_resources.working_set:
            if dist.project_name == 'smbus':
                SMBUS = 'smbus'
                break
            if dist.project_name == 'smbus2':
                SMBUS = 'smbus2'
                break
        if SMBUS == 'smbus':
            import smbus
        elif SMBUS == 'smbus2':
            import smbus2 as smbus
        self._i2c_bus = smbus.SMBus()
        self._i2c_bus.open(bus=self._i2c_bus_number)
        self._log.info('I2C{} open.'.format(self._i2c_bus_number))

    def _setup_ioe(self):
        try:
            self._ioe = io.IOE(i2c_addr=self._ioe_i2c_address, smbus_id=self._i2c_bus_number)
            self._log.info('found IO Expander at 0x{:02X} on I2C{}.'.format(self._ioe_i2c_address, self._i2c_bus_number))
        except Exception as e:
            self._log.error('{} raised setting up IO Expander: {}'.format(type(e), e))
            raise

    def emergency_stop(self):
        '''
        Placeholder for compatibility. No operation for configuration.
        '''
        self._log.info('emergency_stop called (no operation).')

    def close(self):
        '''
        Close I2C bus and cleanup IOE if necessary.
        '''
        if self._i2c_bus:
            self._log.info('closing I2C bus…')
            self._i2c_bus.close()

    def _shutdown_all_sensors(self):
        '''
        Shuts down all sensors by setting their XSHUT pins LOW.
        '''
        for sensor_id, sensor_config in self._cfg_devices.items():
            label = sensor_config.get('label')
            xshut_pin = sensor_config.get('xshut')
            self._log.info("shutting down sensor {} at XSHUT pin {}…".format(label, xshut_pin))
            self._set_xshut(xshut_pin, False)
            time.sleep(0.05)

    def _configure_pins(self):
        for sensor_id, sensor_config in self._cfg_devices.items():
            label = sensor_config.get('label')
            xshut_pin = sensor_config.get('xshut')
            self._ioe.set_mode(xshut_pin, io.OUT)
            self._log.info("configured pin {} for sensor {} as output…".format(xshut_pin, label))

    def _configure_sensor_addresses(self):
        '''
        Sequentially brings up each sensor, sets its I2C address, leaving it enabled.
        '''
        needs_scan = True
        for sensor_id, sensor_config in self._cfg_devices.items():
            xshut_pin = sensor_config.get('xshut')
            i2c_address = sensor_config.get('i2c_address')
            label = sensor_config.get('label')
            self._log.info(Style.DIM + "configuring sensor {} at XSHUT pin {}…".format(label, xshut_pin))
            self._set_xshut(xshut_pin, True)
            found = False
            for i in range(5):
                time.sleep(0.75) # delay for sensor startup
                found = self._i2c_scanner.has_hex_address(['0x29'], force_scan=needs_scan)
                if found:
                    self._log.info(Style.DIM + "[{}] waiting for sensor…".format(i))
                    break
#           needs_scan = False
            if not found:
                self._log.warning("sensor {} did not appear at 0x29.".format(label))
                continue
            try:
                self._set_i2c_address(0x29, i2c_address)
                self._log.info("set address for sensor {} to 0x{:02X}".format(label, i2c_address))
            except Exception as e:
                self._log.error("{} raised setting address for sensor {}: {}".format(type(e), label, e))
            time.sleep(0.05) # brief delay after address set

    def configure(self):
        self._configure_pins()
        self._shutdown_all_sensors()
        self._configure_sensor_addresses()
        self._log.info('all sensor addresses configured.')

    def _set_xshut(self, pin, value):
        '''
        Set the XSHUT pin state using IO Expander.
        '''
        if self._ioe:
            self._ioe.set_pin(pin, value)
        else:
            raise RuntimeError('IOE not available for setting pin.')

    def _set_xshut(self, pin, value):
        '''
        Set the XSHUT pin state using IO Expander.
        '''
        if self._ioe:
            if value:
                self._ioe.output(pin, io.HIGH)
                self._log.debug(Style.DIM + "set pin {} HIGH.".format(pin))
            else:
                self._ioe.output(pin, io.LOW)
                self._log.debug(Style.DIM + "set pin {} LOW.".format(pin))
        else:
            raise RuntimeError('IOE not available for setting pin.')

    def _set_i2c_address(self, current_addr, new_addr):
        '''
        Change VL53L0X I2C address from current_addr to new_addr.
        Assumes sensor is up at current_addr.
        '''
        # VL53L0X: Write 0x8A register with new_addr
        # Note: new_addr should be 7 bits, same as config
        reg = 0x8A
        self._i2c_bus.write_byte_data(current_addr, reg, new_addr)
        time.sleep(0.05)

#EOF
