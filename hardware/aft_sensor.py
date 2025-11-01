#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   altheim
# created:  2025-11-01
# modified: 2025-11-01

import ioexpander as io
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.convert import Convert
from hardware.i2c_scanner import I2CScanner

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class AftSensor(Component):
    '''
    Wraps an IO Expander board as input for a single Pololu analog distance sensor.
    
    :param config:            the application configuration
    :param level:             the log level
    '''
    def __init__(self, config, level=Level.INFO):
        self._log = Logger('aft-sensor', level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['kros'].get('hardware').get('aft_sensor')
        self._i2c_address = _cfg.get('i2c_address')
        self._sensor_pin  = _cfg.get('sensor_pin')
        self._log.info('aft sensor pin assignment: {:d}'.format(self._sensor_pin))
        self._ioe = None
        # confirm availability of IO Expander, set ADC reference and configure pin
        _component_registry = Component.get_registry()
        _i2c_scanner = _component_registry.get(I2CScanner.NAME)
        if not _i2c_scanner:
            _i2c_scanner = I2CScanner(config, level=level)
        try:
            if _i2c_scanner.has_address([self._i2c_address]):
                self._log.info('found IO Expander at address 0x{:02X}, configuring…'.format(self._i2c_address))
                self._ioe = io.IOE(i2c_addr=self._i2c_address)
                self._ioe.set_mode(self._sensor_pin, io.ADC)
                self._ioe.set_adc_vref(3.3)
                self._log.info('aft sensor ready on pin: ' + Fore.GREEN + '{:d}'.format(self._sensor_pin))
            else:
                raise Exception('no IO Expander found at address 0x{:02X}.'.format(self._i2c_address))
        except ImportError:
            raise Exception('This script requires the pimoroni-ioexpander module\nInstall with: pip3 install --user pimoroni-ioexpander')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_raw_value(self):
        '''
        Return the current (live) raw analog value from the sensor.
        '''
        if self._ioe is None:
            raise Exception('IO Expander not configured.')
        return self._ioe.input(self._sensor_pin)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_value(self):
        '''
        Return the current (live) scaled analog value from the sensor (0-100).
        '''
        if self._ioe is None:
            raise Exception('IO Expander not configured.')
        return int(round(self._ioe.input(self._sensor_pin) * 100.0))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_distance(self):
        '''
        Return the current (live) distance in centimeters.
        '''
        _value = self.get_value()
        return int(Convert.convert_to_distance(_value))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enables the sensor.
        '''
        if not self.enabled:
            Component.enable(self)
            self._log.info('aft sensor enabled.')
        else:
            self._log.info('already enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disables the sensor.
        '''
        if self.enabled:
            Component.disable(self)
            self._log.info('aft sensor disabled.')
        else:
            self._log.info('already disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Closes the sensor and frees resources.
        '''
        self.disable()
        if not self.closed:
            Component.close(self)
            self._log.info('aft sensor closed.')
        else:
            self._log.info('already closed.')

# EOF
