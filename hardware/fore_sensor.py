#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   altheim
# created:  2025-11-01
# modified: 2025-11-12

import numpy as np
from collections import deque
import ioexpander as io
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.convert import Convert
from hardware.i2c_scanner import I2CScanner

class ForeSensor(Component):
    NAME = 'fore-sensor'
    '''
    Wraps an IO Expander board as input for a single Pololu analog distance sensor.

    :param config:            the application configuration
    :param level:             the log level
    '''
    def __init__(self, config, level=Level.INFO):
        self._log = Logger(ForeSensor.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['kros'].get('hardware').get('fore_sensor')
        self._i2c_address  = _cfg.get('i2c_address')
        self._sensor_pin   = _cfg.get('sensor_pin')
        self._trim_cm      = _cfg.get('trim_cm')
        # smoothing configuration
        self._smoothing    = _cfg.get('smoothing', True)
        _smoothing_window  = _cfg.get('smoothing_window', 5)
        self._window       = deque(maxlen=_smoothing_window) if self._smoothing else None
        self._log.info('fore sensor pin assignment: {:d}; smoothing: {}; window: {}'.format(
            self._sensor_pin, self._smoothing, _smoothing_window if self._smoothing else 'N/A'))
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
                self._log.info('fore sensor ready on pin: ' + Fore.GREEN + '{:d}'.format(self._sensor_pin))
            else:
                raise Exception('no IO Expander found at address 0x{:02X}.'.format(self._i2c_address))
        except ImportError:
            raise Exception('This script requires the pimoroni-ioexpander module\nInstall with: pip3 install --user pimoroni-ioexpander')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def get_raw_value(self):
        '''
        Return the current (live) raw analog value from the sensor.
        '''
        if self._ioe is None:
            raise Exception('IO Expander not configured.')
        return self._ioe.input(self._sensor_pin)

    def get_value(self):
        '''
        Return the current (live) scaled analog value from the sensor (0-100).
        '''
        if self._ioe is None:
            raise Exception('IO Expander not configured.')
        return int(round(self._ioe.input(self._sensor_pin) * 100.0))

    def _smooth(self, value):
        '''
        Applies smoothing to the value if enabled.
        Returns smoothed value or original if smoothing disabled.
        '''
        if value is None or not self._smoothing:
            return value
        self._window.append(value)
        smoothed = np.mean(self._window)
        return smoothed

    def get_distance_cm(self):
        '''
        Return the current (live) distance in centimeters with optional smoothing,
        adding any configured trim value.
        Returns None if conversion fails.
        '''
        _value = self.get_value()
        _converted = Convert.convert_to_distance(_value)
        if _converted is None:
            return None
        _distance_cm = int(_converted) + self._trim_cm
        return int(self._smooth(_distance_cm)) if self._smoothing else _distance_cm

    def enable(self):
        '''
        Enables the sensor.
        '''
        if not self.enabled:
            super().enable()
            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def disable(self):
        '''
        Disables the sensor.
        '''
        if self.enabled:
            super().disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

    def close(self):
        '''
        Closes the sensor and frees resources.
        '''
        self.disable()
        if not self.closed:
            super().close()
            self._log.info('closed.')
        else:
            self._log.warning('already closed.')

#EOF
