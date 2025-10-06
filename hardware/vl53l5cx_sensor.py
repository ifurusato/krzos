#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-02
# modified: 2025-10-03

import time
from colorama import init, Fore, Style
init()

import vl53l5cx_ctypes as vl53l5cx
from vl53l5cx_ctypes import RANGING_MODE_CONTINUOUS

from core.component import Component
from core.logger import Logger, Level

class Vl53l5cxSensor(Component):
    NAME = 'vl53l5cx'
    '''
    Wrapper for VL53L5CX sensor.
    Handles sensor initialization, configuration, and data acquisition.
    Uses its own config section: "kros.hardware.vl53l5cx".
    Extends Component for lifecycle management.
    '''
    def __init__(self, config, skip=False, level=Level.INFO):
        self._log = Logger(Vl53l5cxSensor.NAME, level)
        Component.__init__(self, self._log, suppressed=True, enabled=False)
        self._log.info('initialising Vl53l5cxSensor…')
        # configuration
        _cfg = config['kros'].get('hardware').get('vl53l5cx')
        if _cfg is None or not isinstance(_cfg, dict):
            raise ValueError('invalid config: missing kros.hardware.vl53l5cx section')
        self.COLS = _cfg.get('cols', 8)
        self.ROWS = _cfg.get('rows', 8)
        self.FOV = _cfg.get('fov', 47.0)
        _i2c_bus_number = _cfg.get('i2c_bus_number')
        self._log.info('initialising VL53L5CX hardware {} on I2C bus {}…'.format(' (skip firmware upload)' if skip else '', _i2c_bus_number))
        if _i2c_bus_number == 0:
            try:
                self._log.info('connecting to I2C bus 0…')
                from smbus2 import SMBus
                _i2c_bus_dev = SMBus(0)
                # __init__(self, i2c_addr=DEFAULT_I2C_ADDRESS, i2c_dev=None, skip_init=False):
                self.vl53 = vl53l5cx.VL53L5CX(i2c_dev=_i2c_bus_dev, skip_init=skip)
            except Exception as e:
                self._log.error('{} raised connecting to I2C bus 0: {}'.format(type(e), e))
                raise e
        else:
            self.vl53 = vl53l5cx.VL53L5CX(skip_init=skip)
        self.vl53.set_resolution(self.COLS * self.ROWS)
        self.vl53.set_ranging_frequency_hz(_cfg.get('ranging_frequency_hz', 15))
        self.vl53.set_integration_time_ms(_cfg.get('integration_time_ms', 20))
        self.vl53.set_ranging_mode(RANGING_MODE_CONTINUOUS)
        self._log.info('VL53L5CX hardware ready.')

    def enable(self):
        if not self.enabled:
            self.vl53.start_ranging()
            super().enable()
            self._log.info('VL53L5CX hardware enabled and ranging.')
        else:
            self._log.info('VL53L5CX hardware already enabled.')

    def get_distance_mm(self):
        '''
        Wait for sensor data and return the grid as a flat list.
        Only returns real data if enabled.
        '''
        if not self.enabled:
            self._log.warning('get_distance_mm called while sensor is not enabled.')
            return None
        for _ in range(30):
            if self.vl53.data_ready():
                data = self.vl53.get_data()
                return data.distance_mm
            time.sleep(1 / 1000)
        return None

    def disable(self):
        if self.enabled:
            self.vl53.stop_ranging()
            super().disable()
            self._log.info('VL53L5CX hardware disabled and stopped ranging.')
        else:
            self._log.info('VL53L5CX hardware already disabled.')

    def close(self):
        if not self.closed:
            self.vl53.stop_ranging()
            super().close()
            self._log.info('VL53L5CX hardware closed.')
        else:
            self._log.info('VL53L5CX hardware already closed.')

#EOF
