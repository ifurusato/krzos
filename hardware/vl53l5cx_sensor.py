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
import numpy as np
from colorama import init, Fore, Style
init()

#import vl53l5cx_ctypes as vl53l5cx
#from vl53l5cx_ctypes import RANGING_MODE_CONTINUOUS
import hardware.VL53L5CX as vl53l5cx
from hardware.VL53L5CX import RANGING_MODE_CONTINUOUS

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
        self._cols = _cfg.get('cols', 8)
        self._rows = _cfg.get('rows', 8)
        self._fov  = _cfg.get('fov', 47.0)
        self._floor_row_means   = [None for _ in range(self._rows)]
        self._floor_row_stddevs = [None for _ in range(self._rows)]
        self._non_floor_rows    = None
        self._calibration_samples = _cfg.get('calibration_samples', 10)
        self._stddev_threshold = _cfg.get('stddev_threshold', 100)
        self._floor_margin = _cfg.get('floor_margin', 50)
        _i2c_bus_number = _cfg.get('i2c_bus_number')
        self._minimum_free_distance = _cfg.get('minimum_free_distance', 500)
        self._log.info('initialising VL53L5CX hardware{} on I2C bus {}…'.format(' (skip firmware upload)' if skip else '', _i2c_bus_number))
        if _i2c_bus_number == 0:
            try:
                self._log.info('connecting to I2C bus 0…')
                from smbus2 import SMBus
                _i2c_bus_dev = SMBus(0)
                # __init__(self, i2c_addr=DEFAULT_I2C_ADDRESS, i2c_dev=None, skip_init=False):
                self._vl53 = vl53l5cx.VL53L5CX(i2c_dev=_i2c_bus_dev, skip_init=skip)
            except Exception as e:
                self._log.error('{} raised connecting to I2C bus 0: {}'.format(type(e), e))
                raise e
        else:
            self._vl53 = vl53l5cx.VL53L5CX(skip_init=skip)
        self._vl53.set_resolution(self._cols * self._rows)
        self._vl53.set_ranging_frequency_hz(_cfg.get('ranging_frequency_hz', 15))
        self._vl53.set_integration_time_ms(_cfg.get('integration_time_ms', 20))
        self._vl53.set_ranging_mode(RANGING_MODE_CONTINUOUS)
        self._log.info('VL53L5CX hardware ready.')

    @property
    def floor_margin(self):
        return self._floor_margin

    @property
    def floor_row_means(self):
        return self._floor_row_means

    @property
    def floor_row_stddevs(self):
        return self._floor_row_stddevs

    @property
    def non_floor_rows(self):
        if self._non_floor_rows is None:
            # find first two non-floor rows (immediately above the last floor row)
            floor_rows = [i for i, v in enumerate(self.floor_row_means) if v is not None]
            if not floor_rows:
                # if no floor rows, use rows 0 and 1
                non_floor_rows = [0, 1]
            else:
                last_floor_row = max(floor_rows)
                non_floor_rows = [r for r in range(last_floor_row + 1, self._rows)][:2]
                if len(non_floor_rows) < 2:
                    # not enough non-floor rows, pad with next available rows
                    non_floor_rows += [self._rows-1] * (2 - len(non_floor_rows))
            self._non_floor_rows = non_floor_rows
        return self._non_floor_rows

    def enable(self):
        if not self.enabled:
            self._vl53.start_ranging()
            super().enable()
            self._calibrate_floor_rows()
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
            if self._vl53.data_ready():
                data = self._vl53.get_data()
                return data.distance_mm
            time.sleep(1 / 1000)
        return None

    def _calibrate_floor_rows(self):
        self._log.info(Fore.WHITE + 'calibrating floor rows…')
        self._floor_row_means   = [None for _ in range(self._rows)]
        self._floor_row_stddevs = [None for _ in range(self._rows)]
        samples = []
        for i in range(self._calibration_samples):
            data = self.get_distance_mm() # returns an 8x8 array
            if data is None:
                self._log.debug("calibration sample {} is None.".format(i))
                continue
            arr = np.array(data).reshape((self._rows, self._cols))
            samples.append(arr)
            time.sleep(0.05)
        if len(samples) == 0:
            self._log.warning('no calibration samples collected, check sensor connection.')
            return
        samples = np.array(samples)  # shape: (samples, _rows, _cols)
        clear_distance = True
        for row in range(self._rows):
            values = samples[:, row, :].flatten()
            mean = values.mean()
            stddev = values.std()
            if mean < self._minimum_free_distance:
                # not enough free space in front of robot
                clear_distance = False
            else:
                self._log.info(Fore.WHITE + 'row {}: mean={:.1f}, stddev={:.1f}'.format(row, mean, stddev))
            if clear_distance and ( stddev < self._stddev_threshold ):
                self._floor_row_means[row] = mean
                self._floor_row_stddevs[row] = stddev
                self._log.info(Fore.WHITE + 'row {} marked as floor (mean={:.1f}, stddev={:.1f})'.format(row, mean, stddev))
            else:
                self._floor_row_means[row] = None
                self._floor_row_stddevs[row] = None
                self._log.info(Fore.WHITE + 'row {} NOT floor (mean={:.1f}, stddev={:.1f})'.format(row, mean, stddev))
                # all rows above this cannot be floor rows
                for above_row in range(row+1, self._rows):
                    self._floor_row_means[above_row] = None
                    self._floor_row_stddevs[above_row] = None
                break  # exit the calibration loop
        if clear_distance:
            detected_floor_rows = [i for i, v in enumerate(self._floor_row_means) if v is not None]
            self._log.info(Fore.WHITE + 'floor rows detected (indices): {}'.format(detected_floor_rows))
        if all(val is None for val in self._floor_row_means):
            # force bottom row as floor
            values = samples[:, 0, :].flatten()
            self._floor_row_means[0] = values.mean()
            self._floor_row_stddevs[0] = values.std()
            if clear_distance:
                self._log.warning('no floor detected during calibration, forcibly marking bottom row as floor.')
            else:
                # disable sensor or behaviour?
                self._log.error('could not calibrate: not enough clear space in front of robot, forcibly marking bottom row as floor.')

    def disable(self):
        if self.enabled:
            self._vl53.stop_ranging()
            super().disable()
            self._log.info('VL53L5CX hardware disabled and stopped ranging.')
        else:
            self._log.info('VL53L5CX hardware already disabled.')

    def close(self):
        if not self.closed:
            self._vl53.stop_ranging()
            self._vl53.close() # note: custom method in local copy
            super().close()
            self._log.info('VL53L5CX hardware closed.')
        else:
            self._log.info('VL53L5CX hardware already closed.')

#EOF
