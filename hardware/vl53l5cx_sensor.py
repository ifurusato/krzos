#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-02
# modified: 2025-10-13

import traceback
import time
import numpy as np
from datetime import datetime as dt
import multiprocessing
from multiprocessing import Process, Queue, Event
from colorama import init, Fore, Style
init()

import hardware.vl53l5cx as vl53l5cx
from hardware.vl53l5cx import RANGING_MODE_CONTINUOUS

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
        self._use_multiprocessing = True
        self._vl53_read_timeout_sec = 2.0 # seconds
        _i2c_bus_number = _cfg.get('i2c_bus_number')
        self._minimum_free_distance = _cfg.get('minimum_free_distance', 500)
        self._poll_interval = _cfg.get('poll_interval', 0.05) # new: default 50ms
        # multiprocessing attributes
        self._queue = Queue(maxsize=20)
        self._stop_event = Event()
        self._process = None
        self._log.info('initialising VL53L5CX hardware{} on I2C bus {}…'.format(' (skip firmware upload)' if skip else '', _i2c_bus_number))
        _start_time = dt.now()
        self._vl53 = None
        if _i2c_bus_number == 0:
            try:
                from smbus2 import SMBus
                _i2c_bus_dev = SMBus(0)
                # __init__(self, i2c_addr=DEFAULT_I2C_ADDRESS, i2c_dev=None, skip_init=False):
                self._vl53 = vl53l5cx.VL53L5CX(i2c_dev=_i2c_bus_dev, skip_init=skip)
            except Exception as e:
                self._log.error('{} raised connecting to VL53L5CX on I2C bus 0: {}'.format(type(e), e))
                raise e
        else:
            self._vl53 = vl53l5cx.VL53L5CX(skip_init=skip)
        self._vl53.set_resolution(self._cols * self._rows)
        self._vl53.set_ranging_frequency_hz(_cfg.get('ranging_frequency_hz', 10))
        self._vl53.set_integration_time_ms(_cfg.get('integration_time_ms', 20))
        self._vl53.set_ranging_mode(RANGING_MODE_CONTINUOUS)
        _elapsed_ms = round((dt.now() - _start_time).total_seconds() * 1000.0)
        self._log.info(Fore.MAGENTA + 'VL53L5CX device ready: elapsed: {:d}ms'.format(_elapsed_ms))

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


    def _polling_loop(self, queue, stop_event, poll_interval):
        '''
        Polls the VL53L5CX sensor as a separate process and puts results into the queue.
        This is only used when multiprocessing is active.
        '''
        try:
            while not stop_event.is_set():
                if self._vl53.data_ready():
                    data = self._vl53.get_data()
                    try:
                        # always convert to a list of ints using numpy, works for both bytes and array
                        _distance_mm = np.frombuffer(bytes(data.distance_mm), dtype=np.int16).tolist()
                        try:
                            queue.put(_distance_mm, block=False)
                        except Exception:
                            try:
                                queue.get_nowait()
                            except Exception:
                                pass
                            queue.put(_distance_mm, block=False)
                    except Exception as e:
                        self._log.error("{} raised converting distance_mm: {}".format(type(e), e))
                else:
                    self._log.info(Fore.MAGENTA + "data not ready.")
                time.sleep(poll_interval)
        except Exception as e:
            self._log.error("error raised polling sensor: {}".format(e))
        finally:
            self._vl53.stop_ranging()

    def enable(self):
        if not self.enabled:
            self._vl53.start_ranging()
            # wait for data to become available
            start = dt.now()
            while not self._vl53.data_ready() and (dt.now() - start).total_seconds() < self._vl53_read_timeout_sec:
                time.sleep(0.01)
            print('')
            super().enable()
            if self._use_multiprocessing:
                # start multiprocessing polling process
                if self._process is None or not self._process.is_alive():
                    self._log.debug('starting external process…')
                    self._stop_event.clear()
                    self._process = Process(
                        target=self._polling_loop,
                        args=(self._queue, self._stop_event, self._poll_interval),
                        daemon=True
                    )
                    self._process.start()
            else:
                pass

            self._calibrate_floor_rows()
            self._log.info('VL53L5CX hardware enabled and ranging.')
        else:
            self._log.info('VL53L5CX hardware already enabled.')

    def get_distance_mm(self):
        '''
        Returns the oldest sensor data from the process queue (non-blocking).
        '''
        if not self.enabled:
            self._log.warning('get_distance_mm called while sensor is not enabled.')
            return None
        if self._use_multiprocessing:
            try:
                value = self._queue.get_nowait()
                return value
            except Exception:
                return None
        else:
            start = dt.now()
            while not self._vl53.data_ready() and (dt.now() - start).total_seconds() < self._vl53_read_timeout_sec:
                time.sleep(0.01)
            try:
                data = self._vl53.get_data()
                return data.distance_mm
            except Exception as e:
                self._log.error("{} raised reading distance_mm: {}\n{}".format(type(e), e, traceback.format_exc()))
                return None

    def _calibrate_floor_rows(self):
        self._log.info(Fore.WHITE + 'calibrating floor rows using {} samples…'.format(self._calibration_samples))
        self._floor_row_means   = [None for _ in range(self._rows)]
        self._floor_row_stddevs = [None for _ in range(self._rows)]
        samples = []
        for i in range(self._calibration_samples):
            data = self.get_distance_mm() # returns an 8x8 array
            if data is None:
                self._log.info("calibration sample {} is None.".format(i))
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
        self._log.info(Fore.WHITE + 'floor rows calibrated.')

    def disable(self):
        if self.enabled:
            # Stop the process if running
            if self._process is not None and self._process.is_alive():
                self._stop_event.set()
                self._process.join(timeout=2)
                if self._process.is_alive():
                    self._process.terminate()
                self._process = None
            self._vl53.stop_ranging()
            super().disable()
            self._log.info('VL53L5CX hardware disabled and stopped ranging.')
        else:
            self._log.info('VL53L5CX hardware already disabled.')

    def close(self):
        if not self.closed:
            self.disable()
            if self._vl53:
                self._vl53.close() # note: custom method in local copy
            super().close()
            self._log.info('VL53L5CX hardware closed.')
        else:
            self._log.info('VL53L5CX hardware already closed.')

#EOF
