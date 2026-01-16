#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-10-02
# modified: 2025-11-13

import traceback
import time
import numpy as np
from datetime import datetime as dt
#import multiprocessing
from multiprocessing import Process, Queue
from multiprocessing import Event as MultiProcessingEvent
from colorama import init, Fore, Style
init()

import hardware.vl53l5cx as vl53l5cx
from hardware.vl53l5cx import RANGING_MODE_CONTINUOUS

from core.component import Component
from core.event import Event
from core.logger import Logger, Level
from core.boot_session_marker import BootSessionMarker
from core.message_factory import MessageFactory
from core.queue_publisher import QueuePublisher

class Vl53l5cxSensor(Component):
    NAME = 'vl53l5cx-sensor'
    '''
    Wrapper for VL53L5CX sensor, handling sensor initialization, configuration,
    and data acquisition. Includes real-time spatial filtering to correct
    unreliable pixel data and publishes a BLIND event if data quality degrades
    past a configurable threshold over a sustained period.

    Data orientation: Row 0 = bottom/floor, Row 7 = top/far

    Args:
        config:    application configuration
        skip:      True = skip firmware load; False = force firmware load; None = use uptime marker
        level:     loggin level
    '''
    def __init__(self, config, skip=None, level=Level.INFO):
        self._log = Logger(Vl53l5cxSensor.NAME, level)
        Component.__init__(self, self._log, suppressed=True, enabled=False)
        self._log.info('initialising Vl53l5cxSensor…')
        self._firmware_marker = BootSessionMarker('vl53l5cx_firmware', level=level)
        # check if we should skip firmware loading
        if skip is False:  # explicitly force load
            self._log.info('force loading firmware (skip=False)')
            _skip_init = False
            self._firmware_marker.mark()
        elif skip is True:  # Explicitly skip
            self._log.info('skipping firmware load (skip=True)')
            _skip_init = True
        else:  # skip is None - use marker auto-detection
            self._log.info('firmware load based on marker (skip=None)')
            if self._firmware_marker.is_marked():
                _skip_init = True
                self._log.info('skipping firmware load (already loaded)')
            else:
                self._log.info('scheduled firmware loading to VL53L5CX.')
                _skip_init = False
                self._firmware_marker.mark()
        # configuration
        _cfg = config['kros'].get('hardware').get('vl53l5cx_sensor')
        if _cfg is None or not isinstance(_cfg, dict):
            raise ValueError('invalid config: missing kros.hardware.vl53l5cx section')
        self._cols = _cfg.get('cols', 8)
        self._rows = _cfg.get('rows', 8)
        self._fov  = _cfg.get('fov', 47.0)
        self._floor_row_means   = [None for _ in range(self._rows)]
        self._floor_row_stddevs = [None for _ in range(self._rows)]
        self._calibration_samples = _cfg.get('calibration_samples', 10)
        self._stddev_threshold = _cfg.get('stddev_threshold', 100)
        self._floor_margin = _cfg.get('floor_margin', 50)
        self._vl53_read_timeout_sec = 2.0 # seconds
        _i2c_bus_number = _cfg.get('i2c_bus_number')
        self._minimum_free_distance = _cfg.get('minimum_free_distance', 500)
        self._poll_interval = _cfg.get('poll_interval', 0.05) # new: default 50ms
        self._unreliable_pixel_threshold = _cfg.get('unreliable_pixel_threshold', 0.25)
        self._unreliable_frame_threshold = _cfg.get('unreliable_frame_threshold', 20)
        self._log.info("unreliable pixel threshold: {:.2%}; frame threshold: {}".format(self._unreliable_pixel_threshold, self._unreliable_frame_threshold))
        # flip orientation options
        self._flip_horizontal = _cfg.get('flip_horizontal', False)
        self._flip_vertical = _cfg.get('flip_vertical', False)
        self._log.info('sensor orientation: flip_horizontal={}, flip_vertical={}'.format(
            self._flip_horizontal, self._flip_vertical))
        # stuck pixel filter
        self._enable_stuck_pixel_filter = _cfg.get('enable_stuck_pixel_filter', False)
        self._stuck_pixel_filter = None
        if self._enable_stuck_pixel_filter:
            from hardware.stuck_pixel_filter import StuckPixelFilter
            self._stuck_pixel_filter = StuckPixelFilter(config, level=level)
            self._log.info('stuck pixel filter enabled.')   
        # messaging and state
        self._message_factory = None # lazy loaded
        self._queue_publisher = None # lazy loaded
        self._unreliable_frame_counter = 0
        self._is_blind = False
        # multiprocessing attributes
        self._use_multiprocessing = _cfg.get('use_multiprocessing', True)
        self._last_distance = None       # cache for last known reading
        self._last_distance_time = None  # timestamp of last reading
        self._stale_data_timeout_sec = _cfg.get('stale_data_timeout_sec', 0.5)
        _queue_maxsize = 50 # was 20 config?
        self._queue = Queue(maxsize=_queue_maxsize)
        self._stop_event = MultiProcessingEvent()
        self._process = None
        self._log.info('initialising VL53L5CX hardware{} on I2C bus {}…'.format(' (skip firmware upload)' if _skip_init else '', _i2c_bus_number))
        _start_time = dt.now()
        self._vl53 = None
        if _i2c_bus_number == 0:
            try:
                from smbus2 import SMBus
                _i2c_bus_dev = SMBus(0)
                # __init__(self, i2c_addr=DEFAULT_I2C_ADDRESS, i2c_dev=None, skip_init=False):
                self._vl53 = vl53l5cx.VL53L5CX(i2c_dev=_i2c_bus_dev, skip_init=_skip_init)
            except Exception as e:
                self._log.error('{} raised connecting to VL53L5CX on I2C bus 0: {}'.format(type(e), e))
                raise
        else:
            self._vl53 = vl53l5cx.VL53L5CX(skip_init=_skip_init)
        self._vl53.set_resolution(self._cols * self._rows)
        self._vl53.set_ranging_frequency_hz(_cfg.get('ranging_frequency_hz', 10))
        self._vl53.set_integration_time_ms(_cfg.get('integration_time_ms', 20))
        self._vl53.set_ranging_mode(RANGING_MODE_CONTINUOUS)
        _elapsed_ms = round((dt.now() - _start_time).total_seconds() * 1000.0)
        self._log.info('VL53L5CX device ready: elapsed: {:d}ms'.format(_elapsed_ms))

    def _get_queue_publisher(self):
        '''Lazy load the QueuePublisher from the component registry.'''
        if self._queue_publisher is None:
            self._queue_publisher = Component.get_registry().get(QueuePublisher.NAME)
            if self._queue_publisher is None:
                self._log.warning('QueuePublisher not found in registry. BLIND events will not be published.')
        return self._queue_publisher

    def _get_message_factory(self):
        '''Lazy load the MessageFactory from the component registry.'''
        if self._message_factory is None:
            self._message_factory = Component.get_registry().get(MessageFactory.NAME)
            if self._message_factory is None:
                self._log.warning('MessageFactory not found in registry. BLIND events will not be published.')
        return self._message_factory

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
    def floor_rows(self):
        '''
        Returns list of floor row indices (dynamically computed from calibration).
        Row 0 = bottom/floor, Row 7 = top/far.
        If rows 0,1 are floor, returns [0, 1].
        '''
        floor_rows = [i for i, v in enumerate(self.floor_row_means) if v is not None]
        return floor_rows

    @property
    def non_floor_rows(self):
        '''
        Returns list of non-floor row indices (dynamically computed from calibration).
        Row 0 = bottom/floor, Row 7 = top/far.
        If rows 0,1 are floor, returns [2, 3, 4, 5, 6, 7].
        '''
        floor_rows = [i for i, v in enumerate(self.floor_row_means) if v is not None]
        if not floor_rows:
            # if no floor rows detected, all rows are obstacle rows
            return list(range(self._rows))
        else:
            last_floor_row = max(floor_rows)  # highest index floor row
            non_floor_rows = list(range(last_floor_row + 1, self._rows))  # all rows above floor
            if len(non_floor_rows) == 0:
                # all rows marked as floor (shouldn't happen, but fallback to top row)
                return [self._rows - 1]
            return non_floor_rows

    def _polling_loop(self, queue, stop_event, poll_interval):
        '''
        Polls the VL53L5CX sensor as a separate process and puts results into the queue.
        This is only used when multiprocessing is active.
        '''
        try:
            self._vl53.start_ranging() 
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
                    self._log.info(Style.DIM + "data not ready.")
                time.sleep(poll_interval)
        except KeyboardInterrupt:
            print('')
            self._log.info('Ctrl-C caught: exiting…')
        except Exception as e:
            self._log.error('error raised polling sensor: {}'.format(e))
        finally:
            try:
                self._vl53.stop_ranging()
            except Exception as e:
                self._log.error("error raised stopping ranging: {}".format(e))

    def _apply_orientation(self, arr):
        '''
        Apply horizontal and/or vertical flipping to sensor data array.
        
        Args:
            arr: numpy array of shape (rows, cols)
            
        Returns:
            numpy array with requested flips applied
        '''
        if self._flip_horizontal:
            arr = np.fliplr(arr)  # flip left-right
        if self._flip_vertical:
            arr = np.flipud(arr)  # flip upside-down
        return arr

    def enable(self):
        if not self.enabled:
            if not self._use_multiprocessing:
                # synchronous mode - start ranging in main process
                self._vl53.start_ranging()
                start = dt.now()
                while not self._vl53.data_ready() and (dt.now() - start).total_seconds() < self._vl53_read_timeout_sec:
                    time.sleep(0.01)
#           super().enable()
            Component.enable(self)
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
                    self._log.info('waiting for multiprocessing queue to populate…')
                    time.sleep(1.0)
            self._calibrate_floor_rows()
            self._log.info('VL53L5CX hardware enabled and ranging.')

    def _process_and_health_check(self, arr):
        '''
        Applies filters, checks sensor data health, and manages BLIND state.

        Returns:
            The cleaned/filtered numpy array.
        '''
        corrected_count = 0
        if self._stuck_pixel_filter:
            # Step 1: Interpolate sporadic zeros
            arr, corrected_count = self._stuck_pixel_filter.interpolate_zeros(arr)
            # Step 2: Update long-term stuck pixel detection
            self._stuck_pixel_filter.update(arr)
            # Step 3: Filter out any long-term stuck pixels
            arr = self._apply_stuck_pixel_filter(arr)

        # Health Check based on corrected pixels
        total_pixels = self._rows * self._cols
        unreliable_ratio = corrected_count / total_pixels
        
        if unreliable_ratio > self._unreliable_pixel_threshold:
            self._unreliable_frame_counter += 1
            self._log.warning("unreliable frame detected ({} corrected pixels); unreliable count: {}".format(
                    corrected_count, self._unreliable_frame_counter))
        else:
            # Frame is reliable, reset counter and check for recovery
            if self._is_blind:
                self._log.info("sensor has recovered from BLIND state.")
                queue_publisher = self._get_queue_publisher()
                message_factory = self._get_message_factory()
                if queue_publisher and message_factory:
                    message = message_factory.create_message(Event.BLIND, False)
                    queue_publisher.put(message)
            self._unreliable_frame_counter = 0
            self._is_blind = False

        # If counter exceeds threshold, enter BLIND state
        if self._unreliable_frame_counter >= self._unreliable_frame_threshold:
            if not self._is_blind:
                self._is_blind = True
                self._log.error("sensor is BLIND: unreliable for {} frames, exceeding threshold of {}.".format(
                        self._unreliable_frame_counter, self._unreliable_frame_threshold))
                queue_publisher = self._get_queue_publisher()
                message_factory = self._get_message_factory()
                if queue_publisher and message_factory:
                    message = message_factory.create_message(Event.BLIND, True)
                    queue_publisher.put(message)
        
        return arr

    def _apply_stuck_pixel_filter(self, arr):
        '''
        Replace long-term stuck pixel values with interpolated values from neighboring pixels.
        
        Args:
            arr: numpy array of shape (rows, cols)
            
        Returns:
            numpy array with stuck pixels replaced
        '''
        # This method is now specifically for long-term stuck pixels
        if not self._stuck_pixel_filter:
            return arr
            
        # replace stuck pixels with interpolated values
        for row, col in self._stuck_pixel_filter.stuck_pixels:
            neighbors = []
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = row + dr, col + dc
                    if 0 <= nr < self._rows and 0 <= nc < self._cols:
                        if not self._stuck_pixel_filter.is_stuck(nr, nc):
                            neighbors.append(arr[nr, nc])
            if neighbors:
                arr[row, col] = int(np.mean(neighbors))
            else:
                arr[row, col] = 0 # should be rare, as it's already been interpolated
        return arr

    def get_distance_mm(self):
        '''
        Returns sensor data as a flat list, with rows properly oriented.
        Data is filtered for noise and checked for reliability. Returns None if
        the sensor is in a BLIND state.

        When using multiprocessing, returns the last known value if the queue is empty,
        but only if the data is not too stale (within stale_data_timeout_sec).
        '''
        if not self.enabled:
            self._log.warning('get_distance_mm called while sensor is not enabled.')
            return None

        raw_value = None
        if self._use_multiprocessing:
            try:
                raw_value = self._queue.get_nowait()
            except Exception:
                # Queue empty, fall through to use cached value if not stale
                pass
        else: # Synchronous mode
            start = dt.now()
            while not self._vl53.data_ready() and (dt.now() - start).total_seconds() < self._vl53_read_timeout_sec:
                time.sleep(0.01)
            try:
                data = self._vl53.get_data()
                raw_value = data.distance_mm
            except Exception as e:
                self._log.error("{} raised reading distance_mm: {}\n{}".format(type(e), e, traceback.format_exc()))
                return None

        if raw_value is not None:
            # We have fresh data, process it
            arr = np.array(raw_value).reshape((self._rows, self._cols))
            arr = self._apply_orientation(arr)
            
            # Apply filters and perform health check
            filtered_arr = self._process_and_health_check(arr)
            
            # Cache the result
            self._last_distance = filtered_arr.flatten().tolist()
            self._last_distance_time = dt.now()

        # If we are in a blind state, always return None
        if self._is_blind:
            return None

        # If we didn't get new data, check if we can use stale data
        if raw_value is None:
            if self._last_distance is not None and self._last_distance_time is not None:
                age_sec = (dt.now() - self._last_distance_time).total_seconds()
                if age_sec <= self._stale_data_timeout_sec:
                    return self._last_distance
        
        return self._last_distance

    def _calibrate_floor_rows(self):
        '''
        Calibrates floor row detection by analyzing multiple samples.
        Floor rows have low standard deviation (consistent readings).
        Starts from row 0 (bottom) and works upward until a non-floor row is found.
        
        Row 0 = bottom/floor, Row 7 = top/far
        '''
        self._log.info('calibrating floor rows using {} samples…'.format(self._calibration_samples))
        self._floor_row_means   = [None for _ in range(self._rows)]
        self._floor_row_stddevs = [None for _ in range(self._rows)]
        samples = []
        for i in range(self._calibration_samples):
            # Temporarily disable blind check for calibration
            is_blind_backup = self._is_blind
            self._is_blind = False
            data = self.get_distance_mm()
            self._is_blind = is_blind_backup
            
            if data is None:
                self._log.info("calibration sample {} is None.".format(i))
                continue
            arr = np.array(data).reshape((self._rows, self._cols))
            samples.append(arr)
            time.sleep(0.15)
        if len(samples) == 0:
            self._log.warning('no calibration samples collected, check sensor connection.')
            return
        samples = np.array(samples)  # shape: (samples, _rows, _cols)
        
        # check if we have minimum clear space (using upper rows which should see into distance)
        # Row 7 = top/far, rows 4-7 are upper half
        upper_rows_data = samples[:, 4:8, :].flatten()  # rows 4-7 should see far
        upper_mean = upper_rows_data.mean()
        clear_distance = upper_mean >= self._minimum_free_distance
        if not clear_distance:
            self._log.warning('not enough clear space detected: upper rows mean = {:.1f}mm (minimum: {}mm)'.format(
                upper_mean, self._minimum_free_distance))
        
        # identify floor rows starting from bottom (row 0) and working up
        # Row 0 = bottom/floor, Row 7 = top/far
        for row in range(self._rows):  # 0, 1, 2, 3, 4, 5, 6, 7
            values = samples[:, row, :].flatten()
            mean = values.mean()
            stddev = values.std()
            self._log.info('row {}: mean={:.1f}, stddev={:.1f}'.format(row, mean, stddev))
            
            # floor detection: low stddev = consistent reading = floor
            if stddev < self._stddev_threshold:
                self._floor_row_means[row] = mean
                self._floor_row_stddevs[row] = stddev
                self._log.info('row {} marked as floor (mean={:.1f}, stddev={:.1f})'.format(row, mean, stddev))
            else:
                # high stddev = variable distances = not floor
                self._floor_row_means[row] = None
                self._floor_row_stddevs[row] = None
                self._log.info('row {} NOT floor (mean={:.1f}, stddev={:.1f})'.format(row, mean, stddev))
                # all rows above this cannot be floor rows
                for above_row in range(row + 1, self._rows):  # mark rows (row+1) through 7 as not floor
                    self._floor_row_means[above_row] = None
                    self._floor_row_stddevs[above_row] = None
                break  # exit the calibration loop
        
        detected_floor_rows = [i for i, v in enumerate(self._floor_row_means) if v is not None]
        if detected_floor_rows:
            self._log.info('floor rows detected (indices): {}'.format(detected_floor_rows))
        
        if all(val is None for val in self._floor_row_means):
            # force bottom row as floor
            values = samples[:, 0, :].flatten()  # row 0 is bottom
            self._floor_row_means[0] = values.mean()
            self._floor_row_stddevs[0] = values.std()
            if clear_distance:
                self._log.warning('no floor detected during calibration, forcibly marking bottom row (0) as floor.')
            else:
                self._log.error('could not calibrate: not enough clear space in front of robot, forcibly marking bottom row (0) as floor.')
        self._log.info('floor rows calibrated.')

    def _terminate_subprocess(self):
        '''
        Terminates the multiprocessing subprocess with escalating force.
        Returns True if successfully terminated, False otherwise.
        '''
        if not self._process.is_alive():
            self._log.debug('subprocess already stopped.')
            self._process = None
            return True
        try:
            self._log.info('signaling subprocess to stop…')
            self._stop_event.set()
            # give it 500ms to exit gracefully
            self._process.join(timeout=0.5)
            if self._process.is_alive():
                self._log.warning('subprocess did not stop gracefully, terminating…')
                self._process.terminate()
                self._process.join(timeout=0.5)
            if self._process.is_alive():
                self._log.error('subprocess still running, killing forcibly…')
                self._process.kill()
                self._process.join(timeout=0.5)
            if self._process.is_alive():
                self._log.error('subprocess could not be killed!')
                return False
            self._log.info('subprocess stopped successfully.')
            self._process = None
            return True
        except Exception as e:
            self._log.error('error terminating subprocess: {} - {}'.format(type(e), e))
            self._process = None
            return False

    def disable(self):
        if self.enabled:
            self._log.info('stopping VL53L5CX…')
            # stop the subprocess if running
            if self._use_multiprocessing and self._process:
                self._terminate_subprocess()
            # stop ranging in main process
            try:
                self._vl53.stop_ranging()
            except Exception as e:
                self._log.warning('error stopping ranging: {}'.format(e))
            super().disable()
            self._log.info('VL53L5CX disabled and stopped ranging.')
        else:
            self._log.debug('already disabled.')

    def close(self):
        if not self.closed:
            self.disable()
            if self._vl53:
                self._vl53.close() # note: custom method in local copy
            super().close()
            self._log.info('closed.')
        else:
            self._log.warning('already closed.')

#EOF
