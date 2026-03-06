#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-03-05

import os
import time
from collections import deque
from threading import Event, Lock, Thread
from colorama import init, Fore, Style
init()

from core.component import Component
from core.illegal_state_error import IllegalStateError
from core.logger import Logger, Level
from core.adaptive_occupancy_map import AdaptiveOccupancyMap, MapVisualizer
from i2c_master import I2CMaster

class RadiozoaController(I2CMaster, Component):
    NAME              = 'radio-ctrl'
    FAR_THRESHOLD     = 4000  # mm; maximum range of VL53L1X sensors
    DISTANCES_COMMAND = 'distances'
    '''
    Extends I2CMaster to control an ESP32-S3 connected to a Radiozoa sensor board.

    Manages the I2C command interface, a background polling loop, and an
    AdaptiveOccupancyMap populated from sensor readings.
    '''
    def __init__(self, config=None, level=Level.INFO):
        self._cwd = os.getcwd()
        if config is None:
            raise ValueError('no config provided.')
        self._log = Logger(RadiozoaController.NAME, level=level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        _cfg         = config['kros']['hardware']['radiozoa-controller']
        _i2c_bus_id  = _cfg.get('i2c_bus_id')
        _i2c_address = _cfg.get('i2c_address')
        _timeset     = _cfg.get('timeset', True)
        _poll_delay  = _cfg.get('poll_delay_sec', 0.67)
        _map_data_file    = _cfg.get('map_data_file') or None
        self._max_retries = _cfg.get('max_retries', 5)
        # superclass
        I2CMaster.__init__(self, i2c_id=_i2c_bus_id, i2c_address=_i2c_address, timeset=_timeset)
        self._poll_delay_sec = _poll_delay
        self._distances      = None  # list of 8 ints, or None if not yet read
        self._distances_lock = Lock()
        self._stop_event     = Event()
        self._poll_thread    = None
        # dynamic write/read delay
        self._dynamic_tuning = True
        self._tune_window        = 5     # calls per evaluation window
        self._tune_step_down     = 1.0 
        self._tune_step_up       = 1.0
        self._tune_floor_ms      = 4.0
        self._tune_ceiling_ms    = 16.0
        self._tune_threshold_lo  = 0.5   # below this, step down
        self._tune_threshold_hi  = 2.0   # above this, step up
        self._tune_history       = deque(maxlen=self._tune_window) # circular window of bool (True=success)
        # occupancy map
        self._occ_map_path = os.path.join(self._cwd, 'occupancy_map.svg')
        self._log.info('occ map path: {}'.format(self._occ_map_path))
        self._occupancy_map = AdaptiveOccupancyMap(min_cell_size=50, initial_size=8000)
        self._log.info('adaptive occupancy map initialized.')
        if _map_data_file is not None:
            self.load_scan_data(_map_data_file)
        self._log.info('ready.')

    @property
    def name(self):
        return self.NAME

    @property
    def occupancy_map(self):
        '''
        Return the AdaptiveOccupancyMap instance.
        '''
        return self._occupancy_map

    def _parse_distances(self, response):
        '''
        Parse a space-delimited response string into a list of 8 ints.
        Returns None if the response is invalid.
        '''
        if response is None or response == self.DISTANCES_COMMAND:
            return None
        try:
            values = [int(v) for v in response.split()]
            if len(values) != 8:
                self._log.warning('expected 8 distance values, got {}: {}'.format(len(values), response))
                return None
            return values
        except ValueError:
            self._log.warning('could not parse distances response: {}'.format(response))
            return None
        except Exception as e:
            self._log.error('{} raised parsing distances response: {}'.format(type(e), e))
            return None

    def _tune_record(self, retries):
        '''
        record retry count and adjust write/read delay after each window.
        dead band between thresholds prevents reaction to normal variation.
        '''
        self._tune_history.append(retries)
        if len(self._tune_history) < self._tune_window:
            return
        avg_retries = sum(self._tune_history) / self._tune_window
        self._tune_history.clear()
        current_ms = self.get_write_read_delay_ms()
        if avg_retries > self._tune_threshold_hi:
            new_ms = min(current_ms + self._tune_step_up, self._tune_ceiling_ms)
        elif avg_retries < self._tune_threshold_lo:
            new_ms = max(current_ms - self._tune_step_down, self._tune_floor_ms)
        else:
            new_ms = current_ms
        if new_ms != current_ms:
            self.set_write_read_delay_ms(new_ms)
            self._log.info('write/read delay: {:.1f}ms → '.format(current_ms)
                    + Fore.GREEN + '{:.1f}ms '.format(new_ms)
                    + Fore.CYAN + '(avg retries: {:.2f})'.format(avg_retries))
        else:
            self._log.debug(Style.DIM + 'write/read delay: {:.1f}ms (avg retries: {:.2f})'.format(current_ms, avg_retries))

    def get_distances(self):
        '''
        Fetch a fresh set of distances from the sensor, store and return them.
        Retries up to max_retries times if the response is invalid.
        Returns a list of 8 ints (mm), or None if retries are exhausted.
        '''
        retries = 0
        response = self.send_request(self.DISTANCES_COMMAND)
        while response is None \
                or response == self.DISTANCES_COMMAND \
                or response == 'ACK' \
                or response == 'ERR':
            if retries >= self._max_retries:
                self._log.warning('max retries ({}) exceeded: no valid distances response.'.format(self._max_retries))
                if self._dynamic_tuning:
                    self._tune_record(self._max_retries)
                return None
            time.sleep(0.01)
            response = self.send_request(self.DISTANCES_COMMAND)
            retries += 1
        distances = self._parse_distances(response)
        if distances is not None:
            with self._distances_lock:
                self._distances = distances
        if self._dynamic_tuning:
            self._tune_record(retries)
        return distances

    @property
    def last_distances(self):
        '''
        Return the most recently fetched distances without issuing a new request.
        Returns a list of 8 ints (mm), or None if no reading has been taken.
        '''
        with self._distances_lock:
            return list(self._distances) if self._distances is not None else None

    def perform_scan(self, xyz):
        '''
        Request a distances snapshot and add it to the occupancy map.
        Returns the distances as a list of 8 ints, or None on failure.
        '''
        try:
            x, y, heading = xyz
            self._log.info('scanning at ({},{},{}°)…'.format(x, y, heading))
            distances = self.get_distances()
            while distances is None:
#               self._log.warning('invalid response, retrying…')
                distances = self.get_distances()
            response = ' '.join('{:04d}'.format(d) for d in distances)
            self._log.info(Fore.CYAN + 'response: ' + Fore.YELLOW + '({},{},{}°) '.format(x, y, heading) + Fore.GREEN + '{}'.format(response))
            self._occupancy_map.process_sensor_reading(response, x, y, heading)
            self._log.info('sensor reading added to map.')
            return distances
        except Exception as e:
            self._log.error('{} raised in perform_scan: {}'.format(type(e), e))
            return None

    def load_scan_data(self, filename):
        '''
        Load and process scan data from a text file into the occupancy map.
        each line: x y heading distance1 distance2 ... distance8
        '''
        try:
            path = os.path.join(self._cwd, filename)
            self._log.info('loading scan data from {}…'.format(path))
            with open(path, 'r') as f:
                lines = f.readlines()
            scan_count = 0
            for line in lines:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if len(parts) != 11:
                    self._log.warning('skipping invalid line: {}'.format(line))
                    continue
                x         = int(parts[0])
                y         = int(parts[1])
                heading   = int(parts[2])
                distances = ' '.join(parts[3:11])
                self._occupancy_map.process_sensor_reading(distances, x, y, heading)
                scan_count += 1
                self._log.info(Fore.CYAN + 'loaded scan {}: '.format(scan_count) + Fore.YELLOW + '({},{},{}°) '.format(x, y, heading) + Fore.GREEN + '{}'.format(distances))
            self._log.info('{} scans loaded from {}'.format(scan_count, filename))
        except FileNotFoundError:
            self._log.error('file not found: {}'.format(filename))
        except Exception as e:
            self._log.error('{} raised loading scan data: {}'.format(type(e), e))

    def generate_svg(self, trim=True, margin_mm=100):
        '''
        Generate an SVG visualisation of the occupancy map.
        '''
        if trim:
            self._occupancy_map.trim_to_data(margin_mm=margin_mm)
        visualizer = MapVisualizer(self._occupancy_map)
        visualizer.generate_svg(self._occ_map_path)

    def start_polling(self):
        '''
        Start a background thread that repeatedly calls get_distances().
        Has no effect if polling is already running.
        '''
        if not self.enabled:
            raise IllegalStateError('not enabled.')
        if self._poll_thread and self._poll_thread.is_alive():
            self._log.warning('polling already running.')
            return
        self._stop_event.clear()
        self._poll_thread = Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()
        self._log.info('polling started.')

    def stop_polling(self):
        '''
        Stop the background polling thread.
        '''
        if self._poll_thread and self._poll_thread.is_alive():
            self._stop_event.set()
            self._poll_thread.join()
            self._log.info('polling stopped.')
        else:
            self._log.warning('polling not running.')
        self._poll_thread = None

    def _poll_loop(self):
        '''
        Internal polling loop, runs until stop_event is set.
        '''
        self._log.info('poll loop started.')
        try:
            while not self._stop_event.is_set():
                distances = self.get_distances()
                if distances is None:
                    self._log.info(Style.DIM + 'poll: no valid response.')
                else:
                    self._log.info('poll: ' + Fore.GREEN + '{}'.format(distances))
                time.sleep(self._poll_delay_sec)
        finally:
            self._log.info('poll loop stopped.')

    def enable(self):
        '''
        Enable the RadiozoaController.
        '''
        if not self.enabled:
            super().enable()
            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def disable(self):
        '''
        Stop polling (if running) then disable the RadiozoaController.
        '''
        if self.enabled:
            if self._poll_thread and self._poll_thread.is_alive():
                self.stop_polling()
            super().disable()
        else:
            self._log.debug('already disabled.')

    def close(self):
        '''
        Disable and close the RadiozoaController.
        '''
        if not self.closed:
            super().close()
        else:
            self._log.warning('already closed.')

#EOF
