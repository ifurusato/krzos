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

import sys
import time
import threading
import numpy as np

from hardware.vl53l5cx_sensor import Vl53l5cxSensor
from core.component import Component
from core.logger import Logger, Level
from core.config_loader import ConfigLoader

class OpenPathSensor(Component):
    '''
    OpenPathSensor analyzes VL53L5CX multizone data to determine the most open direction.
    The control loop runs in a thread when enabled, and can send display data to an external display class via callback.
    Only contiguous bottom rows can be floor rows. Once a row fails the stddev check, all above are not floor.
    '''
    def __init__(self, config, sensor, level=Level.INFO, calibration_samples=10, stddev_threshold=100, margin=50, display=None, loop_period=0.2):
        self._log = Logger('openpath', level)
        Component.__init__(self, self._log, suppressed=True, enabled=False)
        self._log.info('initialising OpenPathSensor…')
        _cfg = config['kros'].get('hardware').get('open_path_sensor')
        if _cfg is None or not isinstance(_cfg, dict):
            raise ValueError('invalid config: missing kros.hardware.open_path_sensor section')
        self.COLS = _cfg.get('cols', 8)
        self.ROWS = _cfg.get('rows', 8)
        self.FOV = _cfg.get('fov', 47.0)
        self.DISTANCE_THRESHOLD = _cfg.get('distance_threshold', 1000)
        self.WEIGHTS = np.array(_cfg.get('weights', [0.6, 0.3, 0.1]))
        self.ALPHA = _cfg.get('alpha', 0.08)
        self._filtered_offset = 0.0
        self.calibration_samples = calibration_samples
        self.stddev_threshold = stddev_threshold
        self.margin = margin
        self.sensor = sensor
        self.floor_row_means = [None for _ in range(self.ROWS)]
        self.floor_row_stddevs = [None for _ in range(self.ROWS)]
        self._display = display
        self._thread = None
        self._running = False
        self.loop_period = loop_period
        self._log.info('open path sensor ready.')

    def enable(self):
        if not self.enabled:
            self.sensor.enable()
            super().enable()
            self._log.info('OpenPathSensor enabled and sensor ranging.')
            self.calibrate_floor_rows()
            self._running = True
            self._thread = threading.Thread(target=self._control_loop, daemon=True)
            self._thread.start()
        else:
            self._log.info('OpenPathSensor already enabled.')

    def disable(self):
        self._running = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        if self.enabled:
            self.sensor.disable()
            super().disable()
            self._log.info('OpenPathSensor disabled and sensor stopped ranging.')
        else:
            self._log.info('OpenPathSensor already disabled.')

    def close(self):
        self.disable()
        if not self.closed:
            self.sensor.close()
            super().close()
            self._log.info('OpenPathSensor closed.')
        else:
            self._log.info('OpenPathSensor already closed.')

    def set_display(self, display):
        self._display = display

    def _control_loop(self):
        while self._running:
            distance_mm = self.get_distance_mm()
            if distance_mm is not None:
                result = self.process(distance_mm)
                if self._display is not None:
                    self._display.update(distance_mm, self.floor_row_means, self.margin, result)
            time.sleep(self.loop_period)

    def calibrate_floor_rows(self):
        self._log.info('Calibrating floor rows...')
        self.floor_row_means = [None for _ in range(self.ROWS)]
        self.floor_row_stddevs = [None for _ in range(self.ROWS)]
        samples = []
        for i in range(self.calibration_samples):
            data = self.sensor.get_distance_mm()
            if data is None:
                self._log.warning(f"Calibration sample {i} is None.")
                continue
            arr = np.array(data).reshape((self.ROWS, self.COLS))
            samples.append(arr)
            time.sleep(0.05)
        if len(samples) == 0:
            self._log.warning('No calibration samples collected! Check sensor connection.')
            return
        samples = np.array(samples)  # shape: (samples, ROWS, COLS)
        for row in range(self.ROWS):
            values = samples[:, row, :].flatten()
            mean = values.mean()
            stddev = values.std()
            self._log.info(f'Row {row}: mean={mean:.1f}, stddev={stddev:.1f}')
            if stddev < self.stddev_threshold:
                self.floor_row_means[row] = mean
                self.floor_row_stddevs[row] = stddev
                self._log.info(f'Row {row} marked as floor (mean={mean:.1f}, stddev={stddev:.1f})')
            else:
                self.floor_row_means[row] = None
                self.floor_row_stddevs[row] = None
                self._log.info(f'Row {row} NOT floor (mean={mean:.1f}, stddev={stddev:.1f})')
                # All rows above this cannot be floor rows
                for above_row in range(row+1, self.ROWS):
                    self.floor_row_means[above_row] = None
                    self.floor_row_stddevs[above_row] = None
                break  # exit the calibration loop
        detected_floor_rows = [i for i, v in enumerate(self.floor_row_means) if v is not None]
        self._log.info(f'Floor rows detected (indices): {detected_floor_rows}')
        if all(val is None for val in self.floor_row_means):
            # Force bottom row as floor
            values = samples[:, 0, :].flatten()
            self.floor_row_means[0] = values.mean()
            self.floor_row_stddevs[0] = values.std()
            self._log.warning('No floor detected during calibration! Forcibly marking bottom row as floor.')

    def x_calibrate_floor_rows(self):
        self._log.info('Calibrating floor rows...')
        samples = []
        for i in range(self.calibration_samples):
            data = self.sensor.get_distance_mm()
            if data is None:
                continue
            arr = np.array(data).reshape((self.ROWS, self.COLS))
            samples.append(arr)
            time.sleep(0.05)
        samples = np.array(samples)  # shape: (samples, ROWS, COLS)
        # Only contiguous bottom rows can be floor rows.
        for row in range(self.ROWS):
            values = samples[:, row, :].flatten()
            mean = values.mean()
            stddev = values.std()
            if stddev < self.stddev_threshold:
                self.floor_row_means[row] = mean
                self.floor_row_stddevs[row] = stddev
                self._log.info(f'Row {row} marked as floor (mean={mean:.1f}, stddev={stddev:.1f})')
            else:
                self.floor_row_means[row] = None
                self.floor_row_stddevs[row] = None
                self._log.info(f'Row {row} NOT floor (mean={mean:.1f}, stddev={stddev:.1f})')
                # All rows above this cannot be floor rows
                for above_row in range(row+1, self.ROWS):
                    self.floor_row_means[above_row] = None
                    self.floor_row_stddevs[above_row] = None
                break  # exit the calibration loop
        detected_floor_rows = [i for i, v in enumerate(self.floor_row_means) if v is not None]
        self._log.info(f'Floor rows detected (indices): {detected_floor_rows}')
        if all(val is None for val in self.floor_row_means):
            self._log.warning('No floor rows detected during calibration!')

    def get_distance_mm(self):
        if not self.enabled:
            self._log.warning('get_distance_mm called while OpenPathSensor is not enabled.')
            return None
        return self.sensor.get_distance_mm()

    def process(self, distance_mm):
        distance = np.array(distance_mm).reshape((self.ROWS, self.COLS))
        pixel_angles = [-(self.FOV/2) + (i + 0.5) * (self.FOV/self.COLS) for i in range(self.COLS)]
        obstacle_rows = [r for r in range(self.ROWS) if self.floor_row_means[r] is None]
        if not obstacle_rows or not np.any(distance[obstacle_rows, :] < self.DISTANCE_THRESHOLD):
            target_offset = 0.0
            self._filtered_offset = 0.0
            port_mult = 1.0
            starboard_mult = 1.0
            weighted_avgs = [0] * self.COLS
            highlighted_idx = min(range(self.COLS), key=lambda i: abs(pixel_angles[i] - self._filtered_offset))
            return dict(
                weighted_avgs=weighted_avgs,
                target_offset=target_offset,
                filtered_offset=self._filtered_offset,
                port_mult=port_mult,
                starboard_mult=starboard_mult,
                highlighted_idx=highlighted_idx,
                pixel_angles=pixel_angles
            )
        weighted_avgs = []
        weights = self.WEIGHTS[:len(obstacle_rows)] if len(self.WEIGHTS) >= len(obstacle_rows) else np.ones(len(obstacle_rows))
        for col in range(self.COLS):
            values = distance[obstacle_rows, col]
            avg = np.average(values, weights=weights)
            weighted_avgs.append(avg)
        max_idx = int(np.argmax(weighted_avgs))
        target_offset = pixel_angles[max_idx]
        self._filtered_offset = self.ALPHA * target_offset + (1 - self.ALPHA) * self._filtered_offset
        port_mult, starboard_mult = self.get_motor_multipliers(self._filtered_offset, self.FOV/2)
        highlighted_idx = min(range(self.COLS), key=lambda i: abs(pixel_angles[i] - self._filtered_offset))
        return dict(
            weighted_avgs=weighted_avgs,
            target_offset=target_offset,
            filtered_offset=self._filtered_offset,
            port_mult=port_mult,
            starboard_mult=starboard_mult,
            highlighted_idx=highlighted_idx,
            pixel_angles=pixel_angles
        )

    def get_motor_multipliers(self, offset, max_angle):
        port_mult = 1.0 - max(0, offset / max_angle)
        starboard_mult = 1.0 - max(0, -offset / max_angle)
        port_mult = max(0.0, min(port_mult, 1.0))
        starboard_mult = max(0.0, min(starboard_mult, 1.0))
        return port_mult, starboard_mult

class OpenPathDisplay:
    '''
    Handles console display of open path sensor data.
    '''
    DIST_COLORS = [
        (0,     150,  "\033[31m"),  # RED
        (151,   300,  "\033[33m"),  # YELLOW
        (301,   500,  "\033[32m"),  # GREEN
        (501,   800,  "\033[34m"),  # BLUE
        (801,  1000,  "\033[34m"),  # BLUE
        (1001, 99999, "\033[30m"),  # BLACK
    ]
    FLOOR_COLOR = "\033[35;1m"  # Bright magenta

    def __init__(self, cols, rows):
        self.cols = cols
        self.rows = rows

    def get_dist_color(self, val):
        for low, high, color in self.DIST_COLORS:
            if low <= val <= high:
                return color
        return "\033[30m"

    def update(self, distance_mm, floor_row_means, margin, result):
        distance = np.array(distance_mm).reshape((len(floor_row_means), self.cols))
        self.print_colored_grid(distance, self.cols, floor_row_means, margin)
        self.print_target_row(
            result['weighted_avgs'],
            result['highlighted_idx'],
            result['pixel_angles'],
            result['target_offset'],
            result['filtered_offset'],
            result['port_mult'],
            result['starboard_mult']
        )

    def print_colored_grid(self, distance, COLS, floor_row_means, margin):
        print("\033[1m" + "{}".format("┈" * (COLS * 6)) + "\033[0m")
        print(f"DEBUG: floor_row_means = {floor_row_means}")
        print("DEBUG: Floor rows (indices):", [i for i, v in enumerate(floor_row_means) if v is not None])
        for row in reversed(range(distance.shape[0])):
            line = ""
            for col in range(distance.shape[1]):
                val = distance[row, col]
                floor_mean = floor_row_means[row]
                # Only color as floor for values in floor rows, unless obstacle
                if floor_mean is not None:
                    if val >= (floor_mean - margin):
                        color = self.FLOOR_COLOR
                    else:
                        color = self.get_dist_color(val)  # obstacle color
                else:
                    color = self.get_dist_color(val)
                line += f"{color}{val:4d}\033[0m "
            print(line + f"   # row {row}")
        print("\033[0m")

    def print_target_row(self, weighted_avgs, highlighted_idx, pixel_angles, target_offset, filtered_offset, port_mult, starboard_mult):
        line = ""
        for i, val in enumerate(weighted_avgs):
            if highlighted_idx is not None and i == highlighted_idx:
                color = "\033[37;1m"  # Bright white
            else:
                color = "\033[30m"    # Black
            line += f"{color}{int(val):4d}\033[0m "
        print(line + f"   Target offset: {target_offset:+.2f}°   (filtered: {filtered_offset:+.2f}°)")
        print(f"Filtered Port multiplier: {port_mult:.2f}  Filtered Starboard multiplier: {starboard_mult:.2f}")

def main():
    _log = Logger('main', level=Level.INFO)
    _config = ConfigLoader(Level.INFO).configure()
    skip = 'skip' in sys.argv or True in sys.argv
    vl53_sensor = Vl53l5cxSensor(_config, skip=skip, level=Level.INFO)
    display = OpenPathDisplay(cols=8, rows=8)
    open_path_sensor = OpenPathSensor(_config, sensor=vl53_sensor, level=Level.INFO, display=display)
    open_path_sensor.enable()

    try:
        while True:
            time.sleep(0.5)  # Let the sensor/display thread run
    except KeyboardInterrupt:
        print("\033[1mcaught Ctrl-C; exiting…\033[0m")
    finally:
        open_path_sensor.close()
        print("\033[36mcomplete.\033[0m")

if __name__== "__main__":
    main()

#EOF
