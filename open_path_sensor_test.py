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
import numpy as np
from colorama import init, Fore, Style
init()

import vl53l5cx_ctypes as vl53l5cx
from vl53l5cx_ctypes import RANGING_MODE_CONTINUOUS

from core.component import Component
from core.logger import Logger, Level
from core.config_loader import ConfigLoader

class OpenPathSensor(Component):
    '''
    OpenPathSensor sets up and manages the VL53L5CX ToF sensor,
    analyzes its multizone data to determine the most open direction,
    and outputs filtered steering/motor multipliers.
    If "skip" is True, firmware upload to VL53L5CX is skipped.

    Convention: Sensor does not begin ranging until enable() is called.
    '''
    def __init__(self, config, skip=False, level=Level.INFO):
        self._log = Logger('openpath', level)
        Component.__init__(self, self._log, suppressed=True, enabled=False)
        self._log.info('initialising OpenPathSensor…')
        # configuration
        _cfg = config['kros'].get('hardware').get('open_path_sensor')
        if _cfg is None or not isinstance(_cfg, dict):
            raise ValueError('invalid config: missing kros.hardware.open_path_sensor section')
        self.COLS = _cfg.get('cols', 8)
        self.ROWS = _cfg.get('rows', 8)
        self.FOV = _cfg.get('fov', 47.0)
        self.DISTANCE_THRESHOLD = _cfg.get('distance_threshold', 1000)
        self.ROW_LOWER = _cfg.get('row_lower', 0)
        self.ROW_UPPER = _cfg.get('row_upper', 2)
        self.WEIGHTS = np.array(_cfg.get('weights', [0.6, 0.3, 0.1]))
        self.ALPHA = _cfg.get('alpha', 0.08)
        self._filtered_offset = 0.0
        # hardware support setup
        self._log.info('initialising VL53L5CX sensor{}…'.format(' (skip firmware upload)' if skip else ''))
        self.vl53 = vl53l5cx.VL53L5CX(skip_init=skip)
        self.vl53.set_resolution(self.COLS * self.ROWS)
        self.vl53.set_ranging_frequency_hz(15)
        self.vl53.set_integration_time_ms(20)
        self.vl53.set_ranging_mode(RANGING_MODE_CONTINUOUS)
        self._log.info('open path sensor ready.')

    def enable(self):
        '''
        Enables sensor and starts ranging if not already running.
        '''
        if not self.enabled:
            self.vl53.start_ranging()
            super().enable()
            self._log.info('VL53L5CX sensor enabled and ranging.')
        else:
            self._log.info('VL53L5CX sensor already enabled.')

    def get_distance_mm(self):
        '''
        Wait for sensor data and return the 8x8 grid as a flat list.
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

    def process(self, distance_mm):
        distance = np.array(distance_mm).reshape((self.ROWS, self.COLS))
        pixel_angles = [-(self.FOV/2) + (i + 0.5) * (self.FOV/self.COLS) for i in range(self.COLS)]

        if not np.any(distance < self.DISTANCE_THRESHOLD):
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
        for col in range(self.COLS):
            values = distance[self.ROW_LOWER:self.ROW_UPPER+1, col]
            avg = np.average(values, weights=self.WEIGHTS)
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

    def disable(self):
        '''
        Disables sensor and stops ranging.
        '''
        if self.enabled:
            self.vl53.stop_ranging()
            super().disable()
            self._log.info('open path sensor disabled and stopped ranging.')
        else:
            self._log.info('open path sensor already disabled.')

    def close(self):
        '''
        Closes the component and stops sensor.
        '''
        if not self.closed:
            self.vl53.stop_ranging()
            super().close()
            self._log.info('open path sensor closed.')
        else:
            self._log.info('open path sensor already closed.')

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

DIST_COLORS = [
    (0,     150,  Fore.RED),
    (151,   300,  Fore.YELLOW),
    (301,   500,  Fore.GREEN),
    (501,   800,  Fore.BLUE),
    (801,  1000,  Fore.BLUE),
    (1001, 99999, Fore.BLACK),
]

def get_dist_color(val):
    for low, high, color in DIST_COLORS:
        if low <= val <= high:
            return color
    return Fore.BLACK  # fallback

def print_colored_grid(distance, COLS):
    print(Style.BRIGHT + "{}".format("┈" * (COLS * 6)) + Style.RESET_ALL)
    for row in reversed(range(distance.shape[0])):
        line = ""
        for col in range(distance.shape[1]):
            val = distance[row, col]
            color = get_dist_color(val)
            line += "{}{:4d}{}".format(color, val, Style.RESET_ALL) + " "
        print(line)
    print(Style.RESET_ALL)

def print_target_row(weighted_avgs, highlighted_idx, pixel_angles, target_offset, filtered_offset, port_mult, starboard_mult):
    line = ""
    for i, val in enumerate(weighted_avgs):
        if highlighted_idx is not None and i == highlighted_idx:
            color = Fore.WHITE
            style = Style.BRIGHT
        else:
            color = Fore.BLACK
            style = ""
        line += "{}{}{:4d}{}".format(color, style, int(val), Style.RESET_ALL) + " "
    print(line + "   Target offset: {:+.2f}°   (filtered: {:+.2f}°)".format(target_offset, filtered_offset))
    print("Filtered Port multiplier: {:.2f}  Filtered Starboard multiplier: {:.2f}".format(port_mult, starboard_mult))

def main():
    _log = Logger('main', level=Level.INFO)
    _config = ConfigLoader(Level.INFO).configure()
    skip = 'skip' in sys.argv or True in sys.argv
    _open_path_sensor = OpenPathSensor(_config, skip=skip, level=Level.INFO)
    _open_path_sensor.enable()

    try:
        while True:
            distance_mm = _open_path_sensor.get_distance_mm()
            if distance_mm is None:
                continue
            result = _open_path_sensor.process(distance_mm)
            distance = np.array(distance_mm).reshape((_open_path_sensor.ROWS, _open_path_sensor.COLS))
            print_colored_grid(distance, _open_path_sensor.COLS)
            print_target_row(
                result['weighted_avgs'],
                result['highlighted_idx'],
                result['pixel_angles'],
                result['target_offset'],
                result['filtered_offset'],
                result['port_mult'],
                result['starboard_mult']
            )
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("{}caught Ctrl-C; exiting…{}".format(Style.BRIGHT, Style.RESET_ALL))
    finally:
        _open_path_sensor.close()
        print("{}complete.{}".format(Fore.CYAN, Style.RESET_ALL))

if __name__== "__main__":
    main()

#EOF
