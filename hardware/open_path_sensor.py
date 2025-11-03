#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-02
# modified: 2025-10-05

import sys
import time
from threading import Thread
import numpy as np
from colorama import init, Fore, Style
init()

from core.component import Component
from core.orientation import Orientation
from core.logger import Logger, Level
from core.rate import Rate
from hardware.vl53l5cx_sensor import Vl53l5cxSensor

class OpenPathSensor(Component):
    NAME = 'open-path'
    OPEN_PATH_PORT_LAMBDA_NAME =  "__open_path_port"
    OPEN_PATH_STBD_LAMBDA_NAME =  "__open_path_stbd"
    '''
    OpenPathSensor analyzes VL53L5CX multizone data to determine the most
    open direction.
    The control loop runs in a thread when enabled, and can send display
    data to an external visualiser class via callback. Only contiguous bottom
    rows can be floor rows. Once a row fails the stddev check, all above
    are not floor.

    If the Vl53l5cxSensor is not provided it will be created.
    '''
    def __init__(self, config, vl53l5cx=None, visualiser=None, motor_control=False, level=Level.INFO):
        self._log = Logger(OpenPathSensor.NAME, level=level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._log.info('initialising OpenPathSensor…')
        if config is None or not isinstance(config, dict):
            raise ValueError('invalid configuration argument.')
        if vl53l5cx is None:
            # see if it's in the ComponentRegistry
            _component_registry = Component.get_registry()
            self._vl53l5cx = _component_registry.get(Vl53l5cxSensor.NAME)
            if self._vl53l5cx is None:
                # otherwise we make one
                self._vl53l5cx = Vl53l5cxSensor(config, level=Level.INFO)
        else:
            self._vl53l5cx = vl53l5cx
        # configuration
        _vl53_cfg = config['kros'].get('hardware').get('vl53l5cx_sensor')
        self._cols = _vl53_cfg.get('cols', 8)
        self._rows = _vl53_cfg.get('rows', 8)
        self._fov  = _vl53_cfg.get('fov', 47.0)
        _cfg = config['kros'].get('hardware').get('open_path_sensor')
        self._distance_threshold = _cfg.get('distance_threshold', 1000)
        self._weights = np.array(_cfg.get('weights', [0.6, 0.3, 0.1]))
        self._alpha   = _cfg.get('alpha', 0.08) #  low-pass filter coefficient
        self._floor_margin = _cfg.get('floor_margin', 50)
        self._rate = Rate(_cfg.get('loop_freq_hz', 5)) # 5Hz, 200ms default
        # variables
        self.set_visualiser(visualiser)
        self._thread  = None
        self._running = False
        self._filtered_offset   = 0.0
        self._port_multiplier   = 1.0
        self._stbd_multiplier   = 1.0
        self._open_path_port_lambda = lambda speed: speed * self._port_multiplier
        self._open_path_stbd_lambda = lambda speed: speed * self._stbd_multiplier
        self._log.info('open path sensor ready.')

    def set_visualiser(self, visualiser):
        '''
        Set the optional visualiser used by the sensor.
        '''
        self._visualiser = visualiser

    def _control_loop(self):
        while self._running:
            distance_mm = self.get_distance_mm()
            if distance_mm is not None:
                result = self._process(distance_mm)
                if self._visualiser is not None:
                    self._visualiser.update(distance_mm, self._vl53l5cx.floor_row_means, self._vl53l5cx.floor_margin, result)
                else:
                    self._log.info('port multiplier: ' + Fore.RED + '{:.2f}'.format(self._port_multiplier)
                            + Fore.CYAN + '; stbd multiplier: ' + Fore.GREEN + '{:.2f}'.format(self._stbd_multiplier))
            self._rate.wait()

    def get_distance_mm(self):
        if not self.enabled:
            self._log.warning('get_distance_mm called while OpenPathSensor is not enabled.')
            return None
        return self._vl53l5cx.get_distance_mm()

    def _process(self, distance_mm):
        distance = np.array(distance_mm).reshape((self._rows, self._cols))
        pixel_angles = [-(self._fov/2) + (i + 0.5) * (self._fov/self._cols) for i in range(self._cols)]
        obstacle_rows = [r for r in range(self._rows) if self._vl53l5cx.floor_row_means[r] is None]
        if not obstacle_rows or not np.any(distance[obstacle_rows, :] < self._distance_threshold):
            target_offset = 0.0
            self._filtered_offset = 0.0
            self._port_multiplier = 1.0
            self._stbd_multiplier = 1.0
            weighted_avgs = [0] * self._cols
            highlighted_idx = min(range(self._cols), key=lambda i: abs(pixel_angles[i] - self._filtered_offset))
            return dict(
                weighted_avgs=weighted_avgs,
                target_offset=target_offset,
                filtered_offset=self._filtered_offset,
                port_mult=self._port_multiplier,
                stbd_mult=self._stbd_multiplier,
                highlighted_idx=highlighted_idx,
                pixel_angles=pixel_angles
            )
        weighted_avgs = []
        weights = self._weights[:len(obstacle_rows)] if len(self._weights) >= len(obstacle_rows) else np.ones(len(obstacle_rows))
        for col in range(self._cols):
            values = distance[obstacle_rows, col]
            avg = np.average(values, weights=weights)
            weighted_avgs.append(avg)
        max_idx = int(np.argmax(weighted_avgs))
        target_offset         = pixel_angles[max_idx]
        self._filtered_offset = self._alpha * target_offset + (1 - self._alpha) * self._filtered_offset
        self._port_multiplier, self._stbd_multiplier  = self._get_motor_multipliers(self._filtered_offset, self._fov/2)
        highlighted_idx       = min(range(self._cols), key=lambda i: abs(pixel_angles[i] - self._filtered_offset))
        return dict(
            weighted_avgs   = weighted_avgs,
            target_offset   = target_offset,
            filtered_offset = self._filtered_offset,
            port_mult       = self._port_multiplier,
            stbd_mult       = self._stbd_multiplier,
            highlighted_idx = highlighted_idx,
            pixel_angles    = pixel_angles
        )

    def _get_motor_multipliers(self, offset, max_angle):
        self._port_multiplier = 1.0 - max(0, offset / max_angle)
        self._stbd_multiplier = 1.0 - max(0, -offset / max_angle)
        self._port_multiplier = max(0.0, min(self._port_multiplier, 1.0))
        self._stbd_multiplier = max(0.0, min(self._stbd_multiplier, 1.0))
        return self._port_multiplier, self._stbd_multiplier

    def enable(self):
        if not self.enabled:
            self._vl53l5cx.enable()
            super().enable()
            self._log.info('enabled and sensor ranging.')
            self._running = True
            self._thread = Thread(target=self._control_loop, daemon=True)
            self._thread.start()
        else:
            self._log.info('already enabled.')

    def decorate_motor_controller(self, motor_controller):
        '''
        Adds the speed multiplier lambda functions of this class to the motors.
        '''
        motor_controller.add_lambda(Orientation.PFWD, self.OPEN_PATH_PORT_LAMBDA_NAME, self._open_path_port_lambda)
        motor_controller.add_lambda(Orientation.SFWD, self.OPEN_PATH_STBD_LAMBDA_NAME, self._open_path_stbd_lambda)
        motor_controller.add_lambda(Orientation.PAFT, self.OPEN_PATH_PORT_LAMBDA_NAME, self._open_path_port_lambda)
        motor_controller.add_lambda(Orientation.SAFT, self.OPEN_PATH_STBD_LAMBDA_NAME, self._open_path_stbd_lambda)
        self._log.info('lambda functions added to motors.')

    def disable(self):
        self._running = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        if self.enabled:
            self._vl53l5cx.disable()
            super().disable()
            self._log.info('disabled and sensor stopped ranging.')
        else:
            self._log.info('already disabled.')

    def close(self):
        self.disable()
        if not self.closed:
            self._vl53l5cx.close()
            super().close()
            self._log.info('closed.')
        else:
            self._log.info('already closed.')

#EOF
