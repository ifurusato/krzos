#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-10
# modified: 2025-11-15

import time
import numpy as np
import itertools
from math import isclose
from threading import Thread, Event as ThreadEvent
import asyncio
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.cardinal import Cardinal
from core.event import Event
from behave.async_behaviour import AsyncBehaviour
from hardware.radiozoa_sensor import RadiozoaSensor
from hardware.digital_pot import DigitalPotentiometer
from hardware.motor_controller import MotorController

class Radiozoa(AsyncBehaviour):
    NAME = 'radiozoa'

    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Radiozoa.NAME, level)
        _component_registry = Component.get_registry()
        _motor_controller = _component_registry.get(MotorController.NAME)
        if _motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        AsyncBehaviour.__init__(self, self._log, config, message_bus, message_factory, _motor_controller, level=level)
        self.add_event(Event.RADIOZOA)
        # configuration
        _cfg = config['kros'].get('behaviour').get('radiozoa')
        self._counter    = itertools.count()
        self._default_speed     = _cfg.get('default_speed', 0.4)
        self._output_scale      = _cfg.get('output_scale', 1.0) # default 1.0 = no scaling
        self._use_dynamic_priority = True
        self._use_dynamic_speed = _cfg.get('dynamic_speed', False)
        self._priority          = _cfg.get('default_priority', 0.4)
        self._verbose           = _cfg.get('verbose', False)
        self._use_color  = True # on console messages
        '''
        Smoothing and hysteresis parameters:

          * Deadband (min_sensor_diff_mm): Robot only reacts when paired sensors differ by at
            least 50mm, preventing tiny differences from causing motion.
          * Settling zone (settling_threshold_mm): When all sensor pairs are balanced within
            30mm, robot stops moving - considers itself "settled."
          * Exponential smoothing (smoothing_factor): Smooths the intent vector over time.
            0.3 means 30% of previous value + 70% of new value, providing smooth transitions
            without lag.

        Tracks maximum imbalance: Uses the worst-case pair imbalance to determine if robot
        should settle.
        '''
        # directienal vectors
        self._pairs = [
            (Cardinal.NORTH, Cardinal.SOUTH),
            (Cardinal.NORTHWEST, Cardinal.SOUTHEAST),
            (Cardinal.WEST, Cardinal.EAST),
            (Cardinal.NORTHEAST, Cardinal.SOUTHWEST),
        ]
        _smoothing_factor        = _cfg.get('smoothing_factor', 0.3)             # 0.0=no smoothing, 1.0=no response
        self._smoothing_factor   = max(0.0, min(1.0, _smoothing_factor))
        self._min_sensor_diff    = _cfg.get('min_sensor_diff_mm', 100)       # minimum difference to react
        self._settling_threshold = _cfg.get('settling_threshold_mm', 60)  # all pairs within this = settled
        # exponential moving average for smoothed intent vector
        self._smoothed_vector = np.array([0.0, 0.0])
        self._last_intent_vector = (0.0, 0.0, 0.0)
        # direction vectors for each axis (unit vectors)
        self._directions = {
            (Cardinal.NORTH, Cardinal.SOUTH): np.array([0, 1]),
            (Cardinal.NORTHWEST, Cardinal.SOUTHEAST): np.array([-1, 1]),
            (Cardinal.WEST, Cardinal.EAST): np.array([-1, 0]),
            (Cardinal.NORTHEAST, Cardinal.SOUTHWEST): np.array([1, 1])
        }
        # registry lookups
        _component_registry = Component.get_registry()
        self._digital_pot = None
        if self._use_dynamic_speed:
            self._default_speed = 0.0
            self._digital_pot = _component_registry.get(DigitalPotentiometer.NAME)
        self._radiozoa_sensor = _component_registry.get(RadiozoaSensor.NAME)
        if self._radiozoa_sensor is None:
            self._log.info(Fore.WHITE + 'creating Radiozoa sensor…')
            self._radiozoa_sensor = RadiozoaSensor(config, level=Level.INFO)
        else:
            self._log.info('using existing Radiozoa sensor.')
        self._log.info('ready.')

    @property
    def name(self):
        return Radiozoa.NAME

    @property
    def is_ballistic(self):
        return False

    def execute(self, message):
        raise NotImplementedError('execute unsupported in Radiozoa.')

    def start_loop_action(self):
        pass

    def stop_loop_action(self):
        pass

    def _dynamic_set_default_speed(self):
        _speed = self._digital_pot.get_scaled_value(False) # values 0.0-1.0
        if isclose(_speed, 0.0, abs_tol=0.08):
            self._digital_pot.set_black() # only on digital pot
            self._default_speed = 0.0
            self._log.info(Fore.BLACK + "default speed: stopped")
        else:
            self._digital_pot.set_rgb(self._digital_pot.value)
            self._default_speed = _speed
            self._log.debug("set default speed: {:4.2f}".format(self._default_speed))

    async def _poll(self):
        try:
            if self._use_dynamic_speed:
                if next(self._counter) % 5 == 0:
                    self._dynamic_set_default_speed()
            distances = self._radiozoa_sensor.get_distances()
            if not distances or all(d is None or d > RadiozoaSensor.FAR_THRESHOLD for d in distances):
                # stop when sensors are unavailable or out of range
                self._smoothed_vector = np.array([0.0, 0.0])
                self._priority = 0.3
                return (0.0, 0.0, 0.0)
            else:
                return self._update_intent_vector(distances)
        except Exception as e:
            self._log.error("{} thrown while polling: {}".format(type(e), e))
            self.disable()
            return (0.0, 0.0, 0.0)

    def _update_intent_vector(self, distances):
        '''
        Compute the robot's movement intent as a single (vx, vy, omega) vector.
        Each pair contributes force only if its imbalance exceeds min_sensor_diff_mm.
        Robot settles naturally when all pairs are balanced within threshold.

        Priority scales continuously with imbalance severity and obstacle proximity
        using the sensor's defined threshold ranges, avoiding arbitrary fixed values.
        
        Returns (vx, vy, omega) tuple.
        '''
        for i, d in enumerate(distances):
            if d is not None and d < 50:
                self._log.warning('impossibly close reading from {} sensor: {}mm'.format(Cardinal.from_index(i).label, d))
                return (0.0, 0.0, 0.0)
        far_threshold = RadiozoaSensor.FAR_THRESHOLD * 0.95
        force_vec = np.zeros(2)
        pair_active = False
        max_imbalance = 0.0  # track worst imbalance for priority
        min_distance = float('inf')  # track closest obstacle
        for c1, c2 in self._pairs:
            d1 = self._radiozoa_sensor.get_sensor_by_cardinal(c1).get_distance()
            d2 = self._radiozoa_sensor.get_sensor_by_cardinal(c2).get_distance()

            if d1 < 0 or d2 < 0: # DIAGNOSTIC
                self._log.warning('d1 {} or d2 are less than zero.',format(d1, d2))
            # treat None, zero, negative, or beyond FAR_THRESHOLD as FAR_THRESHOLD
            d1 = d1 if d1 is not None and 0 < d1 <= RadiozoaSensor.FAR_THRESHOLD else RadiozoaSensor.FAR_THRESHOLD
            d2 = d2 if d2 is not None and 0 < d2 <= RadiozoaSensor.FAR_THRESHOLD else RadiozoaSensor.FAR_THRESHOLD
            # track closest obstacle across all sensors
            min_distance = min(min_distance, d1, d2)
            # both sensors out of range - ignore this pair
            if d1 >= far_threshold and d2 >= far_threshold:
                continue
            diff = abs(d1 - d2)
            # track maximum imbalance for priority calculation
            if diff > max_imbalance:
                max_imbalance = diff
            # only contribute force if difference exceeds threshold
            if diff >= self._min_sensor_diff:
                signed_diff = d1 - d2
                vec = self._directions[(c1, c2)] * signed_diff
                force_vec += vec
                pair_active = True
        # if no pairs are contributing force, we're settled
        if not pair_active:
            self._smoothed_vector = np.array([0.0, 0.0])
            self._priority = 0.3  # low priority when settled
            return (0.0, 0.0, 0.0)
        # normalize the raw force vector
        max_abs = np.max(np.abs(force_vec)) if np.max(np.abs(force_vec)) > 1.0 else 1.0
        normalized_vec = force_vec / max_abs
        # apply exponential moving average for smoothing
        self._smoothed_vector = (self._smoothing_factor * self._smoothed_vector +
                                 (1.0 - self._smoothing_factor) * normalized_vec)
        vx, vy = self._smoothed_vector
        amplitude = self._default_speed
        # calculate priority using analog sensor values
        # imbalance urgency: normalized from min_sensor_diff to FAR_THRESHOLD
        if max_imbalance > self._min_sensor_diff:
            # scale from 0.0 at min_sensor_diff to 1.0 at FAR_THRESHOLD
            imbalance_urgency = min(1.0, (max_imbalance - self._min_sensor_diff) / (RadiozoaSensor.FAR_THRESHOLD - self._min_sensor_diff))
        else:
            imbalance_urgency = 0.0
        # proximity urgency: normalized from FAR_THRESHOLD (0.0) to 0mm (1.0)
        if min_distance < RadiozoaSensor.FAR_THRESHOLD:
            proximity_urgency = 1.0 - (min_distance / RadiozoaSensor.FAR_THRESHOLD)
        else:
            proximity_urgency = 0.0
        # priority formula: scales from 0.4 (gentle guidance) to 1.0 (critical avoidance)
        # imbalance contributes 0.0-0.2: centering force
        # proximity contributes 0.0-0.4: obstacle urgency
        self._priority = 0.4 + (imbalance_urgency * 0.2) + (proximity_urgency * 0.4)

        # apply output scaling to prevent overwhelming other behaviors
        vx_scaled = vx * amplitude * self._output_scale
        vy_scaled = vy * amplitude * self._output_scale

        if abs(vx * amplitude) > 0.4 or abs(vy * amplitude) > 0.4:
            self._log.warning('large intent vector: vx={:.3f}, vy={:.3f}, priority={:.3f}, max_imbal={:.1f}mm, min_dist={:.1f}mm'.format(
                vx * amplitude, vy * amplitude, self._priority, max_imbalance, min_distance))
        if self._verbose:
            self._display_info(vx * amplitude, vy * amplitude)
        return (vx * amplitude, vy * amplitude, 0.0)

    def _get_highlight_color(self, value):
        '''
        Return colorama color/style for multiplier legend.
        '''
        if value <= 0.1:
            return Style.DIM
        elif value <= 0.2:
            return Fore.BLUE
        elif value <= 0.3:
            return Fore.GREEN
        elif value <= 0.5:
            return Fore.YELLOW
        elif value <= 0.6:
            return Fore.RED
        else:  # 0.61 - 0.8
            return Fore.MAGENTA

    def _display_info(self, vx, vy, message=''):
        if self._use_color:
            self._log.info("{} intent vector: {}({:4.2f},{:4.2f}){}".format(
                    message,
                    self._get_highlight_color(vx),
                    vx, vy, Style.RESET_ALL
                )
            )
        else:
            self._log.info("intent vector: ({:.2f},{:.2f})".format(vx, vy))

#EOF
