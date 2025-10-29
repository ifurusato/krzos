#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-10
# modified: 2025-10-30

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
#       Behaviour.__init__(self, self._log, config, message_bus, message_factory, suppressed=True, enabled=False, level=level)
        AsyncBehaviour.__init__(self, self._log, config, message_bus, message_factory, _motor_controller, level=level)
        self.add_event(Event.AVOID)
        # configuration
        _cfg = config['kros'].get('behaviour').get('radiozoa')
        self._loop_delay_ms = _cfg.get('loop_delay_ms', 50)
        self._counter    = itertools.count()
        _default_speed   = _cfg.get('default_speed', 1.0)
        _dynamic_speed   = _cfg.get('dynamic_speed')
        self._verbose    = False
        self._use_color  = True
#       self._loop_instance  = None
#       self._thread     = None
#       self._stop_event = ThreadEvent()
        # intent vector for MotorController
        # directional vectors
        self._pairs = [
            (Cardinal.NORTH, Cardinal.SOUTH),
            (Cardinal.NORTHWEST, Cardinal.SOUTHEAST),
            (Cardinal.WEST, Cardinal.EAST),
            (Cardinal.NORTHEAST, Cardinal.SOUTHWEST),
        ]

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
        _smoothing_factor = _cfg.get('smoothing_factor', 0.3)             # 0.0=no smoothing, 1.0=no response
        self._smoothing_factor = max(0.0, min(1.0, _smoothing_factor))
        self._min_sensor_diff = _cfg.get('min_sensor_diff_mm', 50)        # minimum difference to react
        self._settling_threshold = _cfg.get('settling_threshold_mm', 30)  # all pairs within this = settled
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
        if _dynamic_speed:
            self._default_speed = 0.0
            self._digital_pot = _component_registry.get(DigitalPotentiometer.NAME)
        else:
            self._default_speed = _default_speed
        self._radiozoa_sensor = _component_registry.get(RadiozoaSensor.NAME)
        if self._radiozoa_sensor is None:
            self._log.info(Fore.WHITE + 'creating Radiozoa sensor…')
            self._radiozoa_sensor = RadiozoaSensor(config, level=Level.INFO)
        else:
            self._log.info(Fore.WHITE + 'using existing Radiozoa sensor.')
        self._intent_vector_registered = False
#       self._register_intent_vector()
        self._log.info('ready.')

    @property
    def name(self):
        return Radiozoa.NAME

    @property
    def is_ballistic(self):
        return False

    def execute(self, message):
        print('execute message {}.'.format(message))
        raise Exception('UNSUPPORTED execute') # TEMP
#       if self.suppressed:
#           self._log.info(Style.DIM + 'radiozoa suppressed; message: {}'.format(message.event.label))
#       else:
#           self._log.info('radiozoa execute; message: {}'.format(message.event.label))
#           _payload = message.payload
#           _event = _payload.event
#           _timestamp = self._message_bus.last_message_timestamp
#           if _timestamp is None:
#               self._log.info('radiozoa loop execute; no previous messages.')
#           else:
#               _elapsed_ms = (time.time() - _timestamp.timestamp()) * 1000.0
#               self._log.info('radiozoa loop execute; message age: {:7.2f} ms'.format(_elapsed_ms))
#           if self.enabled:
#               self._log.info('radiozoa enabled, execution on message {}; '.format(message.name) + Fore.YELLOW + ' event: {};'.format(_event.label))
#           else:
#               self._log.info('radiozoa disabled, execution on message {}; '.format(message.name) + Fore.YELLOW + ' event: {};'.format(_event.label))

    def _accelerate(self):
        self._log.info("accelerate…")
        if self._motor_controller:
            self._motor_controller.accelerate(self._default_speed, enabled=lambda: not self._stop_event.is_set())

    def _decelerate(self):
        self._log.info("decelerate…")
        if self._motor_controller:
            self._motor_controller.decelerate(0.0, enabled=lambda: not self._stop_event.is_set())

    def start_loop_action(self):
        self._accelerate()

    def stop_loop_action(self):
        self._decelerate()

    def _dynamic_set_default_speed(self):
        _speed = self._digital_pot.get_scaled_value(False) # values 0.0-1.0
        if isclose(_speed, 0.0, abs_tol=0.08):
            self._digital_pot.set_black() # only on digital pot
            self._default_speed = 0.0
            self._log.info(Fore.BLACK + "default speed: stopped")
        else:
            self._digital_pot.set_rgb(self._digital_pot.value)
            self._default_speed = _speed
            self._log.info(Fore.BLUE + "set default speed: {:4.2f}".format(self._default_speed))

    async def _poll(self):
        try:
            if next(self._counter) % 5 == 0:
                self._dynamic_set_default_speed()
            distances = self._radiozoa_sensor.get_distances()
            if not distances or all(d is None or d > RadiozoaSensor.FAR_THRESHOLD for d in distances):
                # stop when sensors are unavailable or out of range.
                self.clear_intent_vector()
            else:
                self._update_intent_vector(distances)
        except Exception as e:
            self._log.error("{} thrown while polling: {}".format(type(e), e))
            self.disable()


    def _update_intent_vector(self, distances):
        '''
        Compute the robot's movement intent as a single (vx, vy, omega) vector.
        Each pair contributes force only if its imbalance exceeds min_sensor_diff_mm.
        Robot settles naturally when all pairs are balanced within threshold.
        '''
        far_threshold = RadiozoaSensor.FAR_THRESHOLD * 0.95
        force_vec = np.zeros(2)
        pair_active = False
        for c1, c2 in self._pairs:
            d1 = self._radiozoa_sensor.get_sensor_by_cardinal(c1).get_distance()
            d2 = self._radiozoa_sensor.get_sensor_by_cardinal(c2).get_distance()
            d1 = d1 if d1 is not None and d1 > 0 else RadiozoaSensor.FAR_THRESHOLD
            d2 = d2 if d2 is not None and d2 > 0 else RadiozoaSensor.FAR_THRESHOLD
            # both sensors out of range - ignore this pair
            if d1 >= far_threshold and d2 >= far_threshold:
                continue
            diff = abs(d1 - d2)
            # only contribute force if difference exceeds threshold
            if diff >= self._min_sensor_diff:
                signed_diff = d1 - d2
                vec = self._directions[(c1, c2)] * signed_diff
                force_vec += vec
                pair_active = True
        # if no pairs are contributing force, we're settled
        if not pair_active:
            self._intent_vector = (0.0, 0.0, 0.0)
            self._smoothed_vector = np.array([0.0, 0.0])
            return
        # normalize the raw force vector
        max_abs = np.max(np.abs(force_vec)) if np.max(np.abs(force_vec)) > 1.0 else 1.0
        normalized_vec = force_vec / max_abs
        # apply exponential moving average for smoothing
        self._smoothed_vector = (self._smoothing_factor * self._smoothed_vector +
                                 (1.0 - self._smoothing_factor) * normalized_vec)
        vx, vy = self._smoothed_vector
        amplitude = self._default_speed
        self._intent_vector = (vx * amplitude, vy * amplitude, 0.0)
        if self._verbose:
            self._display_info()

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

    def _display_info(self, message=''):
        if self._use_color:
            self._log.info("{} intent vector: {}({:4.2f},{:4.2f}){}".format(
                    message,
                    self._get_highlight_color(self._intent_vector[0]),
                    self._intent_vector[0], self._intent_vector[1], Style.RESET_ALL
                )
            )
        else:
            self._log.info("intent vector: ({:.2f},{:.2f})".format(
                    self._intent_vector[0], self._intent_vector[1]
                )
            )

#EOF
