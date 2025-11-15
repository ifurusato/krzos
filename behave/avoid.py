#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-01
# modified: 2025-11-13

import time
import traceback
import numpy as np
import itertools
from math import isclose
from threading import Thread, Event as ThreadEvent
import asyncio
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.orientation import Orientation
from core.event import Event
from behave.async_behaviour import AsyncBehaviour
from hardware.easing import Easing
from hardware.motor_controller import MotorController
from hardware.avoid_sensor import AvoidSensor

class Avoid(AsyncBehaviour):
    NAME = 'avoid'
    '''
    An avoidance behaviour that uses the side and aft sensors to force the
    robot away from obstacles.
    '''
    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Avoid.NAME, level)
        _component_registry = Component.get_registry()
        _motor_controller = _component_registry.get(MotorController.NAME)
        if _motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        AsyncBehaviour.__init__(self, self._log, config, message_bus, message_factory, _motor_controller, level=level)
        self.add_event(Event.AVOID)
        # configuration
        _cfg = config['kros'].get('behaviour').get('avoid')
        self._side_threshold_mm = _cfg.get('side_threshold_mm', 280)
        self._aft_threshold_mm  = _cfg.get('aft_threshold_mm', 500)
        self._avoid_speed = _cfg.get('avoid_speed', 1.0)
        self._log.info('side threshold: {}mm; aft threshold: {}mm; avoid speed: {:4.2f}'.format(
                self._side_threshold_mm, self._aft_threshold_mm, self._avoid_speed))
        self._side_easing = Easing.from_string(_cfg.get('side_easing', 'SQUARE_ROOT'))
        self._aft_easing  = Easing.from_string(_cfg.get('aft_easing', 'REVERSE_LOGARITHMIC'))
        self._use_aft_sensor = _cfg.get('use_aft_sensor', True)
        self._log.info('side easing: {}; aft easing: {}; use aft sensor: {}'.format(self._side_easing.name, self._aft_easing.name, self._use_aft_sensor))
        self._max_urgency = _cfg.get('max_urgency', 0.7)
        self._port_sensor = AvoidSensor(config, Orientation.PORT, level=level)
        self._stbd_sensor = AvoidSensor(config, Orientation.STBD, level=level)
        if self._use_aft_sensor:
            self._aft_sensor = AvoidSensor(config, Orientation.AFT, level=level)
        else:
            self._aft_sensor = None # explicitly set to None if not used
        self._boost_when_squeezed  = _cfg.get('boost_when_squeezed', True)
        self._use_dynamic_priority = _cfg.get('use_dynamic_priority', True)
        self._default_priority     = _cfg.get('default_priority', 0.3)
        self._priority             = self._default_priority
        self._verbose     = _cfg.get('verbose', False)
        # variables
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._max_vx_change = 0.05  # max change per iteration
        self._max_vy_change = 0.05
        self._squeezed    = False
        self._log.info('ready.')

    @property
    def name(self):
        return Avoid.NAME

    @property
    def is_ballistic(self):
        return False

    @property
    def priority(self):
        '''
        Returns the current priority, dynamically modified by distance if
        the dynamic priority flag is true, otherwise the default value.
        '''
        if self._use_dynamic_priority:
            return self._priority
        else:
            return self._default_priority

    def execute(self, message):
        self._log.info('execute message {}.'.format(message))
        pass

    def start_loop_action(self):
        self._log.info('start loop action.')
        pass

    def stop_loop_action(self):
        self._log.info('stop loop action.')
        pass

    async def _poll(self):
        try:
            # poll sensors and set intent vector accordingly
            _port_distance = self._port_sensor.get_distance()
            _stbd_distance = self._stbd_sensor.get_distance()
            _aft_distance = self._aft_sensor.get_distance() if self._use_aft_sensor else None
            # then set intent vector accordingly
            self._update_intent_vector(_port_distance, _stbd_distance, _aft_distance)
        except Exception as e:
            self._log.error("{} thrown while polling: {}\n{}".format(type(e), e, traceback.format_exc()))
            self.disable()

    def _update_intent_vector(self, port_distance, stbd_distance, aft_distance):
        '''
        Compute the robot's movement intent as a single (vx, vy, omega) vector
        based on the three distance sensors.

        Port sensor pushes robot to starboard (positive vx).
        Starboard sensor pushes robot to port (negative vx).
        Aft sensor pushes robot forward (positive vy) when obstacle detected behind.

        Priority scales continuously with obstacle proximity. When squeezed (both
        port and starboard active), priority is boosted proportionally to ensure
        lateral balancing isn't overwhelmed by other behaviors' forward motion.

        Uses easing functions to scale avoidance force with distance.
        '''
        vx = 0.0
        vy = 0.0
        omega = 0.0
        # track normalized sensor urgencies for priority calculation
        port_urgency = 0.0
        stbd_urgency = 0.0
        aft_urgency  = 0.0
        port_active  = False
        stbd_active  = False
        # port sensor: obstacle closer → push to starboard (positive vx)
        if port_distance is not None and port_distance < self._side_threshold_mm:
            normalised = 1.0 - (port_distance / self._side_threshold_mm)
            port_scale = self._side_easing.apply(normalised)
            vx += port_scale * self._avoid_speed
            port_urgency = port_scale
            port_active = True
        # starboard sensor: obstacle closer → push to port (negative vx)
        if stbd_distance is not None and stbd_distance < self._side_threshold_mm:
            normalised = 1.0 - (stbd_distance / self._side_threshold_mm)
            stbd_scale = self._side_easing.apply(normalised)
            vx -= stbd_scale * self._avoid_speed
            stbd_urgency = stbd_scale
            stbd_active = True
        # squeeze detection
        self._squeezed = port_active and stbd_active
        # aft sensor: obstacle closer → push forward (positive vy)
        if aft_distance is not None and aft_distance < self._aft_threshold_mm:
            normalised = 1.0 - (aft_distance / self._aft_threshold_mm)
            aft_scale = self._aft_easing.apply(normalised)
            if aft_scale > 0.05: # ignore weak signals below 5%
                vy += aft_scale * self._avoid_speed
                aft_urgency = aft_scale
        # calculate priority from maximum sensor urgency
        # base priority scales from 0.3 (no obstacles) to 1.0 (collision imminent)
        max_urgency = max(port_urgency, stbd_urgency, aft_urgency)
        self._priority = 0.3 + (max_urgency * 0.7)
        self._priority = min(self._max_urgency, self._priority) # clamp to maximum
        # squeeze boost: ensure lateral balancing dominates when threading narrow gaps
        if self._squeezed:
            if self._boost_when_squeezed:
                # squeeze severity based on tightest constraint (minimum of side urgencies)
                squeeze_severity = min(port_urgency, stbd_urgency)
                # boost priority by up to 0.15 based on squeeze tightness
                squeeze_boost = squeeze_severity * 0.15
                self._priority = min(1.0, self._priority + squeeze_boost)
                self._log.info('squeezed; boosted priority: {:4.2f}'.format(self._priority))
            else:
                self._log.debug('squeezed; priority: {:4.2f}'.format(self._priority))
        self._log.debug('intent vector: vx={:.3f}, vy={:.3f}, omega={:.3f}'.format(vx, vy, omega))
        self.set_intent_vector(vx, vy, omega)
        if True or self._verbose:
            self._print_info(port_distance, stbd_distance, aft_distance, vx, vy, omega)

    def x_update_intent_vector(self, port_distance, stbd_distance, aft_distance):
        '''
        Compute the robot's movement intent as a single (vx, vy, omega) vector
        based on the three distance sensors.

        Port sensor pushes robot to starboard (positive vx).
        Starboard sensor pushes robot to port (negative vx).
        Aft sensor pushes robot forward (positive vy) when obstacle detected behind.

        Priority scales continuously with obstacle proximity. When squeezed (both
        port and starboard active), priority is boosted proportionally to ensure
        lateral balancing isn't overwhelmed by other behaviors' forward motion.

        Uses easing functions to scale avoidance force with distance.
        '''
        vx = 0.0
        vy = 0.0
        omega = 0.0
        # track normalized sensor urgencies for priority calculation
        port_urgency = 0.0
        stbd_urgency = 0.0
        aft_urgency  = 0.0
        port_active  = False
        stbd_active  = False
        # port sensor: obstacle closer → push to starboard (positive vx)
        if port_distance is not None and port_distance < self._side_threshold_mm:
            normalised = 1.0 - (port_distance / self._side_threshold_mm)
            port_scale = self._side_easing.apply(normalised)
            vx += port_scale * self._avoid_speed
            port_urgency = port_scale
            port_active = True
        # starboard sensor: obstacle closer → push to port (negative vx)
        if stbd_distance is not None and stbd_distance < self._side_threshold_mm:
            normalised = 1.0 - (stbd_distance / self._side_threshold_mm)
            stbd_scale = self._side_easing.apply(normalised)
            vx -= stbd_scale * self._avoid_speed
            stbd_urgency = stbd_scale
            stbd_active = True
        # squeeze detection
        self._squeezed = port_active and stbd_active
        # aft sensor: obstacle closer → push forward (positive vy)
        if aft_distance is not None and aft_distance < self._aft_threshold_mm:
            normalised = 1.0 - (aft_distance / self._aft_threshold_mm)
            aft_scale = self._aft_easing.apply(normalised)
            if aft_scale > 0.05: # ignore weak signals below 5%
                vy += aft_scale * self._avoid_speed
                aft_urgency = aft_scale
        # calculate priority from maximum sensor urgency
        # base priority scales from 0.3 (no obstacles) to 1.0 (collision imminent)
        max_urgency = max(port_urgency, stbd_urgency, aft_urgency)
        self._priority = 0.3 + (max_urgency * 0.7)
        self._priority = min(self._max_urgency, self._priority) # clamp to maximum
        # squeeze boost: ensure lateral balancing dominates when threading narrow gaps
        if self._squeezed:
            if self._boost_when_squeezed:
                # squeeze severity based on tightest constraint (minimum of side urgencies)
                squeeze_severity = min(port_urgency, stbd_urgency)
                # boost priority by up to 0.15 based on squeeze tightness
                squeeze_boost = squeeze_severity * 0.15
                self._priority = min(1.0, self._priority + squeeze_boost)
                self._log.info('squeezed; boosted priority: {:4.2f}'.format(self._priority))
            else:
                self._log.debug('squeezed; priority: {:4.2f}'.format(self._priority))
        self._rate_limiting = True # TODO config
        if self._rate_limiting:
            # rate limit vx and vy changes to prevent sudden motor target swings
            vx_delta = vx - self._last_vx
            if abs(vx_delta) > self._max_vx_change:
                vx = self._last_vx + (self._max_vx_change if vx_delta > 0 else -self._max_vx_change)
            vy_delta = vy - self._last_vy
            if abs(vy_delta) > self._max_vy_change:
                vy = self._last_vy + (self._max_vy_change if vy_delta > 0 else -self._max_vy_change)
            self._last_vx = vx
            self._last_vy = vy
        self._log.debug('intent vector: vx={:.3f}, vy={:.3f}, omega={:.3f}'.format(vx, vy, omega))
        self.set_intent_vector(vx, vy, omega)
        if True or self._verbose:
            self._print_info(port_distance, stbd_distance, aft_distance, vx, vy, omega)

    def _print_info(self, port_distance, stbd_distance, aft_distance, vx, vy, omega):
        '''
        Display current sensor readings, intent vector, and priority with color coding.
        '''
        if port_distance and port_distance < self._side_threshold_mm:
            _port_color = Fore.RED + Style.NORMAL
        else:
            _port_color = Fore.RED + Style.DIM
        if stbd_distance and stbd_distance < self._side_threshold_mm:
            _stbd_color = Fore.GREEN + Style.NORMAL
        else:
            _stbd_color = Fore.GREEN + Style.DIM
        if aft_distance and aft_distance < self._aft_threshold_mm:
            _aft_color = Fore.YELLOW + Style.NORMAL
        else:
            _aft_color = Fore.YELLOW + Style.DIM
        _squeeze_indicator = Fore.MAGENTA + ' [SQUEEZED]' + Style.RESET_ALL if self._squeezed else ''
        # priority color coding
        if self._priority >= 0.9:
            _priority_color = Fore.RED + Style.BRIGHT
        elif self._priority >= 0.7:
            _priority_color = Fore.YELLOW + Style.BRIGHT
        else:
            _priority_color = Fore.CYAN + Style.DIM
        if self._verbose:
            self._log.info("intent: ({:5.2f}, {:5.2f}, {:5.2f}); {}priority: {:.2f};{} port: {};{} stbd: {};{} aft: {}{}".format(
                    vx, vy, omega,
                    _priority_color, self._priority, Style.RESET_ALL,
                    _port_color,
                    '{}mm'.format(port_distance) if port_distance else 'NA',
                    _stbd_color,
                    '{}mm'.format(stbd_distance) if stbd_distance else 'NA',
                    _aft_color,
                    '{}mm'.format(aft_distance) if aft_distance else 'NA',
                    _squeeze_indicator))

    def enable(self):
        if self.enabled:
            self._log.debug('already enabled.')
            return
        self._port_sensor.enable()
        self._stbd_sensor.enable()
        if self._use_aft_sensor and self._aft_sensor:
            self._aft_sensor.enable()
        AsyncBehaviour.enable(self)
        self._log.info('enabled.')

    def disable(self):
        if not self.enabled:
            self._log.debug('already disabled.')
            return
        self._port_sensor.disable()
        self._stbd_sensor.disable()
        if self._use_aft_sensor and self._aft_sensor:
            self._aft_sensor.disable()
        AsyncBehaviour.disable(self)
        self._log.info('disabled.')

#EOF
