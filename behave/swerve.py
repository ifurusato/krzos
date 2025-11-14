#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-07
# modified: 2025-11-09

import sys
import time
import itertools
import numpy as np
from math import isclose
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.event import Event
from behave.async_behaviour import AsyncBehaviour
from hardware.easing import Easing
from hardware.swerve_sensor import SwerveSensor
from hardware.motor_controller import MotorController

class Swerve(AsyncBehaviour):
    NAME = 'swerve'
    '''
    Implements lateral obstacle avoidance using VL53L5CX side columns.

    Swerve monitors the port and starboard edges of the VL53L5CX sensor's
    field of view and generates lateral intent vectors (vx) to nudge the
    robot away from close side obstacles. Note that if the robot encounters
    an obstacle that shows up on both port and starboard sides equally (an
    obstacle in front of the robot), the port and starboard forces will
    balance out and generate no lateral movement.

    Key features:
    - Dynamic reaction distance based on forward speed (faster = react sooner)
    - Dynamic lateral force scaling based on obstacle proximity
    - Dynamic priority (close obstacles = high priority, acts as virtual bumper)
    - No forward/rotation control - only lateral movement (vx)

    Works in coordination with:
    - Roam: provides forward motion (vy)
    - Scout/Radiozoa: provide steering (omega)
    '''
    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Swerve.NAME, level)
        _component_registry = Component.get_registry()
        _motor_controller = _component_registry.get(MotorController.NAME)
        if _motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        AsyncBehaviour.__init__(self, self._log, config, message_bus, message_factory, _motor_controller, level=level)
        # configuration
        _cfg = config['kros'].get('behaviour').get('swerve')
        self._counter = itertools.count()
        self._verbose = _cfg.get('verbose', False)
        # override poll delay of superclass
        self._poll_delay_ms   = _cfg.get('poll_delay_ms')
        self._poll_delay_sec  = self._poll_delay_ms / 1000.0
        self._log.info(Fore.WHITE + "poll delay: {}ms".format(self._poll_delay_ms))
        # distance thresholds (dynamic based on speed)
        self._min_reaction_distance = _cfg.get('min_reaction_distance', 150)  # mm at zero speed
        self._max_reaction_distance = _cfg.get('max_reaction_distance', 500)  # mm absolute max
        self._distance_per_speed = _cfg.get('distance_per_speed', 200)  # mm to add per 1.0 normalized speed
        # force and priority
        self._max_lateral_force = _cfg.get('max_lateral_force', 0.5)  # max vx magnitude
        '''
            LINEAR:              direct proportional
            SQUARE_ROOT:         maintains higher force longer, drops gradually
            QUADRATIC:           values drop quickly at max distance, flatten near min (very aggressive at close range)
            REVERSE_LOGARITHMIC: like Avoid's aft sensor, stays low until very close then spike
        '''
        self._max_vx_change = self._max_lateral_force / 3.0  # max change per poll, reach max in 3 polls
        self._use_rate_limiting = False
        self._last_vx = 0.0
        self._lateral_easing = Easing.from_string(_cfg.get('lateral_easing', 'SQUARE_ROOT'))
        self._log.info(Fore.WHITE + 'using {} lateral easing.'.format(self._lateral_easing.name))
        self._min_priority = _cfg.get('min_priority', 0.2)  # priority when far
        self._max_priority = _cfg.get('max_priority', 0.9)  # priority when very close
        self._deadband_threshold = _cfg.get('deadband_threshold', 0.05)  # ignore vx below this
        # state
        self._port_distance = None
        self._stbd_distance = None
        self._current_reaction_distance = self._min_reaction_distance
        self._current_priority = self._min_priority
        # swerve sensor
        self._swerve_sensor = _component_registry.get(SwerveSensor.NAME)
        if self._swerve_sensor is None:
            self._log.info('creating Swerve sensor…')
            self._swerve_sensor = SwerveSensor(config, level=Level.INFO)
        else:
            self._log.info('using existing Swerve sensor.')

        self._log.info('ready.')

    @property
    def name(self):
        return Swerve.NAME

    @property
    def is_ballistic(self):
        '''Not ballistic - works cooperatively with other behaviors.'''
        return False

    @property
    def priority(self):
        '''Dynamic priority based on obstacle proximity.'''
        return self._current_priority

    def callback(self):
        self._log.info('swerve behaviour callback.')
        raise Exception('UNSUPPORTED callback')

    def execute(self, message):
        print('execute message {}.'.format(message))
        raise Exception('UNSUPPORTED execute')

    def start_loop_action(self):
        '''Called when async loop starts.'''
        self._log.info('swerve loop started')

    def stop_loop_action(self):
        '''Called when async loop stops.'''
        self._log.info('swerve loop stopped')

    async def _poll(self):
        '''
        The asynchronous poll, updates the intent vector on each call.
        '''
        try:
            self._update_intent_vector()
        except Exception as e:
            self._log.error("{} thrown while polling: {}".format(type(e), e))
            self.disable()

    def _calculate_reaction_distance(self):
        '''
        Calculate dynamic reaction distance based on current forward speed.
        Faster forward motion = need to react to obstacles further away.

        Returns:
            float: Reaction distance in mm
        '''
        forward_velocity = abs(self._motor_controller.forward_velocity)  # 0.0 to 1.0

        # linear scaling: add distance proportional to speed
        dynamic_distance = self._min_reaction_distance + (forward_velocity * self._distance_per_speed)

        # clamp to max
        reaction_distance = min(dynamic_distance, self._max_reaction_distance)

        return reaction_distance

    def _calculate_lateral_force(self, obstacle_distance, reaction_distance):
        '''
        Calculate lateral force magnitude based on obstacle distance.
        Closer obstacles = stronger lateral push.
        Uses easing function for non-linear response.

        Args:
            obstacle_distance: Distance to obstacle in mm
            reaction_distance: Current reaction threshold in mm

        Returns:
            float: Lateral force magnitude (0.0 to max_lateral_force)
        '''
        if obstacle_distance is None or obstacle_distance >= reaction_distance:
            return 0.0
        # normalize: 1.0 when very close, 0.0 at reaction_distance
        normalised = 1.0 - (obstacle_distance / reaction_distance)
        normalised = np.clip(normalised, 0.0, 1.0)
        # apply easing function for non-linear scaling
        eased = self._lateral_easing.apply(normalised)

        # apply to max force
        force = self._max_lateral_force * eased
        return force

    def _calculate_priority(self, obstacle_distance, reaction_distance):
        '''
        Calculate dynamic priority based on obstacle proximity.
        Very close obstacles get high priority (acts as virtual bumper).

        Args:
            obstacle_distance: Distance to obstacle in mm
            reaction_distance: Current reaction threshold in mm

        Returns:
            float: Priority value (min_priority to max_priority)
        '''
        if obstacle_distance is None or obstacle_distance >= reaction_distance:
            return self._min_priority
        # normalize: 1.0 when very close, 0.0 when far
        normalized = 1.0 - (obstacle_distance / reaction_distance)
        normalized = np.clip(normalized, 0.0, 1.0)
        # scale between min and max priority
        priority = self._min_priority + (normalized * (self._max_priority - self._min_priority))
        return priority

    def _update_intent_vector(self):
        '''
        Update the intent vector based on side obstacle detection.

        Swerve only controls lateral motion (vx). No forward (vy) or rotation (omega).

        Robot-relative components of the vector:
            vx:    lateral velocity (positive = move right/starboard, negative = move left/port)
            vy:    longitudinal velocity (always 0.0 for Swerve)
            omega: angular velocity (always 0.0 for Swerve)
        '''
        if self._motor_controller.braking_active:
            self._log.debug('braking active: intent vector suppressed')
            self.clear_intent_vector()
            return
        # get current side distances
        self._port_distance, self._stbd_distance = self._swerve_sensor.get_side_distances()
        # calculate dynamic reaction distance based on forward speed
        self._current_reaction_distance = self._calculate_reaction_distance()
        # calculate lateral forces for each side
        port_force = self._calculate_lateral_force(self._port_distance, self._current_reaction_distance)
        stbd_force = self._calculate_lateral_force(self._stbd_distance, self._current_reaction_distance)
        # determine net lateral motion
        # port obstacle detected: move starboard (positive vx), stbd obstacle detected: move port (negative vx)
        vx = port_force - stbd_force
        # rate limiting
        vx_delta = vx - self._last_vx
        if self._use_rate_limiting and (abs(vx_delta) > self._max_vx_change):
            vx = self._last_vx + (self._max_vx_change if vx_delta > 0 else -self._max_vx_change)
            self._log.warning('⚠️  rate limiting vx: delta={:.3f}, clamped to {:.3f}'.format(vx_delta, vx))
        self._last_vx = vx
        # calculate priority based on closest obstacle
        closest_distance = None
        if self._port_distance is not None and self._stbd_distance is not None:
            closest_distance = min(self._port_distance, self._stbd_distance)
        elif self._port_distance is not None:
            closest_distance = self._port_distance
        elif self._stbd_distance is not None:
            closest_distance = self._stbd_distance
        self._current_priority = self._calculate_priority(closest_distance, self._current_reaction_distance)
        # apply deadband
        _within_deadband = abs(vx) < self._deadband_threshold
        if _within_deadband:
            self.clear_intent_vector()
            self._last_vx = 0.0
        else:
            vy = 0.0     # no forward control
            omega = 0.0  # no rotation control
            self.set_intent_vector(vx, vy, omega)
        # logging
        if self._verbose or (next(self._counter) % 20 == 0):
            if _within_deadband:
#               self._log.debug(Style.DIM + 'port: {}mm, stbd: {}mm, react: {:.0f}mm, vx: {:.3f}, priority: {:.2f} (deadband)'.format(
#                   '{:.0f}'.format(self._port_distance) if self._port_distance else 'None',
#                   '{:.0f}'.format(self._stbd_distance) if self._stbd_distance else 'None',
#                   self._current_reaction_distance, vx, self._current_priority))
                pass
            else:
                self._log.info('port: {}mm, stbd: {}mm, react: {:.0f}mm, vx: {:.3f}, priority: {:.2f}'.format(
                    '{:.0f}'.format(self._port_distance) if self._port_distance else 'None',
                    '{:.0f}'.format(self._stbd_distance) if self._stbd_distance else 'None',
                    self._current_reaction_distance, vx, self._current_priority))

    def enable(self):
        if self.enabled:
            self._log.debug("already enabled.")
            return
        self._log.info("enabling swerve…")
        if not self._swerve_sensor.enabled:
            self._swerve_sensor.enable()
        AsyncBehaviour.enable(self)
        self._log.info("swerve enabled.")

    def disable(self):
        if not self.enabled:
            self._log.debug("already disabled.")
            return
        self._log.info("disabling swerve…")
        self.clear_intent_vector()
        AsyncBehaviour.disable(self)
        self._log.info('disabled.')

#EOF
