#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-20
# modified: 2025-12-06

import math
import itertools
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.event import Event
from behave.async_behaviour import AsyncBehaviour
from hardware.easing import Easing
from hardware.motor_controller import MotorController

class Geofence(AsyncBehaviour):
    NAME = 'geofence'
    '''
    Virtual boundary fence that keeps the robot within a defined rectangular area.

    Uses a three-stage state machine to manage boundary interactions:
    - normal:   robot far from boundaries, no intervention
    - warning:  approaching boundary, throttle Roam and apply gentle forces
    - critical: very close to boundary, strong rotation to escape

    This orchestrates with Roam (throttling forward speed) and provides active
    rotation (omega) to turn the robot away from boundaries.
    '''
    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Geofence.NAME, level)
        _component_registry = Component.get_registry()
        _motor_controller = _component_registry.get(MotorController.NAME)
        if _motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        AsyncBehaviour.__init__(self, self._log, config, message_bus, message_factory, _motor_controller, level=level)

        # read configuration
        _cfg = config['kros'].get('behaviour').get('geofence')

        # boundary definition (in cm)
        self._min_x = _cfg.get('min_x', -200.0)
        self._max_x = _cfg.get('max_x', 200.0)
        self._min_y = _cfg.get('min_y', -200.0)
        self._max_y = _cfg.get('max_y', 200.0)

        # initial robot pose
        initial_x = _cfg.get('initial_x', 0.0)
        initial_y = _cfg.get('initial_y', 0.0)
        initial_theta_deg = _cfg.get('initial_theta', 0.0)
        initial_theta = math.radians(initial_theta_deg)

        # behavior parameters
        self._margin = _cfg.get('margin', 50.0)
        self._max_lateral_force = _cfg.get('max_lateral_force', 0.8)
        self._max_rotation_force = _cfg.get('max_rotation_force', 0.5)
        self._escape_rotation_speed = _cfg.get('escape_rotation_speed', 0.6)
        self._escape_priority = _cfg.get('escape_priority', 0.95)
        self._easing = Easing.from_string(_cfg.get('easing', 'SQUARE_ROOT'))
        self._min_priority = _cfg.get('min_priority', 0.3)
        self._max_priority = _cfg.get('max_priority', 0.8)
        self._use_dynamic_priority = True
        self._verbose = _cfg.get('verbose', False)

        # state machine thresholds
        self._warning_threshold = 0.3
        self._critical_threshold = 0.7
        self._clear_threshold = 0.2

        # state
        self._counter = itertools.count()
        self._current_priority = self._min_priority
        self._escape_mode = 'normal'
        self._target_escape_heading = None

        # get odometer and set initial pose
        self._odometer = self._motor_controller.get_odometer()
        if self._odometer is None:
            raise MissingComponentError('odometer not available from motor controller.')

        self._odometer.set_pose(initial_x, initial_y, initial_theta)
        self._log.info('set initial pose: x={:.1f}cm, y={:.1f}cm, theta={:.1f}° ({:.2f}rad)'.format(
            initial_x, initial_y, initial_theta_deg, initial_theta))

        self._log.info('boundary: x=[{:.1f}, {:.1f}]cm, y=[{:.1f}, {:.1f}]cm, margin: {:.1f}cm'.format(
            self._min_x, self._max_x, self._min_y, self._max_y, self._margin))
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def is_ballistic(self):
        return False

    @property
    def priority(self):
        return self._current_priority

    def execute(self, message):
        raise NotImplementedError('execute unsupported in Geofence.')

    def start_loop_action(self):
        self._log.info('geofence active')

    def stop_loop_action(self):
        self._log.info('geofence inactive')

    async def _poll(self):
        '''
        Check current position and orchestrate escape behavior if near boundaries.
        Returns (vx, vy, omega) tuple.
        '''
        if self.suppressed:
            return
        print('geofence: _poll')
        try:
            x, y, theta = self._odometer.pose
            urgency, world_fx, world_fy = self._calculate_boundary_state(x, y)
            self._update_escape_state(urgency, world_fx, world_fy, theta)
            return self._generate_intent_vector(urgency, world_fx, world_fy, theta)
        except Exception as e:
            self._log.error('{} thrown while polling: {}'.format(type(e), e))
            self.disable()
            return (0.0, 0.0, 0.0)

    def _calculate_boundary_state(self, x, y):
        '''
        Calculate urgency and world-frame repulsion forces for all boundaries.
        Returns (max_urgency, world_fx, world_fy).
        '''
        world_fx = 0.0
        world_fy = 0.0
        max_urgency = 0.0

        # X boundaries (east/west)
        if x < (self._min_x + self._margin):
            distance_from_edge = max(0.0, x - self._min_x)
            if distance_from_edge == 0.0:
                self._log.warning('outside west boundary: x={:.1f}cm'.format(x))
            normalised_distance = distance_from_edge / self._margin
            urgency = 1.0 - normalised_distance
            max_urgency = max(max_urgency, urgency)
            world_fx += 1.0

        elif x > (self._max_x - self._margin):
            distance_from_edge = max(0.0, self._max_x - x)
            if distance_from_edge == 0.0:
                self._log.warning('outside east boundary: x={:.1f}cm'.format(x))
            normalised_distance = distance_from_edge / self._margin
            urgency = 1.0 - normalised_distance
            max_urgency = max(max_urgency, urgency)
            world_fx -= 1.0

        # Y boundaries (north/south)
        if y < (self._min_y + self._margin):
            distance_from_edge = max(0.0, y - self._min_y)
            if distance_from_edge == 0.0:
                self._log.warning('outside south boundary: y={:.1f}cm'.format(y))
            normalised_distance = distance_from_edge / self._margin
            urgency = 1.0 - normalised_distance
            max_urgency = max(max_urgency, urgency)
            world_fy += 1.0

        elif y > (self._max_y - self._margin):
            distance_from_edge = max(0.0, self._max_y - y)
            if distance_from_edge == 0.0:
                self._log.warning('outside north boundary: y={:.1f}cm'.format(y))
            normalised_distance = distance_from_edge / self._margin
            urgency = 1.0 - normalised_distance
            max_urgency = max(max_urgency, urgency)
            world_fy -= 1.0

        # normalize force direction if corner (both axes active)
        if world_fx != 0.0 and world_fy != 0.0:
            magnitude = math.sqrt(world_fx**2 + world_fy**2)
            world_fx /= magnitude
            world_fy /= magnitude

        return max_urgency, world_fx, world_fy

    def _update_escape_state(self, urgency, world_fx, world_fy, theta):
        '''
        State machine: normal → warning → critical → normal.
        Adjusts Roam throttle and calculates escape heading.
        '''
        _last_mode = self._escape_mode

        if urgency > self._critical_threshold:
            self._escape_mode = 'critical'
            if world_fx != 0.0 or world_fy != 0.0:
                self._target_escape_heading = math.atan2(world_fy, world_fx)

        elif urgency > self._warning_threshold:
            self._escape_mode = 'warning'

        elif urgency < self._clear_threshold:
            self._escape_mode = 'normal'
            self._target_escape_heading = None

        # log state transitions
        if _last_mode != self._escape_mode:
            self._log.warning('geofence mode: {} → {} (urgency={:.2f})'.format(
                _last_mode, self._escape_mode, urgency))

        # adjust Roam throttle based on state
        if self._escape_mode == 'critical':
            roam_multiplier = 0.1
        elif self._escape_mode == 'warning':
            range_size = self._critical_threshold - self._warning_threshold
            roam_multiplier = 1.0 - ((urgency - self._warning_threshold) / range_size * 0.9)
        else:
            roam_multiplier = 1.0

        self._motor_controller.set_behavior_speed_multiplier('roam', roam_multiplier)

    def _generate_intent_vector(self, urgency, world_fx, world_fy, theta):
        '''
        Generate (vx, vy, omega) based on escape state.
        '''
        if self._escape_mode == 'critical':
            if self._target_escape_heading is not None:
                heading_error = self._target_escape_heading - theta
                while heading_error > math.pi:
                    heading_error -= 2 * math.pi
                while heading_error < -math.pi:
                    heading_error += 2 * math.pi

                omega = self._escape_rotation_speed * math.copysign(1.0, heading_error)

                cos_theta = math.cos(theta)
                sin_theta = math.sin(theta)
                vx = world_fx * cos_theta + world_fy * sin_theta
                vy = -world_fx * sin_theta + world_fy * cos_theta

                force_scale = urgency ** 2
                vx *= force_scale * self._max_lateral_force * 0.5
                vy *= force_scale * self._max_lateral_force * 0.5

                self._current_priority = self._escape_priority

                self._log.info('CRITICAL ESCAPE: rotating to {:.1f}°, error={:.1f}°, omega={:.2f}'.format(
                    math.degrees(self._target_escape_heading),
                    math.degrees(heading_error),
                    omega))

                return (vx, vy, omega)

        elif self._escape_mode == 'warning':
            cos_theta = math.cos(theta)
            sin_theta = math.sin(theta)
            vx = world_fx * cos_theta + world_fy * sin_theta
            vy = -world_fx * sin_theta + world_fy * cos_theta

            if world_fx != 0.0 or world_fy != 0.0:
                desired_heading = math.atan2(world_fy, world_fx)
                heading_error = desired_heading - theta
                while heading_error > math.pi:
                    heading_error -= 2 * math.pi
                while heading_error < -math.pi:
                    heading_error += 2 * math.pi
                omega = self._max_rotation_force * urgency * math.copysign(1.0, heading_error)
            else:
                omega = 0.0

            force_scale = urgency ** 2
            vx *= force_scale * self._max_lateral_force
            vy *= force_scale * self._max_lateral_force

            self._current_priority = 0.3 + (urgency * 0.5)

            return (vx, vy, omega)

        else:
            self._current_priority = 0.3
            return (0.0, 0.0, 0.0)

    def enable(self):
        if not self.enabled:
            super().enable()
            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def disable(self):
        if self.enabled:
            super().disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

#EOF
