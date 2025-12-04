#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-20
# modified: 2025-11-20

import math
import numpy as np
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

    Unlike simple lateral avoidance, this behavior generates repulsion forces that
    reflect the robot's approach angle to the boundary - like bouncing off a wall.
    This produces more natural boundary interactions with both lateral motion (vx)
    and rotation (omega) to turn away from the boundary.
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
        self._min_x = _cfg.get('min_x', 0.0)
        self._max_x = _cfg.get('max_x', 300.0)
        self._min_y = _cfg.get('min_y', 0.0)
        self._max_y = _cfg.get('max_y', 400.0)

        # initial robot pose
        initial_x = _cfg.get('initial_x', 0.0)
        initial_y = _cfg.get('initial_y', 0.0)
        initial_theta_deg = _cfg.get('initial_theta', 0.0)  # degrees from config
        initial_theta = math.radians(initial_theta_deg)     # convert to radians

        # behavior parameters
        self._margin = _cfg.get('margin', 50.0)
        self._max_lateral_force = _cfg.get('max_lateral_force', 0.8)
        self._max_rotation_force = _cfg.get('max_rotation_force', 0.5)
        self._easing = Easing.from_string(_cfg.get('easing', 'SQUARE_ROOT'))
        self._min_priority = _cfg.get('min_priority', 0.3)
        self._max_priority = _cfg.get('max_priority', 0.95)
        self._use_dynamic_priority = True
        self._verbose = _cfg.get('verbose', False)

        # state
        self._counter = itertools.count()
        self._current_priority = self._min_priority
        self._at_boundary = False # track if we're currently at a boundary

        # get odometer and set initial pose
        self._odometer = self._motor_controller.get_odometer()
        if self._odometer is None:
            raise MissingComponentError('odometer not available from motor controller.')

        # set the odometer's initial pose
        self._odometer.set_pose(initial_x, initial_y, initial_theta)
        self._log.info('set initial pose: x={:.1f}cm, y={:.1f}cm, theta={:.1f}° ({:.2f}rad)'.format(
            initial_x, initial_y, initial_theta_deg, initial_theta))

        self._log.info('boundary: x=[{:.1f}, {:.1f}]cm, y=[{:.1f}, {:.1f}]cm, margin: {:.1f}cm'.format(
            self._min_x, self._max_x, self._min_y, self._max_y, self._margin))
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

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
        Check current position and generate repulsion forces if near boundaries.
        Returns (vx, vy, omega) tuple.
        '''
        try:
            x, y, theta = self._odometer.pose
            return self._update_intent_vector(x, y, theta)
        except Exception as e:
            self._log.error("{} thrown while polling: {}".format(type(e), e))
            self.disable()
            return (0.0, 0.0, 0.0)

    def _update_intent_vector(self, x, y, theta):
        '''
        Calculate repulsion forces based on proximity to boundaries.
        Converts world-frame boundary repulsion into robot-frame intent vector.
        '''
        # World-frame repulsion forces
        world_fx = 0.0  # force in world x-direction (east/west)
        world_fy = 0.0  # force in world y-direction (north/south)
        max_urgency = 0.0

        # log current position and distances
        dist_to_west = x - self._min_x
        dist_to_east = self._max_x - x
        dist_to_south = y - self._min_y
        dist_to_north = self._max_y - y

        if next(self._counter) % 20 == 0:
            self._log.info('pose: ({:.1f}, {:.1f})cm @ {:.1f}° | distances: W:{:.1f} E:{:.1f} S:{:.1f} N:{:.1f}cm'.format(
                x, y, math.degrees(theta),
                dist_to_west, dist_to_east, dist_to_south, dist_to_north))

        # X boundaries (lateral: port/stbd in WORLD frame = east/west) ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        if x < (self._min_x + self._margin):
            # west boundary (min_x) - push EAST (positive world x)
            distance_from_edge = max(0.0, x - self._min_x)
            if distance_from_edge == 0.0:
                self._log.warning('outside west boundary: x={:.1f}cm'.format(x))
            normalised_distance = distance_from_edge / self._margin
            urgency = 1.0 - normalised_distance
            force_magnitude = self._easing.apply(urgency)
            max_urgency = max(max_urgency, urgency)
            # push east (positive world x)
            world_fx += self._max_lateral_force * force_magnitude

        elif x > (self._max_x - self._margin):
            # east boundary (max_x) - push WEST (negative world x)
            distance_from_edge = max(0.0, self._max_x - x)
            if distance_from_edge == 0.0:
                self._log.warning('outside east boundary: x={:.1f}cm'.format(x))
            normalised_distance = distance_from_edge / self._margin
            urgency = 1.0 - normalised_distance
            force_magnitude = self._easing.apply(urgency)
            max_urgency = max(max_urgency, urgency)
            # push west (negative world x)
            world_fx -= self._max_lateral_force * force_magnitude

        # Y boundaries (longitudinal: forward/aft in WORLD frame = north/south) ┈┈┈┈┈┈┈┈┈┈

        if y < (self._min_y + self._margin):
            # south boundary (min_y) - push NORTH (positive world y)
            distance_from_edge = max(0.0, y - self._min_y)
            if distance_from_edge == 0.0:
                self._log.warning('outside south boundary: y={:.1f}cm'.format(y))
            normalised_distance = distance_from_edge / self._margin
            urgency = 1.0 - normalised_distance
            force_magnitude = self._easing.apply(urgency)
            max_urgency = max(max_urgency, urgency)
            # push north (positive world y)
            world_fy += self._max_lateral_force * force_magnitude

        elif y > (self._max_y - self._margin):
            # north boundary (max_y) - push SOUTH (negative world y)
            distance_from_edge = max(0.0, self._max_y - y)
            if distance_from_edge == 0.0:
                self._log.warning('outside north boundary: y={:.1f}cm'.format(y))
            normalised_distance = distance_from_edge / self._margin
            urgency = 1.0 - normalised_distance
            force_magnitude = self._easing.apply(urgency)
            max_urgency = max(max_urgency, urgency)
            # push south (negative world y)
            world_fy -= self._max_lateral_force * force_magnitude

        # transform world-frame forces to robot-frame intent ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        # rotation matrix from world frame to robot frame
        # Robot heading theta: 0=east, π/2=north, π=west, -π/2=south
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        # transform: rotate world forces into robot's reference frame
        # vx = lateral (positive = right/starboard in robot frame)
        # vy = longitudinal (positive = forward in robot frame)
        vx = world_fx * cos_theta + world_fy * sin_theta
        vy = -world_fx * sin_theta + world_fy * cos_theta

        # rotation force: turn away from boundary if approaching
        omega = 0.0
        if max_urgency > 0.1:  # only apply rotation if significantly close
            # Calculate desired heading (direction of repulsion in world frame)
            if world_fx != 0.0 or world_fy != 0.0:
                desired_heading = math.atan2(world_fy, world_fx)
                # Angular difference between current and desired heading
                heading_error = desired_heading - theta
                # Normalize to [-π, π]
                while heading_error > math.pi:
                    heading_error -= 2 * math.pi
                while heading_error < -math.pi:
                    heading_error += 2 * math.pi
                # Apply rotation proportional to heading error and urgency
                omega = self._max_rotation_force * max_urgency * np.sign(heading_error)

        # detect boundary encounter
        _was_at_boundary = self._at_boundary
        self._at_boundary = (max_urgency > 0.0)

        # log boundary encounter/exit
        if self._at_boundary and not _was_at_boundary:
            self._log.warning(Fore.RED + '⚠️  BOUNDARY ENCOUNTERED at pos: ({:.1f}, {:.1f})cm, heading: {:.1f}°'.format(
                x, y, math.degrees(theta)))
        elif not self._at_boundary and _was_at_boundary:
            self._log.info(Fore.GREEN + '✓ Cleared boundary')

        # calculate priority based on urgency
        self._current_priority = self._min_priority + (max_urgency * (self._max_priority - self._min_priority))

        # logging
        if self._verbose or (next(self._counter) % 20 == 0):
            if max_urgency > 0.0:
                self._log.info('world force: ({:.3f}, {:.3f}) → robot intent: ({:.3f}, {:.3f}, {:.3f}), priority: {:.2f}'.format(
                    world_fx, world_fy, vx, vy, omega, self._current_priority))

        return (vx, vy, omega)

    def x_update_intent_vector(self, x, y, theta):
        '''
        Calculate repulsion forces based on proximity to boundaries.

        For each boundary:
        1. Calculate distance from boundary
        2. If within margin, calculate approach angle
        3. Generate reflection vector (lateral + rotational)
        4. Scale by proximity using easing function

        The repulsion behavior mimics bouncing off a wall:
        - Lateral force (vx/vy) pushes away from boundary
        - Rotational force (omega) turns heading away if approaching

        Args:
            x, y: Current position in cm
            theta: Current heading in radians

        Returns:
            (vx, vy, omega) tuple
        '''
        vx = 0.0
        vy = 0.0
        omega = 0.0
        max_urgency = 0.0

        # calculate velocity components from heading (for approach angle detection)
        heading_vx     = math.cos(theta)
        heading_vy     = math.sin(theta)
        _theta_degrees = math.degrees(theta)

       # log current position and distances to all boundaries
        dist_to_west = x - self._min_x
        dist_to_east = self._max_x - x
        dist_to_south = y - self._min_y
        dist_to_north = self._max_y - y

        if next(self._counter) % 20 == 0:  # log every 20 polls to avoid spam
            self._log.info('pose: ({:.1f}, {:.1f})cm @ {:.1f}° | distances: W:{:.1f} E:{:.1f} S:{:.1f} N:{:.1f}cm'.format(
                x, y, math.degrees(theta),
                dist_to_west, dist_to_east, dist_to_south, dist_to_north))

        # X boundaries (lateral - port/stbd) ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        # west boundary (min_x) - too far port
        if x < (self._min_x + self._margin):
            distance_from_edge = x - self._min_x
            if distance_from_edge < 0:
                distance_from_edge = 0  # already outside!
                self._log.warning('outside west boundary: x={:.1f}cm'.format(x))

            normalised_distance = distance_from_edge / self._margin  # 0.0 to 1.0
            urgency = 1.0 - normalised_distance
            force_magnitude = self._easing.apply(urgency)
            max_urgency = max(max_urgency, urgency)

            # push starboard (positive vx)
            vx += self._max_lateral_force * force_magnitude

            # if heading toward boundary (negative vx component), rotate away
            if heading_vx < 0:
                # rotate clockwise (negative omega) to turn right/away from boundary
                omega -= self._max_rotation_force * force_magnitude

        # east boundary (max_x) - too far starboard
        elif x > (self._max_x - self._margin):
            distance_from_edge = self._max_x - x
            if distance_from_edge < 0:
                distance_from_edge = 0
                self._log.warning('outside east boundary: x={:.1f}cm'.format(x))

            normalised_distance = distance_from_edge / self._margin
            urgency = 1.0 - normalised_distance
            force_magnitude = self._easing.apply(urgency)
            max_urgency = max(max_urgency, urgency)

            # push port (negative vx)
            vx -= self._max_lateral_force * force_magnitude

            # if heading toward boundary (positive vx component), rotate away
            if heading_vx > 0:
                # rotate counter-clockwise (positive omega) to turn left/away from boundary
                omega += self._max_rotation_force * force_magnitude

        # Y boundaries (forward/aft) ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        # south boundary (min_y) - too far aft
        if y < (self._min_y + self._margin):
            distance_from_edge = y - self._min_y
            if distance_from_edge < 0:
                distance_from_edge = 0
                self._log.warning('outside south boundary: y={:.1f}cm'.format(y))

            normalised_distance = distance_from_edge / self._margin
            urgency = 1.0 - normalised_distance
            force_magnitude = self._easing.apply(urgency)
            max_urgency = max(max_urgency, urgency)

            # push forward (positive vy)
            vy += self._max_lateral_force * force_magnitude

            # if heading toward boundary (negative vy component), rotate away
            # rotate to point away from south boundary (toward north)
            if heading_vy < 0:
                # determine which way to rotate based on current heading
                # if facing more west, rotate clockwise; if more east, rotate counter-clockwise
                if heading_vx < 0:
                    omega -= self._max_rotation_force * force_magnitude
                else:
                    omega += self._max_rotation_force * force_magnitude

        # north boundary (max_y) - too far forward
        elif y > (self._max_y - self._margin):
            distance_from_edge = self._max_y - y
            if distance_from_edge < 0:
                distance_from_edge = 0
                self._log.warning('outside north boundary: y={:.1f}cm'.format(y))

            normalised_distance = distance_from_edge / self._margin
            urgency = 1.0 - normalised_distance
            force_magnitude = self._easing.apply(urgency)
            max_urgency = max(max_urgency, urgency)

            # push aft (negative vy)
            vy -= self._max_lateral_force * force_magnitude

            # if heading toward boundary (positive vy component), rotate away
            # rotate to point away from north boundary (toward south)
            if heading_vy > 0:
                # determine which way to rotate based on current heading
                if heading_vx < 0:
                    omega += self._max_rotation_force * force_magnitude
                else:
                    omega -= self._max_rotation_force * force_magnitude

        # detect boundary encounter
        _was_at_boundary = self._at_boundary
        self._at_boundary = (max_urgency > 0.0)
        # log boundary encounter/exit
        if self._at_boundary and not _was_at_boundary:
            self._log.warning(Fore.RED + '⚠️  BOUNDARY ENCOUNTERED at pos: ({:.1f}, {:.1f})cm, heading: {:.1f}°'.format(
                x, y, math.degrees(theta)))
        elif not self._at_boundary and _was_at_boundary:
            self._log.info(Fore.GREEN + '✓ Cleared boundary')

        # calculate priority based on urgency
        self._current_priority = self._min_priority + (max_urgency * (self._max_priority - self._min_priority))
        # logging
        if self._verbose or (next(self._counter) % 20 == 0):
            if max_urgency > 0.0:
                self._log.info('pos: ({:.1f}, {:.1f})cm, heading: {:.1f}°, intent: ({:.3f}, {:.3f}, {:.3f}), priority: {:.2f}'.format(
                    x, y, _theta_degrees, vx, vy, omega, self._current_priority))
        return (vx, vy, omega)

    def enable(self):
        if not self.enabled:
            super().enable()
            self._log.info('enabled.')
        else:
            self._log.warning("already enabled.")

    def disable(self):
        if self.enabled:
            super().disable()
            self._log.info('disabled.')
        else:
            self._log.warning("already disabled.")

#EOF
