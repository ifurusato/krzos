#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-04
# modified: 2025-11-04

import sys
import time
import itertools
from enum import Enum
import numpy as np
from math import isclose
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.cardinal import Cardinal
from core.event import Event
from behave.async_behaviour import AsyncBehaviour
from hardware.digital_pot import DigitalPotentiometer
from hardware.compass_encoder import CompassEncoder
from hardware.motor_controller import MotorController
from hardware.usfs import Usfs
from core.orientation import Orientation

class HeadingMode(Enum):
    '''
    Scout is a behavior that seeks to locate the most open heading for exploration
    by rotating the robot. The heading mode determines how directional control is applied.

    HeadingMode.RELATIVE:
        Sensor-reactive rotation that continuously adjusts to face the most open direction
        detected by ScoutSensor. The robot rotates to minimize the heading offset reported
        by the sensor, making it purely reactive to current environmental conditions without
        maintaining any fixed heading reference. Does not use IMU compass.

    HeadingMode.ABSOLUTE:
        IMU compass-based servo to an absolute world heading with obstacle avoidance.
        A base heading is set via set_heading_degrees() (or compass encoder), and
        ScoutSensor provides real-time offset adjustments for obstacle avoidance.
        The robot servos to (base + offset) in world coordinates, allowing navigation
        toward a cardinal direction while finding the clearest path. For example,
        base=270° (west) with ScoutSensor dynamically adjusting ±23° to avoid obstacles.
    '''
    RELATIVE = 1
    ABSOLUTE = 2

    @staticmethod
    def from_string(value):
        value_upper = value.upper()
        for mode in HeadingMode:
            if value_upper == mode.name:
                return mode
        raise NotImplementedError("HeadingMode not implemented for value: {}".format(value))

class Scout(AsyncBehaviour):
    NAME = 'scout'
    '''
    Scout is a rotation-only behavior that finds the most open direction for exploration.

    Scout controls ONLY rotation (omega). It does NOT move the robot forward.
    Other behaviors (e.g., Roam) handle forward motion (vy) independently.

    Scout can operate in two modes:
    - RELATIVE: Rotate to face direction indicated by ScoutSensor (stateless, reactive)
    - ABSOLUTE: Rotate to absolute compass heading using IMU with ScoutSensor offset adjustments

    The intent vector is always (0, 0, omega) - pure rotation.
    '''
    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Scout.NAME, level)
        _component_registry = Component.get_registry()
        _motor_controller = _component_registry.get(MotorController.NAME)
        if _motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        AsyncBehaviour.__init__(self, self._log, config, message_bus, message_factory, _motor_controller, level=level)
        self.add_event(Event.AVOID)
        # configuration
        _cfg = config['kros'].get('behaviour').get('scout')
        self._counter = itertools.count()
        self._verbose = _cfg.get('verbose', False)
        self._use_color = True
        self._heading_mode = HeadingMode.from_string(_cfg.get('heading_mode', 'RELATIVE'))
        self._use_dynamic_heading = _cfg.get('use_dynamic_heading', True)
        # heading control
        self._heading_degrees        = 0.0
        self._target_relative_offset = 0.0
        self._rotation_speed         = _cfg.get('rotation_speed', 0.5)
        self._rotation_tolerance     = _cfg.get('rotation_tolerance', 2.0)
        self._min_distance           = _cfg.get('min_distance', 200.0)
        self._omega_gain             = _cfg.get('omega_gain', 0.10) # base proportional gain
        # compass encoder tracking for dynamic heading updates
        # encoder tracking for RELATIVE mode
        self._last_encoder_pfwd      = None
        self._last_encoder_sfwd      = None
        self._last_encoder_paft      = None
        self._last_encoder_saft      = None
        self._last_encoder_value     = None
        self._last_omega             = 0.0
        self._max_omega_change       = 0.05  # maximum change in omega per iteration
        # ScoutSensor - always needed since both modes use it
        from hardware.scout_sensor import ScoutSensor
        self._scout_sensor = _component_registry.get(ScoutSensor.NAME)
        if self._scout_sensor is None:
            self._log.info(Fore.WHITE + 'creating Scout sensor…')
            self._scout_sensor = ScoutSensor(config, level=Level.INFO)
        else:
            self._log.info(Fore.WHITE + 'using existing Scout sensor.')
        self._max_distance = self._scout_sensor.distance_threshold
        # IMU - only needed for ABSOLUTE mode
        self._imu = None
        if self._heading_mode == HeadingMode.ABSOLUTE:
            self._imu = _component_registry.get(Usfs.NAME)
            if self._imu is None:
                raise MissingComponentError('IMU not available for ABSOLUTE mode.')
        # compass encoder for dynamic heading input
        self._compass_encoder = None
        if self._use_dynamic_heading:
            self._compass_encoder = _component_registry.get(CompassEncoder.NAME)
        # get motor objects
#       self._motor_pfwd = self._motor_controller.get_motor(Orientation.PFWD)
#       self._motor_sfwd = self._motor_controller.get_motor(Orientation.SFWD)
#       self._motor_paft = self._motor_controller.get_motor(Orientation.PAFT)
#       self._motor_saft = self._motor_controller.get_motor(Orientation.SAFT)
#       # geometry configuration
#       velocity = self._motor_pfwd.get_velocity()
#       self._steps_per_rotation = velocity.steps_per_rotation
#       self._wheel_diameter_mm = velocity._wheel_diameter
#       self._wheel_track_mm = config['kros']['geometry']['wheel_track']
#       wheel_circumference_cm = np.pi * self._wheel_diameter_mm / 10.0
#       rotation_circle_cm = np.pi * self._wheel_track_mm / 10.0
#       steps_per_degree_theoretical = (rotation_circle_cm / wheel_circumference_cm * self._steps_per_rotation) / 360.0
#       self._steps_per_degree = _cfg.get('steps_per_degree', steps_per_degree_theoretical)
#       self._log.info('steps_per_degree set to: {}'.format(self._steps_per_degree))
#       self._log.info('ready.')
        self._log.info('ready with heading mode: {}'.format(self._heading_mode.name))

    def set_heading_degrees(self, degrees):
        '''
        Sets the target heading.
        '''
        degrees = float(degrees) % 360.0
        self._heading_degrees = degrees
        self._log.info('heading set to {:.2f}°'.format(degrees))

    def set_heading_radians(self, radians):
        self.set_heading_degrees(np.degrees(radians))

    def set_heading_cardinal(self, cardinal):
        self.set_heading_degrees(cardinal.degrees)

    @property
    def name(self):
        return Scout.NAME

    @property
    def is_ballistic(self):
        return False

    @property
    def priority(self):
        '''
        Returns dynamic priority based on environmental constraint.
        '''
        _, max_open_distance = self._scout_sensor.get_heading_offset()
        return self._calculate_priority(max_open_distance)

    def callback(self):
        self._log.info('scout behaviour callback.')
        raise Exception('UNSUPPORTED callback')

    def execute(self, message):
        print('execute message {}.'.format(message))
        raise Exception('UNSUPPORTED execute')

    def start_loop_action(self):
        pass  # scout doesn't accelerate forward

    def stop_loop_action(self):
        pass  # scout doesn't decelerate forward

    def _dynamic_set_heading(self):
        '''
        Updates target heading from compass encoder deltas.
        Accumulates changes into _target_relative_offset.
        '''
        if self._compass_encoder:
            self._compass_encoder.update()
            _degrees = self._compass_encoder.get_degrees()
            if self._last_encoder_value is None:
                # first read - initialize
                self._last_encoder_value = _degrees
                self._log.info("compass encoder initialized at {:.2f}°".format(_degrees))
            else:
                # calculate change with wraparound handling
                delta = _degrees - self._last_encoder_value
                if delta > 180:
                    delta -= 360
                elif delta < -180:
                    delta += 360
                if abs(delta) > 1.0:  # threshold to ignore noise
                    self._last_encoder_value = _degrees
                    # accumulate delta into target offset
                    self._target_relative_offset = (self._target_relative_offset + delta) % 360.0
                    self._log.info("compass encoder delta: {:+.2f}°, target offset now: {:.2f}°".format(
                        delta, self._target_relative_offset))

    async def _poll(self):
        '''
        the asynchronous poll, updates the intent vector on each call.
        '''
        try:
            if next(self._counter) % 5 == 0:
                if self._use_dynamic_heading:
                    self._dynamic_set_heading()
            self._update_intent_vector()
        except Exception as e:
            self._log.error("{} thrown while polling: {}".format(type(e), e))
            self.disable()

    def _update_intent_vector(self):
        '''
        update the intent vector based on heading mode.

        scout only controls rotation. delegates to mode-specific methods.

        robot-relative components of the vector:
            vx:    lateral velocity (always 0.0 for Scout)
            vy:    longitudinal velocity (always 0.0 for Scout)
            omega: angular velocity (rotation) - this is what Scout controls
        '''
        match self._heading_mode:
            case HeadingMode.RELATIVE:
                self._update_intent_vector_relative()
            case HeadingMode.ABSOLUTE:
                self._update_intent_vector_absolute()
            case _:
                raise NotImplementedError("unhandled heading mode: {}".format(self._heading_mode))

    def _update_intent_vector_relative(self):
        '''
        RELATIVE mode: sensor-reactive rotation toward most open direction.
        Uses odometry to track actual heading, steers based on ScoutSensor offset.
        '''
        # track heading before update
        previous_heading = self._heading_degrees
        # update actual heading from motor encoders (odometry)
        self._update_heading_from_encoders()
        # calculate how much we actually rotated
        actual_rotation = (self._heading_degrees - previous_heading + 180.0) % 360.0 - 180.0
        # get exploration guidance from ScoutSensor (get this early so it's available)
        scout_offset, max_open_distance = self._scout_sensor.get_heading_offset()
        # consume rotation from target offset ONLY if encoder has been meaningfully active
        if abs(self._target_relative_offset) > 2.0:  # threshold above noise
            self._target_relative_offset -= actual_rotation
            # normalize
            if self._target_relative_offset > 180:
                self._target_relative_offset -= 360
            elif self._target_relative_offset < -180:
                self._target_relative_offset += 360
        else:
            # near zero - clamp to exactly zero to prevent drift from chase affecting it
            self._target_relative_offset = 0.0
            # reset odometry reference so chase doesn't accumulate drift
            # only reset when no obstacles present to avoid snap-back
            if abs(scout_offset) < 1.0:
                self._heading_degrees = 0.0
        # error is simply the sum of both inputs
        error = self._target_relative_offset + scout_offset
        # normalize
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        # calculate priority and omega
        priority = self._calculate_priority(max_open_distance)
        omega = self._calculate_omega(error, max_open_distance)
        vx = 0.0
        vy = 0.0
        self._intent_vector = (vx, vy, omega)
        if self._verbose:
            self._log.info("RELATIVE: target_offset={:+.2f}°; scout_offset={:+.2f}°; error={:+.2f}°; actual_rot={:+.2f}°; max_open={:.0f}mm; priority={:.2f}; omega={:.3f}".format(
                self._target_relative_offset, scout_offset, error, actual_rotation, max_open_distance, priority, omega))
            self._display_info('RELATIVE')

    def _update_intent_vector_absolute(self):
        '''
        ABSOLUTE mode: absolute compass heading with dynamic offset adjustments.
        Base heading from self._heading_degrees (set by methods or encoder).
        Offset from ScoutSensor for obstacle avoidance.
        '''
        # base heading from self._heading_degrees
        base_heading = self._heading_degrees % 360.0

        # get dynamic offset from ScoutSensor
        offset, max_open_distance = self._scout_sensor.get_heading_offset()

        # combine base + offset for final world heading
        desired_heading = (base_heading + offset) % 360.0
        current_heading = self._imu.poll() % 360.0
        error = (desired_heading - current_heading + 180.0) % 360.0 - 180.0

        # calculate priority and omega based on environmental constraint
        priority = self._calculate_priority(max_open_distance)
        omega = self._calculate_omega(error, max_open_distance)

        # scout only rotates
        vx = 0.0
        vy = 0.0
        self._intent_vector = (vx, vy, omega)

        if self._verbose:
            self._log.info("ABSOLUTE: base={:.2f}°; offset={:+.2f}°; desired={:.2f}°; current={:.2f}°; error={:.2f}°; max_open={:.0f}mm; priority={:.2f}; omega={:.3f}".format(
                base_heading, offset, desired_heading, current_heading, error, max_open_distance, priority, omega))
            self._display_info('ABSOLUTE')

    def _calculate_priority(self, max_open_distance):
        '''
        Calculate dynamic priority based on environmental constraint.
        More constrained environment (closer obstacles) = higher priority.

        Returns priority value 0.0 to 1.0
        '''
        min_priority = 0.2
        max_priority = 0.9
        clamped_distance = max(self._min_distance, min(max_open_distance, self._max_distance))
        normalized = (clamped_distance - self._min_distance) / (self._max_distance - self._min_distance)
        priority = max_priority - (normalized * (max_priority - min_priority))
        return priority

    def _calculate_omega(self, error, max_open_distance):
        '''
        Calculate rotation speed with urgency scaling and rate limiting.
        Closer obstacles = faster rotation toward opening.
        Rate limiting prevents oscillation while allowing quick response to sign changes.
        '''
        abs_error = abs(error)
        # deadzone
        if abs_error <= self._rotation_tolerance:
            self._last_omega = 0.0
            return 0.0
        # urgency multiplier based on environmental constraint
        urgency_mult = self._calculate_urgency(max_open_distance)
        gain = self._omega_gain * urgency_mult
        target_omega = abs_error * gain
        # clamp to limits
        target_omega = max(min(target_omega, self._rotation_speed), 0.08)
        target_omega = target_omega * (1 if error > 0 else -1)
        # check for sign change (overshoot detection)
        sign_changed = (self._last_omega * target_omega < 0)
        # rate limit: allow faster change on sign flip
        omega_change = target_omega - self._last_omega
        max_change = self._max_omega_change * 3.0 if sign_changed else self._max_omega_change
        if abs(omega_change) > max_change:
            omega = self._last_omega + max_change * (1 if omega_change > 0 else -1)
        else:
            omega = target_omega
        self._last_omega = omega
        return omega

    def _calculate_urgency(self, max_open_distance):
        '''
        Calculate urgency multiplier based on how constrained the environment is.
        More constrained (closer obstacles) = higher urgency = faster rotation.

        Returns multiplier 1.0 to max_multiplier
        '''
        # maximum urgency multiplier applied when obstacles are at min_distance
        # at max_distance (clear), multiplier is 1.0; at min_distance, multiplier is max_multiplier
        max_multiplier = 3.0
        # power curve exponent - controls how quickly urgency ramps up as obstacles approach
        # higher values (>2.0) = aggressive ramp near obstacles; lower values (~1.0) = more linear response
        urgency_power  = 2.0
        # clamp distance to meaningful range
        clamped_distance = max(self._min_distance, min(max_open_distance, self._max_distance))
        # normalize to 0.0 (at min_distance) to 1.0 (at max_distance)
        normalized_distance = (clamped_distance - self._min_distance) / (self._max_distance - self._min_distance)
        # calculate urgency: inverse of normalized distance with power curve
        urgency_mult = 1.0 + (max_multiplier - 1.0) * ((1.0 - normalized_distance) ** urgency_power)
        return urgency_mult

    def _update_heading_from_encoders(self):
        '''
        update self._heading_degrees based on actual motor encoder deltas.
        tracks rotation by measuring differential encoder movement.
        used by RELATIVE mode to maintain awareness of actual robot orientation.
        '''
        # get current encoder positions
        current_pfwd = self._motor_pfwd.decoder.steps
        current_sfwd = self._motor_sfwd.decoder.steps
        current_paft = self._motor_paft.decoder.steps
        current_saft = self._motor_saft.decoder.steps
        # initialize baseline on first call
        if self._last_encoder_pfwd is None:
            self._last_encoder_pfwd = current_pfwd
            self._last_encoder_sfwd = current_sfwd
            self._last_encoder_paft = current_paft
            self._last_encoder_saft = current_saft
            return
        # calculate deltas since last update
        delta_pfwd = current_pfwd - self._last_encoder_pfwd
        delta_sfwd = current_sfwd - self._last_encoder_sfwd
        delta_paft = current_paft - self._last_encoder_paft
        delta_saft = current_saft - self._last_encoder_saft
        # mecanum rotation: port wheels positive CW, starboard wheels negative CW
        # sum all wheel contributions (don't divide by 4)
        rotation_steps = delta_pfwd + delta_paft - delta_sfwd - delta_saft
        degrees_rotated = rotation_steps / self._steps_per_degree
        # update heading
        self._heading_degrees = (self._heading_degrees + degrees_rotated) % 360.0
        # store for next iteration
        self._last_encoder_pfwd = current_pfwd
        self._last_encoder_sfwd = current_sfwd
        self._last_encoder_paft = current_paft
        self._last_encoder_saft = current_saft

    def _display_info(self, message=''):
        if self._use_color:
            if self._intent_vector[2] == 0.0:
                self._log.info(Style.DIM + "{} intent vector: ({:4.2f}, {:4.2f}, {:4.2f})".format(
                    message, self._intent_vector[0], self._intent_vector[1], self._intent_vector[2]))
            else:
                self._log.info("{} intent vector: ({:4.2f}, {:4.2f}, ".format(
                        message, self._intent_vector[0], self._intent_vector[1])
                    + Fore.CYAN + "{:4.2f}".format(self._intent_vector[2]) + Style.RESET_ALL + ")")
        else:
            self._log.info("intent vector: ({:.2f},{:.2f},{:.2f})".format(
                self._intent_vector[0], self._intent_vector[1], self._intent_vector[2]))

    def enable(self):
        if self.enabled:
            self._log.warning("already enabled.")
            return
        self._log.info("enabling scout…")
        if self._scout_sensor and not self._scout_sensor.enabled:
            self._scout_sensor.enable()
        AsyncBehaviour.enable(self)
        # initialize encoder value to prevent immediate override
        if self._compass_encoder:
            self._compass_encoder.update()
            self._last_encoder_value = self._compass_encoder.get_degrees()
        # initialize heading based on mode
        if self._heading_mode == HeadingMode.RELATIVE:
            self._heading_degrees = 0.0
            self._log.info("RELATIVE mode initialized (sensor-reactive, no fixed heading)")
        elif self._heading_mode == HeadingMode.ABSOLUTE:
            self._heading_degrees = self._imu.poll() % 360.0
            self._log.info("ABSOLUTE mode initialized to current IMU heading: {:.2f}°".format(self._heading_degrees))
        self._log.info("scout enabled.")

    def disable(self):
        if not self.enabled:
            self._log.warning("already disabled.")
            return
        self._log.info("disabling scout…")
        if self._scout_sensor and self._scout_sensor.enabled:
            self._scout_sensor.disable()
        AsyncBehaviour.disable(self)
        self._log.info(Fore.YELLOW + 'disabled.')

#EOF
