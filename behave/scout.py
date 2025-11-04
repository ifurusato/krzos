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

    RELATIVE (HeadingMode.RELATIVE):
        Sensor-reactive rotation that continuously adjusts to face the most open direction
        detected by ScoutSensor. The robot rotates to minimize the heading offset reported
        by the sensor, making it purely reactive to current environmental conditions without
        maintaining any fixed heading reference.

    ABSOLUTE (HeadingMode.ABSOLUTE):
        IMU compass-based servo to an absolute world heading. When set_heading_degrees(270)
        is called, the robot continuously servos to face west (270°) using the IMU
        magnetometer, correcting for drift in real-time. Suitable for commanding
        cardinal directions (e.g., "face north", "face bearing 135°").

    BLENDED (HeadingMode.BLENDED):
        Absolute compass heading with dynamic offset adjustments. A base absolute heading
        is set via set_heading_degrees(), and a real-time offset is provided by
        ScoutSensor (or compass encoder). The robot servos to (base + offset) in world
        coordinates, allowing a primary direction with adjustments for finding the
        clearest path. For example, base=270° (west) with ScoutSensor dynamically
        adjusting ±23° to find open space.
    '''
    RELATIVE = 1
    ABSOLUTE = 2
    BLENDED  = 3

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
    
    Scout can operate in three modes:
    - RELATIVE: Rotate to face direction indicated by ScoutSensor (stateless, reactive)
    - ABSOLUTE: Rotate to absolute compass heading using IMU
    - BLENDED: Absolute heading with ScoutSensor offset adjustments
    
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
        self._use_world_coordinates = _cfg.get('use_world_coordinates')
        # heading control
        self._heading_degrees = 0.0
        self._rotation_speed = _cfg.get('rotation_speed', 0.5)
        self._rotation_tolerance = _cfg.get('rotation_tolerance', 2.0)
        # compass encoder tracking for dynamic heading updates
        self._last_encoder_value = None
        self._last_omega = 0.0
        self._max_omega_change = 0.05  # rate limiting for smooth rotation
        # ScoutSensor for dynamic obstacle avoidance heading
        self._scout_sensor = None
        if self._heading_mode in [HeadingMode.RELATIVE, HeadingMode.BLENDED]:
            from hardware.scout_sensor import ScoutSensor
            self._scout_sensor = _component_registry.get(ScoutSensor.NAME)
            if self._scout_sensor is None:
                self._log.info(Fore.WHITE + 'creating Scout sensor…')
                self._scout_sensor = ScoutSensor(config, level=Level.INFO)
            else:
                self._log.info(Fore.WHITE + 'using existing Scout sensor.')
        # compass encoder for dynamic heading input
        self._compass_encoder = None
        if self._use_dynamic_heading:
            self._compass_encoder = _component_registry.get(CompassEncoder.NAME)
        self._imu = _component_registry.get(Usfs.NAME)
        if self._imu is None:
            raise MissingComponentError('IMU not available.')
        self._log.info('ready with heading mode: {}'.format(self._heading_mode.name))

    def set_heading_degrees(self, degrees, internal=False):
        '''
        set target heading. interpretation depends on mode:
        - RELATIVE: ignored (purely sensor-reactive, no fixed heading)
        - ABSOLUTE/BLENDED: degrees is absolute compass heading (0-360°)
        '''
        if self._heading_mode == HeadingMode.RELATIVE:
            self._log.info('RELATIVE mode: ignoring heading set (sensor-reactive only)')
            return
        if not internal and self._heading_mode == HeadingMode.ABSOLUTE:
            self._log.info('ABSOLUTE mode: ignoring programmatic heading change (encoder only)')
            return
        degrees = float(degrees) % 360.0
        self._heading_degrees = degrees
        self._log.info('heading set to {:.2f}°'.format(degrees))

    def set_heading_radians(self, radians, internal=False):
        self.set_heading_degrees(np.degrees(radians), internal)

    def set_heading_cardinal(self, cardinal, internal=False):
        self.set_heading_degrees(cardinal.degrees, internal)

    @property
    def name(self):
        return Scout.NAME

    @property
    def is_ballistic(self):
        return False

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
        updates heading from compass encoder only when it changes.
        '''
        if self._compass_encoder:
            self._compass_encoder.update()
            _degrees = self._compass_encoder.get_degrees()
            # only update if encoder value has changed
            if self._last_encoder_value is None:
                self._log.info("encoder first read: {:.2f}° (storing but not applying)".format(_degrees))
                self._last_encoder_value = _degrees
            elif abs(_degrees - self._last_encoder_value) > 1.0:
                self._log.info("encoder changed: {:.2f}° → {:.2f}° (delta: {:.2f}°)".format(
                    self._last_encoder_value, _degrees, abs(_degrees - self._last_encoder_value)))
                self._last_encoder_value = _degrees
                self.set_heading_degrees(_degrees, internal=True)
                self._log.info("dynamic heading: {:4.2f} degrees".format(_degrees))
            else:
                self._log.debug("encoder unchanged: {:.2f}°".format(_degrees))

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
            case HeadingMode.BLENDED:
                self._update_intent_vector_blended()
            case _:
                raise NotImplementedError("unhandled heading mode: {}".format(self._heading_mode))

    def _update_intent_vector_relative(self):
        '''
        RELATIVE mode: rotate to face the direction ScoutSensor indicates.
        purely reactive to sensor input - no state tracking needed.
        '''
        # ScoutSensor provides the target direction as an offset
        if self._scout_sensor:
            scout_offset, open_distance = self._scout_sensor.get_heading_offset()
            print("RAW SENSOR: offset={:+.2f}°, distance={:.1f}mm".format(scout_offset, open_distance))
        else:
            scout_offset = 0.0
            open_distance = None
        
        # error is simply the scout offset - rotate to make it zero
        error = scout_offset
        omega = self._calculate_omega(error)
        
        # scout only rotates - no forward or lateral motion
        vx = 0.0
        vy = 0.0
        self._intent_vector = (vx, vy, omega)
        
        if self._verbose:
#           self._log.info("RELATIVE: scout_offset={:+.2f}°; error={:.2f}°; omega={:.3f}{}".format(
            print("RELATIVE: scout_offset={:+.2f}°; error={:.2f}°; omega={:.3f}{}".format(
                scout_offset,
                error,
                omega,
                "; open_dist={:.1f}mm".format(open_distance) if open_distance else ""))
            self._display_info('RELATIVE')

    def _update_intent_vector_absolute(self):
        '''
        ABSOLUTE mode: rotate to match compass heading (from IMU).
        uses self._heading_degrees as target (set via setter methods or encoder).
        '''
        current_heading = self._imu.poll() % 360.0
        desired_heading = self._heading_degrees % 360.0
        error = (desired_heading - current_heading + 180.0) % 360.0 - 180.0
        omega = self._calculate_omega(error)
        # scout only rotates
        vx = 0.0
        vy = 0.0
        self._intent_vector = (vx, vy, omega)
        if self._verbose:
            self._log.info("ABSOLUTE: desired={:.2f}°; current={:.2f}°; error={:.2f}°; omega={:.3f}".format(
                desired_heading, current_heading, error, omega))
            self._display_info('ABSOLUTE')

    def _update_intent_vector_blended(self):
        '''
        BLENDED mode: absolute compass heading with dynamic offset adjustments.
        base heading from self._heading_degrees (set by methods or encoder).
        offset from ScoutSensor for obstacle avoidance.
        '''
        # base heading from self._heading_degrees
        base_heading = self._heading_degrees % 360.0
        # get dynamic offset from ScoutSensor
        if self._scout_sensor:
            offset, open_distance = self._scout_sensor.get_heading_offset()
        else:
            offset = 0.0
            open_distance = None
        # combine base + offset for final world heading
        desired_heading = (base_heading + offset) % 360.0
        current_heading = self._imu.poll() % 360.0
        error = (desired_heading - current_heading + 180.0) % 360.0 - 180.0
        omega = self._calculate_omega(error)
        # scout only rotates
        vx = 0.0
        vy = 0.0
        self._intent_vector = (vx, vy, omega)
        if self._verbose:
            self._log.info("BLENDED: base={:.2f}°; offset={:+.2f}°; desired={:.2f}°; current={:.2f}°; error={:.2f}°; omega={:.3f}{}".format(
                base_heading, offset, desired_heading, current_heading, error, omega,
                "; open_dist={:.1f}mm".format(open_distance) if open_distance else ""))
            self._display_info('BLENDED')

    def _calculate_omega(self, error):
        '''
        calculate rotation speed with rate limiting to prevent oscillation.
        allows faster response when crossing through target (error sign change).
        '''
        abs_error = abs(error)
        # deadzone
        if abs_error <= self._rotation_tolerance:
            self._last_omega = 0.0
            return 0.0
        gain = 0.05  # conservative gain
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
        elif self._heading_mode == HeadingMode.BLENDED:
            self._heading_degrees = self._imu.poll() % 360.0
            self._log.info("BLENDED mode initialized to current IMU heading: {:.2f}°".format(self._heading_degrees))
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
