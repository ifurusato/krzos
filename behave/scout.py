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
        Encoder-based rotation to a relative heading offset from the robot's current
        orientation. When set_heading_degrees(45) is called, the robot rotates 45°
        clockwise from its current facing using motor encoder odometry. ScoutSensor
        can dynamically adjust this offset for obstacle avoidance.

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
    - RELATIVE: Rotate relative to current orientation using encoders + ScoutSensor
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
        self._target_relative_offset = 0.0
        self._rotation_speed = _cfg.get('rotation_speed', 0.5)
        self._rotation_tolerance = _cfg.get('rotation_tolerance', 2.0)
        
        # encoder tracking for RELATIVE mode
        self._last_encoder_pfwd = None
        self._last_encoder_sfwd = None
        self._last_encoder_paft = None
        self._last_encoder_saft = None
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
        
        # IMU for absolute/blended modes
        self._imu = None
        if self._use_world_coordinates and self._heading_mode in [HeadingMode.ABSOLUTE, HeadingMode.BLENDED]:
            self._imu = _component_registry.get(Usfs.NAME)
        
        # validate required components for heading modes
        if self._heading_mode == HeadingMode.ABSOLUTE and self._imu is None:
            raise MissingComponentError('ABSOLUTE heading mode requires IMU (USFS) to be available.')
        if self._heading_mode == HeadingMode.BLENDED and self._imu is None:
            raise MissingComponentError('BLENDED heading mode requires IMU (USFS) to be available.')
        
        # get motor objects for encoder tracking
        self._motor_pfwd = self._motor_controller.get_motor(Orientation.PFWD)
        self._motor_sfwd = self._motor_controller.get_motor(Orientation.SFWD)
        self._motor_paft = self._motor_controller.get_motor(Orientation.PAFT)
        self._motor_saft = self._motor_controller.get_motor(Orientation.SAFT)
        
        # geometry configuration for encoder-based rotation
        velocity = self._motor_pfwd.get_velocity()
        self._steps_per_rotation = velocity.steps_per_rotation
        self._wheel_diameter_mm = velocity._wheel_diameter
        self._wheel_track_mm = config['kros']['geometry']['wheel_track']
        wheel_circumference_cm = np.pi * self._wheel_diameter_mm / 10.0
        rotation_circle_cm = np.pi * self._wheel_track_mm / 10.0
        steps_per_degree_theoretical = (rotation_circle_cm / wheel_circumference_cm * self._steps_per_rotation) / 360.0
        self._steps_per_degree = _cfg.get('steps_per_degree', steps_per_degree_theoretical)
        self._log.info('steps_per_degree set to: {}'.format(self._steps_per_degree))
        
        self._log.info('ready with heading mode: {}'.format(self._heading_mode.name))

    def set_heading_degrees(self, degrees):
        '''
        Set target heading. Interpretation depends on mode:
        - RELATIVE: degrees is an offset from starting orientation (0-360°)
        - ABSOLUTE/BLENDED: degrees is absolute compass heading (0-360°)
        '''
        degrees = float(degrees) % 360.0
        if self._heading_mode == HeadingMode.RELATIVE:
            self._target_relative_offset = degrees
            self._log.info('RELATIVE: target offset set to {:.2f}°'.format(degrees))
        else:
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

    def callback(self):
        self._log.info('scout behaviour callback.')
        raise Exception('UNSUPPORTED callback')

    def execute(self, message):
        print('execute message {}.'.format(message))
        raise Exception('UNSUPPORTED execute')

    def start_loop_action(self):
        pass  # Scout doesn't accelerate forward

    def stop_loop_action(self):
        pass  # Scout doesn't decelerate forward

    def _dynamic_set_heading(self):
        '''
        Updates heading from compass encoder if available and dynamic heading enabled.
        '''
        if self._compass_encoder:
            self._compass_encoder.update()
            _degrees = self._compass_encoder.get_degrees()
            self.set_heading_degrees(_degrees)
            self._log.info("dynamic heading: {:4.2f} degrees".format(_degrees))

    async def _poll(self):
        '''
        The asynchronous poll, updates the intent vector on each call.
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
        Update the intent vector based on heading mode.
        
        Scout only controls rotation. Delegates to mode-specific methods.
        
        Robot-relative components of the vector:
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
                raise NotImplementedError("Unhandled heading mode: {}".format(self._heading_mode))

    def _update_heading_from_encoders(self):
        '''
        Update self._heading_degrees based on actual motor encoder deltas.
        Tracks rotation by measuring differential encoder movement.
        Used by RELATIVE mode to maintain awareness of actual robot orientation.
        '''
        # Get current encoder positions
        current_pfwd = self._motor_pfwd.decoder.steps
        current_sfwd = self._motor_sfwd.decoder.steps
        current_paft = self._motor_paft.decoder.steps
        current_saft = self._motor_saft.decoder.steps

        # Initialize baseline on first call
        if self._last_encoder_pfwd is None:
            self._last_encoder_pfwd = current_pfwd
            self._last_encoder_sfwd = current_sfwd
            self._last_encoder_paft = current_paft
            self._last_encoder_saft = current_saft
            return

        # Calculate deltas since last update
        delta_pfwd = current_pfwd - self._last_encoder_pfwd
        delta_sfwd = current_sfwd - self._last_encoder_sfwd
        delta_paft = current_paft - self._last_encoder_paft
        delta_saft = current_saft - self._last_encoder_saft

        # Calculate rotation (Mecanum kinematics for in-place rotation)
        rotation_steps = (delta_pfwd + delta_paft - delta_sfwd - delta_saft) / 4.0
        degrees_rotated = rotation_steps / self._steps_per_degree

        # Update heading
        self._heading_degrees = (self._heading_degrees + degrees_rotated) % 360.0

        # Store for next iteration
        self._last_encoder_pfwd = current_pfwd
        self._last_encoder_sfwd = current_sfwd
        self._last_encoder_paft = current_paft
        self._last_encoder_saft = current_saft

    def _update_intent_vector_relative(self):
        '''
        RELATIVE mode: Continuously servo to achieve a relative heading offset.
        Uses ScoutSensor for dynamic obstacle avoidance if available.
        '''
        self._update_heading_from_encoders()
        
        # Get desired relative offset using ScoutSensor if available
        if self._scout_sensor:
            desired_offset, open_distance = self._scout_sensor.get_heading_offset()
        else:
            desired_offset = self._target_relative_offset
            open_distance = None
        
        # Calculate error
        current_offset = self._heading_degrees % 360.0
        error = (desired_offset - current_offset + 180.0) % 360.0 - 180.0
        omega = self._calculate_omega(error)
        
        # Scout only rotates - no forward or lateral motion
        vx = 0.0
        vy = 0.0
        self._intent_vector = (vx, vy, omega)
        
        if self._verbose:
            self._log.info("RELATIVE: target={:.2f}°; current={:.2f}°; error={:.2f}°; omega={:.3f}{}".format(
                desired_offset, current_offset, error, omega,
                "; open_dist={:.1f}mm".format(open_distance) if open_distance else ""))
            self._display_info('RELATIVE')

    def _update_intent_vector_absolute(self):
        '''
        ABSOLUTE mode: Rotate to match compass heading (from IMU).
        '''
        current_heading = self._imu.poll() % 360.0
        if self._compass_encoder:
            self._compass_encoder.update()
            desired_heading = self._compass_encoder.get_degrees() % 360.0
        else:
            desired_heading = self._heading_degrees % 360.0

        error = (desired_heading - current_heading + 180.0) % 360.0 - 180.0
        omega = self._calculate_omega(error)

        # Scout only rotates
        vx = 0.0
        vy = 0.0
        self._intent_vector = (vx, vy, omega)

        if self._verbose:
            self._log.info("ABSOLUTE: desired={:.2f}°; current={:.2f}°; error={:.2f}°; omega={:.3f}".format(
                desired_heading, current_heading, error, omega))
            self._display_info('ABSOLUTE')

    def _update_intent_vector_blended(self):
        '''
        BLENDED mode: Absolute compass heading with dynamic offset adjustments.
        Base heading set via set_heading_degrees(), offset from ScoutSensor or compass encoder.
        Combines absolute world heading with real-time adjustments for obstacles.
        '''
        # Get base heading (set by behavior via set_heading_degrees())
        base_heading = self._heading_degrees % 360.0
        
        # Get dynamic offset from ScoutSensor (priority) or compass encoder (fallback)
        if self._scout_sensor:
            offset, open_distance = self._scout_sensor.get_heading_offset()
        elif self._compass_encoder:
            self._compass_encoder.update()
            # Map encoder 0-360° to offset range ±180°
            offset = (self._compass_encoder.get_degrees() - 180.0)
            open_distance = None
        else:
            offset = 0.0
            open_distance = None
        
        # Combine base + offset for final world heading
        desired_heading = (base_heading + offset) % 360.0
        current_heading = self._imu.poll() % 360.0
        error = (desired_heading - current_heading + 180.0) % 360.0 - 180.0
        omega = self._calculate_omega(error)
        
        # Scout only rotates
        vx = 0.0
        vy = 0.0
        self._intent_vector = (vx, vy, omega)
        
        if self._verbose:
            self._log.info("BLENDED: base={:.2f}°; offset={:.2f}°; desired={:.2f}°; current={:.2f}°; omega={:.3f}{}".format(
                base_heading, offset, desired_heading, current_heading, omega,
                "; open_dist={:.1f}mm".format(open_distance) if open_distance else ""))
            self._display_info('BLENDED')

    def _calculate_omega(self, error):
        '''
        Calculate rotation speed with rate limiting to prevent oscillation.
        '''
        abs_error = abs(error)
        
        # Deadzone
        if abs_error <= self._rotation_tolerance:
            self._last_omega = 0.0
            return 0.0
        
        gain = 0.05  # Conservative gain to start - tune as needed
        target_omega = abs_error * gain
        
        # Clamp to limits
        target_omega = max(min(target_omega, self._rotation_speed), 0.08)
        target_omega = target_omega * (1 if error > 0 else -1)
        
        # Rate limit: smooth the change
        omega_change = target_omega - self._last_omega
        if abs(omega_change) > self._max_omega_change:
            omega = self._last_omega + self._max_omega_change * (1 if omega_change > 0 else -1)
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
        # Initialize RELATIVE mode to current orientation if using world coordinates
        if self._heading_mode == HeadingMode.RELATIVE and self._use_world_coordinates:
            self._log.info(Fore.YELLOW + "initialize RELATIVE mode to current orientation…")
            current_heading = self._imu.poll() % 360.0
            self._heading_degrees = current_heading
            self._log.info("RELATIVE mode initialized at IMU heading: {:.2f}°".format(current_heading))
        self._log.info("scout enabled.")

    def disable(self):
        if not self.enabled:
            self._log.debug("already disabled.")
            return
        self._log.info("disabling scout…")
        if self._scout_sensor and self._scout_sensor.enabled:
            self._scout_sensor.disable()
        AsyncBehaviour.disable(self)
        self._log.info(Fore.YELLOW + 'disabled.')

#EOF
