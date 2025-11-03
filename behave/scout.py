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
from math import isclose, copysign
from threading import Thread, Event as ThreadEvent
import asyncio
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.cardinal import Cardinal
from core.event import Event
from behave.async_behaviour import AsyncBehaviour
from hardware.roam_sensor import RoamSensor
from hardware.digital_pot import DigitalPotentiometer
from hardware.compass_encoder import CompassEncoder
from hardware.motor_controller import MotorController
from hardware.usfs import Usfs
from core.orientation import Orientation

class HeadingMode(Enum):
    '''
    Scout is a behavior that seeks to locate the most open heading for further
    exploration. The heading mode determines how directional control is applied.

    NONE (HeadingMode.NONE):
        Pure obstacle avoidance with no directional control. Robot moves straight
        forward (0°) while scaling speed based on detected obstacles. This is the
        default mode for simple roaming without external direction.

    RELATIVE (HeadingMode.RELATIVE):
        Encoder-based rotation to a relative heading offset from the robot's current
        orientation. When set_heading_degrees(45) is called, the robot rotates 45°
        clockwise from its current facing using motor encoder odometry, then maintains
        that orientation while moving forward. Suitable for behaviors that command
        relative direction changes (e.g., "turn 30° right").

    ABSOLUTE (HeadingMode.ABSOLUTE):
        IMU compass-based servo to an absolute world heading. When set_heading_degrees(270)
        is called, the robot continuously servos to face west (270°) using the IMU
        magnetometer, correcting for drift in real-time. The robot moves forward while
        maintaining the absolute compass bearing. Suitable for behaviors that command
        cardinal directions (e.g., "go north", "face bearing 135°").

    BLENDED (HeadingMode.BLENDED):
        Absolute compass heading with dynamic offset adjustments. A base absolute heading
        is set via set_heading_degrees(), and a real-time offset is provided by the
        compass encoder (or other dynamic source). The robot servos to (base + offset)
        in world coordinates, allowing behaviors to specify a primary direction while
        permitting adjustments for obstacle avoidance or path optimization. For example,
        OpenPathSensor might set base=270° (west) while dynamically adjusting ±30° to
        find the clearest path.
    '''
    NONE     = 0
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
    Implements a roaming behaviour. The end result of this Behaviour is to
    provide a forward speed limit based on a fused distance value provided
    by RoamSensor (PWM + VL53L5CX). If no obstacle is perceived, the velocity
    limit is removed.

    Scout uses a vector-based motor control, allowing for heading-based driving.
    Rotational alignment is performed when the heading changes, using Mecanum
    wheel kinematics and encoder counts.
    '''
    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Scout.NAME, level)
        _component_registry = Component.get_registry()
        _motor_controller = _component_registry.get(MotorController.NAME)
        if _motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        AsyncBehaviour.__init__(self, self._log, config, message_bus, message_factory, _motor_controller, level=level)
        self.add_event(Event.AVOID)
        _cfg = config['kros'].get('behaviour').get('scout')
        self._counter = itertools.count()
        self._verbose                = _cfg.get('verbose', False)
        self._use_color              = True
        self._heading_mode           = HeadingMode.from_string(_cfg.get('heading_mode', 'NONE'))
        self._default_speed          = _cfg.get('default_speed', 0.8)
        self._use_dynamic_speed      = _cfg.get('use_dynamic_speed', True)
        self._use_dynamic_heading    = _cfg.get('use_dynamic_heading', True) and self._heading_mode is not HeadingMode.NONE
        self._deadband_threshold     = _cfg.get('deadband_threshold', 0.07)
        self._use_world_coordinates  = _cfg.get('use_world_coordinates')
        _rs_cfg = config['kros'].get('hardware').get('roam_sensor')
        self._min_distance           = _rs_cfg.get('min_distance')
        self._max_distance           = _rs_cfg.get('max_distance')
        self._heading_degrees        = 0.0
        self._target_relative_offset = 0.0
        self._rotation_speed         = _cfg.get('rotation_speed', 0.5)
        self._rotation_tolerance     = _cfg.get('rotation_tolerance', 2.0)
        self._minimum_steering_speed = _cfg.get('minimum_steering_speed', 0.3)
        self._front_distance         = 0.0
        self._last_amplitude         = 0.0
        # encoder tracking for RELATIVE mode
        self._last_encoder_pfwd      = None
        self._last_encoder_sfwd      = None
        self._last_encoder_paft      = None
        self._last_encoder_saft      = None
        self._last_omega             = 0.0
        self._max_omega_change       = 0.05  # maximum change in omega per iteration
        # component access
        self._roam_sensor = _component_registry.get(RoamSensor.NAME)
        if self._roam_sensor is None:
            self._log.info(Fore.WHITE + 'creating Roam sensor…')
            self._roam_sensor = RoamSensor(config, level=Level.INFO)
        else:
            self._log.info(Fore.WHITE + 'using existing Roam sensor.')
        # OpenPathSensor for dynamic obstacle avoidance heading
        self._open_path_sensor = None
        if self._heading_mode in [HeadingMode.RELATIVE, HeadingMode.BLENDED]:
            from hardware.open_path_sensor import OpenPathSensor
            self._open_path_sensor = _component_registry.get(OpenPathSensor.NAME)
            if self._open_path_sensor is None:
                self._log.info(Fore.WHITE + 'creating OpenPath sensor…')
                self._open_path_sensor = OpenPathSensor(config, level=Level.INFO)
            else:
                self._log.info(Fore.WHITE + 'using existing OpenPath sensor.')
        # digital pot and compass encoder
        self._digital_pot = None
        if self._use_dynamic_speed:
            self._digital_pot = _component_registry.get(DigitalPotentiometer.NAME)
        self._compass_encoder = None
        if self._use_dynamic_heading:
            self._compass_encoder = _component_registry.get(CompassEncoder.NAME)
        # use IMU if world coordinates flag set and heading mode requires it
        self._imu = None
        if self._use_world_coordinates and self._use_dynamic_heading:
            self._imu = _component_registry.get(Usfs.NAME)
        # validate required components for heading modes
        if self._heading_mode == HeadingMode.ABSOLUTE and self._imu is None:
            raise MissingComponentError('ABSOLUTE heading mode requires IMU (USFS) to be available.')
        if self._heading_mode == HeadingMode.BLENDED and self._imu is None:
            raise MissingComponentError('BLENDED heading mode requires IMU (USFS) to be available.')
        if self._use_dynamic_heading and self._compass_encoder is None and self._digital_pot is None:
            raise MissingComponentError('Dynamic heading requires compass encoder or digital pot to be available.')
        # get motor objects
        self._motor_pfwd = self._motor_controller.get_motor(Orientation.PFWD)
        self._motor_sfwd = self._motor_controller.get_motor(Orientation.SFWD)
        self._motor_paft = self._motor_controller.get_motor(Orientation.PAFT)
        self._motor_saft = self._motor_controller.get_motor(Orientation.SAFT)
        # geometry configuration
        velocity = self._motor_pfwd.get_velocity()
        self._steps_per_rotation = velocity.steps_per_rotation
        self._wheel_diameter_mm = velocity._wheel_diameter
        self._wheel_track_mm = config['kros']['geometry']['wheel_track']
        wheel_circumference_cm = np.pi * self._wheel_diameter_mm / 10.0
        rotation_circle_cm = np.pi * self._wheel_track_mm / 10.0
        steps_per_degree_theoretical = (rotation_circle_cm / wheel_circumference_cm * self._steps_per_rotation) / 360.0
        self._steps_per_degree = _cfg.get('steps_per_degree', steps_per_degree_theoretical)
        self._log.info('steps_per_degree set to: {}'.format(self._steps_per_degree))
        self._log.info('ready.')

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
        self._accelerate()

    def stop_loop_action(self):
        self._decelerate()

    def _dynamic_set_default_speed(self):
        if self._digital_pot:
            _speed = self._digital_pot.get_scaled_value(False)
            if isclose(_speed, 0.0, abs_tol=0.08):
                self._digital_pot.set_black()
                self._default_speed = 0.0
            else:
                self._digital_pot.set_rgb(self._digital_pot.value)
                self._default_speed = _speed

    def _dynamic_set_heading(self):
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
                self._dynamic_set_default_speed()
                if self._use_dynamic_heading:
                    self._dynamic_set_heading()
            self._update_intent_vector()
        except Exception as e:
            self._log.error("{} thrown while polling: {}".format(type(e), e))
            self.disable()

    def _update_intent_vector(self):
        '''
        Update the intent vector based on heading, speed, obstacle logic, and
        rotation alignment. Delegates to rotation and linear movement handlers.
        Dispatches heading logic based on current mode.

        Robot-relative components of the vector:

            vx:    lateral velocity component (robot‑relative). Positive vx produces
                   lateral motion to starboard, negative to port.
            vy:    longitudinal velocity component (robot‑relative). Positive vy
                   produces forward motion, negative reverse motion.
            omega: angular velocity (rotation) component. Positive omega produces a
                   clockwise rotation, negative a counter-clockwise rotation.
        '''
        match self._heading_mode:
            case HeadingMode.NONE:
                self._update_linear_vector()
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
        Uses OpenPathSensor for dynamic obstacle avoidance if available.
        '''
        self._update_heading_from_encoders()
        # get desired relative offset and distance at that heading, using OpenPathSensor steering if obstacle is close
        if self._open_path_sensor and self._front_distance < self._max_distance:
            desired_offset, open_distance = self._open_path_sensor.get_heading_offset()
        else:
            desired_offset = self._target_relative_offset
            open_distance = self._max_distance  # assume clear if no sensor
        # calculate error
        current_offset = self._heading_degrees % 360.0
        error = (desired_offset - current_offset + 180.0) % 360.0 - 180.0
        omega = self._calculate_omega(error)
        # get forward movement vector, then add rotation
        self._update_linear_vector()
        vx, vy, _ = self._intent_vector
        self._intent_vector = (vx, vy, omega)
        if self._verbose:
            self._log.info("RELATIVE: target={:.2f}; current={:.2f}; error={:.2f}; omega={:.3f}; open_dist={:.1f}mm".format(
                desired_offset, current_offset, error, omega, open_distance))
            self._display_info('RELATIVE')

    def _update_intent_vector_absolute(self):
        '''
        ABSOLUTE mode: Rotate to match compass heading (from IMU)
        while performing obstacle-aware forward movement.
        '''
        current_heading = self._imu.poll() % 360.0
        if self._compass_encoder:
            self._compass_encoder.update()
            desired_heading = self._compass_encoder.get_degrees() % 360.0
        else:
            desired_heading = self._heading_degrees % 360.0

        error = (desired_heading - current_heading + 180.0) % 360.0 - 180.0
        omega = self._calculate_omega(error)

        # let _update_linear_vector handle forward movement, then add rotation
        self._update_linear_vector()
        vx, vy, _ = self._intent_vector
        self._intent_vector = (vx, vy, omega)

        if self._verbose:
            self._log.info("ABSOLUTE: desired={:.2f}; current={:.2f}; error={:.2f}; omega={:.3f}".format(
                desired_heading, current_heading, error, omega))
            self._display_info('ABSOLUTE')

    def _update_intent_vector_blended(self):
        '''
        BLENDED mode: Absolute compass heading with dynamic offset adjustments.
        Base heading set via set_heading_degrees(), offset from OpenPathSensor or compass encoder.
        Combines absolute world heading with real-time adjustments for obstacles.
        '''
        # get base heading (set by behavior via set_heading_degrees())
        base_heading = self._heading_degrees % 360.0
        # get dynamic offset from OpenPathSensor (priority) or compass encoder (fallback)
        if self._open_path_sensor:
            offset = self._open_path_sensor.get_heading_offset()
        elif self._compass_encoder:
            self._compass_encoder.update()
            # map encoder 0-360° to offset range ±180°
            offset = (self._compass_encoder.get_degrees() - 180.0)
        else:
            offset = 0.0
        # combine base + offset for final world heading
        desired_heading = (base_heading + offset) % 360.0
        current_heading = self._imu.poll() % 360.0
        error = (desired_heading - current_heading + 180.0) % 360.0 - 180.0
        omega = self._calculate_omega(error)
        # let _update_linear_vector handle forward movement, then add rotation
        self._update_linear_vector()
        vx, vy, _ = self._intent_vector
        self._intent_vector = (vx, vy, omega)
        if self._verbose:
            self._log.info("BLENDED: base={:.2f}; offset={:.2f}; desired={:.2f}; current={:.2f}; omega={:.3f}".format(
                base_heading, offset, desired_heading, current_heading, omega))
            self._display_info('BLENDED')

    def _calculate_omega(self, error):
        '''
        Calculate rotation speed with rate limiting to prevent oscillation.
        '''
        abs_error = abs(error)
        # deadzone
        if abs_error <= self._rotation_tolerance:
            self._last_omega = 0.0
            return 0.0
        gain = 0.15 # gain indicates turning force
        target_omega = abs_error * gain
        # clamp to limits
        target_omega = max(min(target_omega, self._rotation_speed), 0.08)
        target_omega = target_omega * (1 if error > 0 else -1)
        # rate limit: smooth the change
        omega_change = target_omega - self._last_omega
        if abs(omega_change) > self._max_omega_change:
            omega = self._last_omega + self._max_omega_change * (1 if omega_change > 0 else -1)
        else:
            omega = target_omega
        self._last_omega = omega
        return omega

    def _update_linear_vector(self):
        '''
        Handles normal movement intent vector, including obstacle scaling and deadband.
        
        HeadingMode.NONE: Scales speed to zero at obstacles (legacy stop behavior)
        HeadingMode.RELATIVE/BLENDED: Maintains minimum speed for steering avoidance
        
        Produces forward or reverse motion in robot frame (vx=0, vy=amplitude, omega=0).
        Positive amplitude moves forward, negative moves reverse.
        The mode-specific methods (_update_intent_vector_*) add rotation as needed.

        Note that Scout does not handle obstacle avoidance except at the front of the
        robot.
        '''
        if self._motor_controller.braking_active:
            self._log.warning('braking active: intent vector suppressed')
            return
        amplitude = self._default_speed
        if self._digital_pot:
            amplitude = self._digital_pot.get_scaled_value(False)
        if isclose(amplitude, 0.0, abs_tol=0.01):
            self.clear_intent_vector()
            if self._verbose:
                self._display_info('update_linear_vector (stopped)')
            return
        # obstacle scaling only for forward motion
        if amplitude > 0.0:
            self._front_distance = self._roam_sensor.get_distance(apply_easing=True)
            if self._front_distance < 0.0:
                self._log.warning('braking: no long range distance available.')
                self._motor_controller.brake()
                return
            elif self._front_distance is None:
                self._log.info(Fore.BLUE + 'scout sensor returned: None; maintaining current vector')
                return
            elif self._front_distance >= self._max_distance:
                obstacle_scale = 1.0
            elif self._front_distance <= self._min_distance:
                # check if steering mode has a viable escape path
                if self._heading_mode in [HeadingMode.RELATIVE, HeadingMode.BLENDED]:
                    # get the distance at the open path heading
                    if self._open_path_sensor:
                        _, open_distance = self._open_path_sensor.get_heading_offset()
                        if open_distance > self._min_distance:
                            obstacle_scale = self._minimum_steering_speed / amplitude
                            self._log.debug('maintaining minimum steering speed: open path at {:.1f}mm'.format(open_distance))
                        else:
                            obstacle_scale = 0.0  # stop: no escape route
                            self._log.warning('stopping: open path only {:.1f}mm away'.format(open_distance))
                            # TODO come up with an escape plan, send a message for help
                    else:
                        obstacle_scale = 0.0
                else:
                    obstacle_scale = 0.0
            else:
                # scale between min and max distance
                obstacle_scale = (self._front_distance - self._min_distance) / (self._max_distance - self._min_distance)
                obstacle_scale = np.clip(obstacle_scale, 0.0, 1.0)
                # for steering modes, enforce minimum speed threshold
                if self._heading_mode in [HeadingMode.RELATIVE, HeadingMode.BLENDED]:
                    min_scale = self._minimum_steering_speed / amplitude
                    obstacle_scale = max(obstacle_scale, min_scale)
            amplitude *= obstacle_scale
        else:
            # reverse motion - no obstacle detection
            self._front_distance = 0.0
        # smooth amplitude changes
        max_amplitude_change = 0.10
        delta = amplitude - self._last_amplitude
        if abs(delta) > max_amplitude_change:
            amplitude = self._last_amplitude + (max_amplitude_change if delta > 0 else -max_amplitude_change)
        self._last_amplitude = amplitude
        # log every 10 cycles
        if next(self._counter) % 10 == 0:
            direction = "FWD" if amplitude >= 0.0 else "REV"
            self._log.info('{}: amp: {:.3f}; dist: {:.1f}mm'.format(direction, amplitude, self._front_distance))
        # deadband
        if abs(amplitude) < self._deadband_threshold:
            self.clear_intent_vector()
        else:
            vx = 0.0
            vy = amplitude
            omega = 0.0
            self._intent_vector = (vx, vy, omega)
        if self._verbose:
            self._display_info('update_linear_vector (active)')

    def _display_info(self, message=''):
        if self._use_color:
            if self._intent_vector[0] + self._intent_vector[1] == 0.0:
                self._log.info(Style.DIM + "{} intent vector: {}({:4.2f}, {:4.2f}, {:4.2f}); ".format(
                        message,
                        self.get_highlight_color(self._intent_vector[0]),
                        self._intent_vector[0], self._intent_vector[1], self._intent_vector[2])
                    + Fore.YELLOW + Style.NORMAL + 'distance: {:3.1f}mm'.format(self._front_distance))
            else:
                self._log.info("{} intent vector: {}({:4.2f}, {:4.2f}, {:4.2f}); ".format(
                        message,
                        self.get_highlight_color(self._intent_vector[0]),
                        self._intent_vector[0], self._intent_vector[1], self._intent_vector[2])
                    + Fore.YELLOW + Style.NORMAL + 'distance: {:3.1f}mm'.format(self._front_distance))
        else:
            self._log.info("intent vector: ({:.2f},{:.2f},{:.2f})".format(
                    self._intent_vector[0], self._intent_vector[1], self._intent_vector[2]))

    def get_highlight_color(self, value):
        from colorama import Fore, Style
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
        else:
            return Fore.MAGENTA

    def _accelerate(self):
        self._log.info("accelerate…")
        if self._motor_controller:
            self._motor_controller.accelerate(self._default_speed, enabled=lambda: not self._stop_event.is_set())

    def _decelerate(self):
        self._log.info("decelerate…")
        if self._motor_controller:
            self._motor_controller.decelerate(0.0, enabled=lambda: not self._stop_event.is_set())

    def enable(self):
        if self.enabled:
            self._log.warning("already enabled.")
            return
        self._log.info("enabling scout…")
        if not self._roam_sensor.enabled:
            self._roam_sensor.enable()
        if self._open_path_sensor and not self._open_path_sensor.enabled:
            self._open_path_sensor.enable()
        AsyncBehaviour.enable(self)
        # only auto-align for RELATIVE mode when using world coordinates
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
        if self._open_path_sensor and self._open_path_sensor.enabled:
            self._open_path_sensor.disable()
        AsyncBehaviour.disable(self)
        self._log.info(Fore.YELLOW + 'disabled.')

#EOF
