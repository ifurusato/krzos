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
        self._use_world_coordinates = _cfg.get('use_world_coordinates')
        # heading control
        self._heading_degrees = 0.0
        self._target_relative_offset = 0.0
        self._rotation_speed = _cfg.get('rotation_speed', 0.5)
        self._rotation_tolerance = _cfg.get('rotation_tolerance', 2.0)
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

    def x_dynamic_set_heading(self):
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
                self.set_heading_degrees(_degrees)
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
            case _:
                raise NotImplementedError("unhandled heading mode: {}".format(self._heading_mode))

    def _update_intent_vector_relative(self):
        '''
        RELATIVE mode: Servo to encoder-specified heading using motor encoder feedback.
        ScoutSensor provides obstacle avoidance steering.
        '''
        # update actual heading from motor encoders
        self._update_heading_from_encoders()
        current_heading = self._heading_degrees
        # get obstacle avoidance steering from ScoutSensor
        if self._scout_sensor:
            scout_offset, open_distance = self._scout_sensor.get_heading_offset()
            print("RAW SENSOR: offset={:+.2f}°, distance={:.1f}mm".format(scout_offset, open_distance))
        else:
            scout_offset = 0.0
            open_distance = None
        # combine target offset (from compass encoder) with scout steering
        desired_heading = (self._target_relative_offset + scout_offset) % 360.0
        # calculate error
        error = (desired_heading - current_heading + 180.0) % 360.0 - 180.0
        omega = self._calculate_omega(error, open_distance)
        # get forward movement vector, then add rotation
        self._update_linear_vector()
        vx, vy, _ = self._intent_vector
        self._intent_vector = (vx, vy, omega)
        if self._verbose:
            print("RELATIVE: target={:.2f}°, current={:.2f}°, scout={:+.2f}°, error={:.2f}°; omega={:.3f}{}".format(
                self._target_relative_offset, current_heading, scout_offset, error, omega,
                "; open_dist={:.1f}mm".format(open_distance) if open_distance else ""))
            self._display_info('RELATIVE')

    def b_update_intent_vector_relative(self):
        '''
        RELATIVE mode: Servo to encoder-specified heading using motor encoder feedback.
        OpenPathSensor provides obstacle avoidance steering if available.
        '''
        # update actual heading from motor encoders
        self._update_heading_from_encoders()
        current_heading = self._heading_degrees  # this comes from motor encoders
        # get obstacle avoidance steering from OpenPathSensor
        if self._open_path_sensor and self._front_distance < self._max_distance:
            steering_offset, open_distance = self._open_path_sensor.get_heading_offset()
        else:
            steering_offset = 0.0
            open_distance = self._max_distance
        # combine target offset (from compass encoder) with steering offset
        desired_heading = (self._target_relative_offset + steering_offset) % 360.0
        # calculate error
        error = (desired_heading - current_heading + 180.0) % 360.0 - 180.0
        omega = self._calculate_omega(error)
        # get forward movement vector, then add rotation
        self._update_linear_vector()
        vx, vy, _ = self._intent_vector
        self._intent_vector = (vx, vy, omega)
        if self._verbose:
            self._log.info("RELATIVE: target={:.2f}°; current={:.2f}°; steering={:+.2f}°; error={:.2f}°; omega={:.3f}; dist={:.1f}mm".format(
                self._target_relative_offset, current_heading, steering_offset, error, omega, open_distance))
            self._display_info('RELATIVE')

    def _update_heading_from_encoders(self):
        '''
        Update self._heading_degrees based on actual motor encoder deltas.
        Tracks rotation by measuring differential encoder movement.
        Used by RELATIVE mode to maintain awareness of actual robot orientation.
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
        # calculate rotation (Mecanum kinematics for in-place rotation)
        rotation_steps = (delta_pfwd + delta_paft - delta_sfwd - delta_saft) / 4.0
        degrees_rotated = rotation_steps / self._steps_per_degree
        # update heading
        self._heading_degrees = (self._heading_degrees + degrees_rotated) % 360.0
        # store for next iteration
        self._last_encoder_pfwd = current_pfwd
        self._last_encoder_sfwd = current_sfwd
        self._last_encoder_paft = current_paft
        self._last_encoder_saft = current_saft

    def x_update_intent_vector_relative(self):
        '''
        relative mode: rotate to face the direction scoutsensor indicates,
        plus encoder offset.
        '''
        # scoutsensor provides obstacle avoidance offset
        if self._scout_sensor:
            scout_offset, open_distance = self._scout_sensor.get_heading_offset()
            print("RAW SENSOR: offset={:+.2f}°, distance={:.1f}mm".format(scout_offset, open_distance))
        else:
            scout_offset = 0.0
            open_distance = None
        
        # use encoder offset (updated every 5 cycles by _dynamic_set_heading)
        # convert to -180 to +180 range
        encoder_offset = ((self._heading_degrees + 180.0) % 360.0 - 180.0)
        
        # combine sensor avoidance with encoder offset
        error = scout_offset + encoder_offset
        omega = self._calculate_omega(error, open_distance)
        
        vx = 0.0
        vy = 0.0
        self._intent_vector = (vx, vy, omega)
        
        if self._verbose:
            print("RELATIVE: scout={:+.2f}°, encoder={:+.2f}°, error={:+.2f}°; omega={:.3f}{}".format(
                scout_offset, encoder_offset, error, omega,
                "; open_dist={:.1f}mm".format(open_distance) if open_distance else ""))
            self._display_info('RELATIVE')

    def z_update_intent_vector_relative(self):
        '''
        relative mode: rotate to face the direction scout sensor indicates.
        heading_degrees provides additional bias from encoder.
        '''
        # scout sensor provides the target direction as an offset
        if self._scout_sensor:
            scout_offset, open_distance = self._scout_sensor.get_heading_offset()
            print("RAW SENSOR: offset={:+.2f}°, distance={:.1f}mm".format(scout_offset, open_distance))
        else:
            scout_offset = 0.0
            open_distance = None
        # convert heading_degrees to relative offset (-180 to +180)
        heading_offset = ((self._heading_degrees + 180.0) % 360.0 - 180.0)
        # combine sensor offset with heading offset
        error = scout_offset + heading_offset
        omega = self._calculate_omega(error, open_distance)
        # scout only rotates - no forward or lateral motion
        vx = 0.0
        vy = 0.0
        self._intent_vector = (vx, vy, omega)
        if self._verbose:
            print("RELATIVE: scout={:+.2f}°, heading={:+.2f}°, error={:+.2f}°; omega={:.3f}{}".format(
                scout_offset, heading_offset, error, omega,
                "; open_dist={:.1f}mm".format(open_distance) if open_distance else ""))
            self._display_info('RELATIVE')

    def _update_intent_vector_absolute(self):
        '''
        ABSOLUTE mode: absolute compass heading with dynamic offset adjustments.
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
            self._log.info("ABSOLUTE: base={:.2f}°; offset={:+.2f}°; desired={:.2f}°; current={:.2f}°; error={:.2f}°; omega={:.3f}{}".format(
                base_heading, offset, desired_heading, current_heading, error, omega,
                "; open_dist={:.1f}mm".format(open_distance) if open_distance else ""))
            self._display_info('ABSOLUTE')

    def _calculate_omega(self, error, obstacle_distance=None):
        '''
        calculate rotation speed with rate limiting to prevent oscillation.
        allows faster response when crossing through target (error sign change).
        applies urgency scaling based on obstacle proximity.
        '''
        abs_error = abs(error)
        # deadzone
        if abs_error <= self._rotation_tolerance:
            self._last_omega = 0.0
            return 0.0
        # base gain
        gain = 0.05
        # apply urgency multiplier based on obstacle distance
        if obstacle_distance is not None:    
            if obstacle_distance < 300:      # very close
                gain *= 3.0
            elif obstacle_distance < 500:    # close
                gain *= 2.0
            elif obstacle_distance < 700:    # moderate
                gain *= 1.5
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

    def x_calculate_omega(self, error):
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
