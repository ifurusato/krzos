#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:    Murray Altheim
# created:  2025-11-07
# modified: 2025-12-18

import time
from enum import Enum
import numpy as np
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.orientation import Orientation
from core.rotation import Rotation

class RotationPhase(Enum):
    '''
    Represents the phases of a rotation operation.
    '''
    INACTIVE  = (0, 'inactive')
    IDLE      = (1, 'idle')
    ACCEL     = (2, 'accel')
    ROTATE    = (3, 'rotate')
    DECEL     = (4, 'decel')

    def __init__(self, num, name):
        self._name = name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def name(self):
        return self._name

class RotationController(Component):
    NAME = 'rotation-ctrl'
    '''
    Provides encoder-based rotation control for the robot.

    Handles angle-based acceleration, constant-speed rotation, and deceleration
    to achieve precise total angular rotation. The rotate() method specifies the
    total rotation including accel and decel phases.

    Acceleration and deceleration use time-based ramping with configured maximum
    acceleration rate to protect motor gearboxes. Phase transitions use angle-based
    detection for precise positioning.

    Behaviors can register for phase change callbacks to coordinate with rotation phases.

    Intent vector is registered only during active rotation and removed immediately
    upon completion to prevent dangerous motor surges.
    '''
    def __init__(self, config, motor_controller, level=Level.INFO):
        self._log = Logger(RotationController.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        if motor_controller is None:
            raise ValueError('motor controller is required.')
        self._motor_controller = motor_controller
        # get motor references
        self._motor_pfwd = self._motor_controller.get_motor(Orientation.PFWD)
        self._motor_sfwd = self._motor_controller.get_motor(Orientation.SFWD)
        self._motor_paft = self._motor_controller.get_motor(Orientation.PAFT)
        self._motor_saft = self._motor_controller.get_motor(Orientation.SAFT)
        # configuration
        _cfg = config['kros'].get('rotation_controller')
        self._rotation_speed = _cfg.get('default_rotation_speed_deg_per_sec', 20.0)  # degrees/sec
        self._rotation_speed_rad = np.deg2rad(self._rotation_speed)
        # maximum acceleration rate (protects motor gearboxes)
        self._max_acceleration = _cfg.get('max_acceleration_deg_per_sec_sq', 15.0)  # deg/s²
        self._max_acceleration_rad = np.deg2rad(self._max_acceleration)
        # angle-based accel/decel distances
        self._accel_degrees = _cfg.get('accel_degrees', 20.0)  # degrees
        self._decel_degrees = _cfg.get('decel_degrees', 20.0)  # degrees
        # calculate time required to reach v_max with constant acceleration
        # v_max = a * t  →  t = v_max / a
        self._accel_time = self._rotation_speed / self._max_acceleration
        self._decel_time = self._rotation_speed / self._max_acceleration
        # empirical value from 10x 360° rotations on carpet with 11-roller mecanum wheels
        self._steps_per_degree = _cfg.get('steps_per_degree', 6.256)
        self._log.info('rotation speed: {:.1f}°/sec'.format(self._rotation_speed))
        self._log.info('max acceleration: {:.1f}°/s²'.format(self._max_acceleration))
        self._log.info('accel:  {:.1f}° in {:.2f}s'.format(self._accel_degrees, self._accel_time))
        self._log.info('decel: {:.1f}° in {:.2f}s'.format(self._decel_degrees, self._decel_time))
        self._log.info('steps_per_degree: {:.3f}'.format(self._steps_per_degree))
        # rotation state variables
        self._rotation_phase = RotationPhase.INACTIVE
        self._rotation_direction = Rotation.CLOCKWISE
        self._intent_vector = (0.0, 0.0, 0.0)
        self._priority = 0.0
        self._intent_vector_registered = False
        # callbacks
        self._poll_callbacks = []
        self._phase_change_callbacks = []
        # encoder baselines for rotation tracking
        self._baseline_pfwd = 0
        self._baseline_sfwd = 0
        self._baseline_paft = 0
        self._baseline_saft = 0
        self._start_time = None
        # rotation tracking across phases
        self._total_target_rotation = 0.0  # total rotation requested (including accel/decel)
        self._rotate_target   = 0.0        # target for constant-speed ROTATE phase
        self._accel_rotation  = 0.0        # actual degrees rotated during accel
        self._rotate_rotation = 0.0        # actual degrees rotated during rotate
        # rotation state
        self._current_omega   = 0.0
        self._omega_increment = 0.0
        # cumulative heading state
        self._current_heading_offset = 0.0
        self._heading_markers = []
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def is_rotating(self):
        '''
        Returns True if currently executing a rotation.
        '''
        return self._rotation_phase not in (RotationPhase.INACTIVE, RotationPhase.IDLE)

    @property
    def rotation_phase(self):
        '''
        Returns the current rotation phase.
        '''
        return self._rotation_phase

    @property
    def rotation_direction(self):
        '''
        Returns the current rotation direction.
        '''
        return self._rotation_direction

    @property
    def intent_vector(self):
        '''
        Returns the current intent vector (for debugging/monitoring).
        '''
        return self._intent_vector

    @property
    def accel_degrees(self):
        '''
        Returns the configured acceleration distance in degrees.
        '''
        return self._accel_degrees

    @property
    def decel_degrees(self):
        '''
        Returns the configured deceleration distance in degrees.
        '''
        return self._decel_degrees

    def add_poll_callback(self, callback):
        '''
        Register a callback to be invoked on every poll() call during active rotation.
        Callback signature: callback()
        '''
        if not callable(callback):
            raise TypeError('callback must be callable')
        self._poll_callbacks.append(callback)
        self._log.info('added poll callback')

    def remove_poll_callback(self, callback):
        '''
        Remove a previously registered poll callback.
        '''
        if callback in self._poll_callbacks:
            self._poll_callbacks. remove(callback)
            self._log.info('removed poll callback')

    def add_phase_change_callback(self, callback):
        '''
        Register a callback to be notified of phase transitions.
        Callback signature: callback(old_phase, new_phase)
        '''
        if not callable(callback):
            raise TypeError('callback must be callable')
        self._phase_change_callbacks.append(callback)
        self._log.info('added phase change callback')

    def remove_phase_change_callback(self, callback):
        '''
        Remove a previously registered phase change callback.
        '''
        if callback in self._phase_change_callbacks:
            self._phase_change_callbacks.remove(callback)
            self._log.info('removed phase change callback')

    def _notify_phase_change(self, old_phase, new_phase):
        '''
        Notify all registered callbacks of a phase transition.
        '''
        for callback in self._phase_change_callbacks:
            try:
                callback(old_phase, new_phase)
            except Exception as e:
                self._log.error('error in phase change callback: {}'.format(e))

    def get_current_heading(self):
        '''
        Returns current relative heading (degrees from startup/reset).
        '''
        if self._rotation_phase != RotationPhase.IDLE and self._rotation_phase != RotationPhase.INACTIVE:
            current_rotation = self._get_accumulated_rotation()
            return (self._current_heading_offset + current_rotation) % 360.0
        return self._current_heading_offset

    def push_heading_marker(self, label=None):
        '''
        Save current heading as a reference point.
        Returns marker ID for later retrieval.
        '''
        current = self.get_current_heading()
        marker = {
            'label': label,
            'heading': current,
            'timestamp': time.time()
        }
        self._heading_markers.append(marker)
        marker_id = len(self._heading_markers) - 1
        self._log.debug('pushed heading marker "{}": {:.1f}° (id={})'.format(
            label, current, marker_id))
        return marker_id

    def get_heading_marker(self, marker_id):
        '''
        Retrieve a previously saved heading marker value.
        '''
        if 0 <= marker_id < len(self._heading_markers):
            return self._heading_markers[marker_id]['heading']
        raise IndexError('heading marker id {} out of range'.format(marker_id))

    def reset_heading(self):
        '''
        Reset cumulative heading to zero and clear all markers.
        '''
        self._current_heading_offset = 0.0
        self._heading_markers.clear()
        self._log.info('heading reset to 0.0°')

    def rotate_absolute(self, target_heading_degrees):
        '''
        Rotate to exactly the specified compass heading in degrees, choosing
        the direction of rotation that requires the least amount of rotation.

        This uses the ICM20948 to determine current heading and track progress.
        If not available, raises a MissingComponentError.

        Args:
            target_heading_degrees: Target compass heading (0-359°)

        Returns:
            True if rotation completed successfully, False if cancelled/timed out
        '''
        self._log.info(Fore.MAGENTA + '💜 absolute rotation to {:.1f}°…'.format(target_heading_degrees))
        from hardware.icm20948 import Icm20948
        from behave.behaviour_manager import BehaviourManager

        _component_registry = Component.get_registry()
        _icm20948 = _component_registry.get(Icm20948.NAME)
        if not _icm20948:
            raise MissingComponentError('icm20948 required for absolute rotation.')
        if not _icm20948.is_calibrated:
            raise Exception('icm20948 must be calibrated before absolute rotation.')
        # suppress behaviours
        _behaviour_manager = _component_registry.get(BehaviourManager.NAME)
        if _behaviour_manager:
            _behaviour_manager.suppress_all_behaviours()
        # get current heading from IMU
        _current_heading = _icm20948.mean_heading
        # calculate shortest rotation direction and degrees
        _delta = (target_heading_degrees - _current_heading + 180) % 360 - 180
        _rotation_degrees = abs(_delta)
        _direction = Rotation.CLOCKWISE if _delta > 0 else Rotation.COUNTER_CLOCKWISE
        # validate rotation is large enough for accel/decel
        if _rotation_degrees < (self._accel_degrees + self._decel_degrees):
            self._log.warning('rotation of {:.1f}° too small, adjusting to minimum'.format(_rotation_degrees))
            _rotation_degrees = self._accel_degrees + self._decel_degrees + 10.0
        self._log.info('rotating from {}° to {}° ({:.1f}° {})'.format(
            _current_heading, target_heading_degrees, _rotation_degrees, _direction.name))
        # perform rotation using existing blocking method
        _success = self.rotate_blocking(_rotation_degrees, _direction)
        # verify we reached target (within tolerance)
        _final_heading = _icm20948.mean_heading
        _error = abs((_final_heading - target_heading_degrees + 180) % 360 - 180)
        if _error > 5.0: # tolerance: 5°
            self._log.warning('rotation completed but heading error: {:.1f}° (target: {}°, actual: {}°)'.format(
                _error, target_heading_degrees, _final_heading))
        else:
            self._log.info('rotation complete: target: {}°, actual: {}°, error: {:.1f}°'.format(
                target_heading_degrees, _final_heading, _error))
        if _behaviour_manager:
            _behaviour_manager.release_all_behaviours()
        return _success and _error <= 5.0

    def rotate(self, degrees, direction=Rotation.CLOCKWISE):
        '''
        Rotate exactly the specified total degrees in the given direction.
        Total rotation (including accel/decel) = degrees.

        Args:
            degrees:    Total rotation angle (including accel and decel phases)
            direction:  Rotation.CLOCKWISE or Rotation.COUNTER_CLOCKWISE
        '''
        if degrees <= (self._accel_degrees + self._decel_degrees):
            raise ValueError('rotation {} too small for accel ({}) + decel ({})'.format(
                degrees, self._accel_degrees, self._decel_degrees))
        self._total_target_rotation = degrees
        self._rotate_target = degrees - self._accel_degrees - self._decel_degrees
        self._rotation_direction = direction
        self._log.info(Fore.GREEN + 'initiating {:.1f}° rotation {} (accel={:.1f}°, rotate={:.1f}°, decel={:.1f}°)'.format(
            degrees, direction.name, self._accel_degrees, self._rotate_target, self._decel_degrees))
        old_phase = self._rotation_phase
        self._rotation_phase = RotationPhase.ACCEL
        self._start_time = time.time()
        self._accel_rotation = 0.0
        self._rotate_rotation = 0.0

        # rotation state variables
        # calculate omega increment per poll cycle
        # we want to reach rotation_speed_rad over accel_time
        poll_freq = 20  # Hz
        accel_ticks = int(self._accel_time * poll_freq)
        self._omega_increment = self._rotation_speed_rad / accel_ticks
        self._current_omega = 0.0

        # notify phase change
        self._notify_phase_change(old_phase, self._rotation_phase)
        # initialize encoder baselines
        self._baseline_pfwd = self._motor_pfwd.steps
        self._baseline_sfwd = self._motor_sfwd.steps
        self._baseline_paft = self._motor_paft.steps
        self._baseline_saft = self._motor_saft.steps
        # register intent vector with motor controller
        if not self._intent_vector_registered:
            self._priority = 1.0
            self._motor_controller.add_intent_vector(
                RotationController.NAME,
                lambda: self._intent_vector,
                lambda: self._priority
            )
            self._intent_vector_registered = True
            self._log.info('intent vector registered with motor controller')
        self._log.info('encoder baselines:  PFWD={}, SFWD={}, PAFT={}, SAFT={}'.format(
            self._baseline_pfwd, self._baseline_sfwd, self._baseline_paft, self._baseline_saft))

    def rotate_blocking(self, degrees, direction=Rotation.CLOCKWISE, poll_rate_hz=20):
        '''
        Synchronously rotate the specified degrees. Blocks until rotation completes.
        This is a convenience method that handles the phase loop internally.

        Args:
            degrees: Total rotation angle (including accel and decel phases)
            direction:  Rotation.CLOCKWISE or Rotation.COUNTER_CLOCKWISE
            poll_rate_hz:  Polling frequency for phase updates

        Returns:
            True if rotation completed successfully, False if cancelled/timed out
        '''
        # initiate rotation
        self.rotate(degrees, direction)

        # polling loop
        _poll_interval = 1.0 / poll_rate_hz
        _timeout = 60.0  # safety timeout in seconds
        _start = time.time()

        while self.is_rotating:
            # safety timeout
            if time.time() - _start > _timeout:
                self._log.error('rotation timeout after {:.1f}s'.format(_timeout))
                self.cancel_rotation()
                return False

            # poll and get current state
            current_time, elapsed, accumulated_rotation = self.poll()

            # handle current phase
            if self._rotation_phase == RotationPhase.ACCEL:
                self.handle_accel_phase(elapsed, accumulated_rotation, current_time)

            elif self._rotation_phase == RotationPhase.ROTATE:
                self.handle_rotate_phase(accumulated_rotation, current_time)

            elif self._rotation_phase == RotationPhase.DECEL:
                decel_complete = self.handle_decel_phase(elapsed, accumulated_rotation)
                if decel_complete:
                    break

            # wait for next poll cycle
            time.sleep(_poll_interval)

        return self._rotation_phase == RotationPhase.IDLE

    def _get_accumulated_rotation(self):
        '''
        Calculate accumulated rotation in degrees from encoder deltas.
        Averages all four mecanum wheels for robust tracking.
        Accounts for rotation direction.
        '''
        current_pfwd = self._motor_pfwd.steps
        current_sfwd = self._motor_sfwd.steps
        current_paft = self._motor_paft.steps
        current_saft = self._motor_saft.steps
        # for CW rotation:  port wheels go forward (+), starboard wheels go backward (-)
        # for CCW rotation:  port wheels go backward (-), starboard wheels go forward (+)
        if self._rotation_direction == Rotation.CLOCKWISE:
            rotation_steps = ((current_pfwd - self._baseline_pfwd) +
                              (current_paft - self._baseline_paft) -
                              (current_sfwd - self._baseline_sfwd) -
                              (current_saft - self._baseline_saft)) / 4.0
        else:  # COUNTER_CLOCKWISE
            rotation_steps = ((current_sfwd - self._baseline_sfwd) +
                              (current_saft - self._baseline_saft) -
                              (current_pfwd - self._baseline_pfwd) -
                              (current_paft - self._baseline_paft)) / 4.0
        degrees = rotation_steps / self._steps_per_degree
        return degrees

    def poll(self):
        '''
        Execute one rotation control step. Call repeatedly from async loop.
        Returns:  (current_time, elapsed, accumulated_rotation)
        '''
        if self._rotation_phase == RotationPhase.INACTIVE or self._rotation_phase == RotationPhase.IDLE:
            return (time.time(), 0.0, 0.0)
        current_time = time.time()
        elapsed = current_time - self._start_time
        accumulated_rotation = self._get_accumulated_rotation()
        # call all registered poll callbacks
        for callback in self._poll_callbacks:
            try:
                callback()
            except Exception as e:
                self._log.error('error in poll callback: {}'.format(e))
        return (current_time, elapsed, accumulated_rotation)

    def handle_accel_phase(self, elapsed, accumulated_rotation, current_time):
        '''
        Handle acceleration phase:  ramp up to full rotation speed.
        Uses configured max_acceleration rate, incremented each poll cycle.
        '''
        # Calculate omega increment per poll cycle using max_acceleration
        # max_acceleration is in deg/s², poll cycle is ~0.05s (20Hz)
        poll_interval = 0.05  # seconds (20Hz from config)
        omega_increment_rad = self._max_acceleration_rad * poll_interval
        # increment omega each tick up to max speed
        if self._current_omega < self._rotation_speed_rad:
            self._current_omega += omega_increment_rad
            if self._current_omega > self._rotation_speed_rad:
                self._current_omega = self._rotation_speed_rad
        omega = self._current_omega
        # apply direction
        if self._rotation_direction == Rotation.COUNTER_CLOCKWISE:
            omega = -omega
        self._intent_vector = (0.0, 0.0, omega)
        # angle-based transition detection
        if accumulated_rotation >= self._accel_degrees:
            old_phase = self._rotation_phase
            self._rotation_phase = RotationPhase.ROTATE
            self._start_time = current_time
            self._accel_rotation = accumulated_rotation
            self._baseline_pfwd = self._motor_pfwd.steps
            self._baseline_sfwd = self._motor_sfwd.steps
            self._baseline_paft = self._motor_paft.steps
            self._baseline_saft = self._motor_saft.steps
            self._log.info('acceleration complete at {:.1f}°, starting constant rotation'.format(self._accel_rotation))
            self._notify_phase_change(old_phase, self._rotation_phase)

    def handle_rotate_phase(self, accumulated_rotation, current_time):
        '''
        Handle rotate phase: maintain constant rotation speed for rotate_target degrees.
        '''
        # maintain constant rotation speed
        omega = self._rotation_speed_rad
        # apply direction
        if self._rotation_direction == Rotation.COUNTER_CLOCKWISE:
            omega = -omega
        self._intent_vector = (0.0, 0.0, omega)
        # check if we've completed the target rotation
        if accumulated_rotation >= self._rotate_target:
            old_phase = self._rotation_phase
            self._rotation_phase = RotationPhase.DECEL
            self._start_time = time.time()
            self._rotate_rotation = accumulated_rotation
            self._baseline_pfwd = self._motor_pfwd.steps
            self._baseline_sfwd = self._motor_sfwd.steps
            self._baseline_paft = self._motor_paft.steps
            self._baseline_saft = self._motor_saft.steps
            total_so_far = self._accel_rotation + self._rotate_rotation
            self._log.info('starting deceleration at {:.1f}° (rotate target:  {:.1f}°, total target: {:.1f}°)'.format(
                total_so_far, self._rotate_target, self._total_target_rotation))
            self._notify_phase_change(old_phase, self._rotation_phase)
            return True
        return False

    def handle_decel_phase(self, elapsed, accumulated_rotation):
        '''
        Handle deceleration phase: ramp down to stop.
        Uses time-based ramping with configured max acceleration rate.
        Completes based on angle or when omega reaches zero.
        Returns:  True if deceleration complete, False otherwise
        '''
        # time-based deceleration with max rate limit
        # omega = v_max - a * t, floored at 0
        omega = max(self._rotation_speed_rad - self._max_acceleration_rad * elapsed, 0.0)
        # apply direction
        if self._rotation_direction == Rotation.COUNTER_CLOCKWISE:
            omega = -omega
        self._intent_vector = (0.0, 0.0, omega)
        # complete when omega reaches zero OR angle target reached
        if omega == 0.0 or accumulated_rotation >= self._decel_degrees:
            # stop - zero intent vector
            self._intent_vector = (0.0, 0.0, 0.0)
            # remove from MotorController immediately
            if self._intent_vector_registered:
                self._motor_controller.remove_intent_vector(RotationController.NAME)
                self._intent_vector_registered = False
                self._log.info('intent vector removed (rotation complete)')
            # calculate total rotation:  accel + rotate + decel
            total_rotation = self._accel_rotation + self._rotate_rotation + accumulated_rotation
            self._log.info('rotation complete:  accel={:.1f}°, rotate={:.1f}°, decel={:.1f}°, total={:.1f}° (target={:.1f}°, error={:.1f}°)'.format(
                self._accel_rotation, self._rotate_rotation, accumulated_rotation,
                total_rotation, self._total_target_rotation, total_rotation - self._total_target_rotation))
            # update cumulative heading (account for direction)
            if self._rotation_direction == Rotation.CLOCKWISE:
                self._current_heading_offset = (self._current_heading_offset + total_rotation) % 360.0
            else:
                self._current_heading_offset = (self._current_heading_offset - total_rotation) % 360.0
            # transition to idle
            old_phase = self._rotation_phase
            self._rotation_phase = RotationPhase.IDLE
            # notify phase change
            self._notify_phase_change(old_phase, self._rotation_phase)
            return True
        return False

    def clear_intent_vector(self):
        '''
        Zero the intent vector.
        '''
        self._intent_vector = (0.0, 0.0, 0.0)

    def cancel_rotation(self):
        '''
        Immediately cancel current rotation.
        '''
        if self._rotation_phase != RotationPhase.IDLE and self._rotation_phase != RotationPhase.INACTIVE:
            self._log.warning('rotation cancelled')
            # zero intent vector first
            self._intent_vector = (0.0, 0.0, 0.0)
            # remove from motor controller
            if self._intent_vector_registered:
                self._motor_controller.remove_intent_vector(RotationController.NAME)
                self._intent_vector_registered = False
                self._log.info('intent vector removed (cancelled)')
            old_phase = self._rotation_phase
            self._rotation_phase = RotationPhase.IDLE
            # notify phase change
            self._notify_phase_change(old_phase, self._rotation_phase)

    def enable(self):
        if not self.enabled:
            Component.enable(self)
            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def disable(self):
        '''
        Disable the rotation controller.
        '''
        if self.enabled:
            self._log.debug('disabling rotation controller…')
            # cancel any active rotation
            if self._rotation_phase != RotationPhase.IDLE and self._rotation_phase != RotationPhase.INACTIVE:
                self.cancel_rotation()
            # zero intent vector
            self._intent_vector = (0.0, 0.0, 0.0)
            # unregister from motor controller
            if self._intent_vector_registered:
                try:
                    self._motor_controller.remove_intent_vector(RotationController.NAME)
                    self._log.info('intent vector removed (disable)')
                except Exception as e:
                    self._log.warning('could not remove intent vector: {} (may have been removed by MotorController)'.format(e))
                self._intent_vector_registered = False
            self._rotation_phase = RotationPhase.INACTIVE
            super().disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

    def close(self):
        if not self.closed:
            super().close()
            self._log.info('closed.')
        else:
            self._log.warning('already closed.')

#EOF
