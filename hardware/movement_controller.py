#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-20
# modified: 2026-03-12

import time
from enum import Enum
from math import isclose, sqrt
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level
from core.orientation import Orientation
from core.direction import Direction
from core.rotation import Rotation
from hardware.rotation_controller import RotationController

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class MovementPhase(Enum):
    '''
    Represents the phases of a movement operation.
    '''
    INACTIVE  = ( 0, 'inactive' )
    IDLE      = ( 1, 'idle'     )
    ACCEL     = ( 2, 'accel'    )
    MOVE      = ( 3, 'move'     )
    DECEL     = ( 4, 'decel'    )
    ROTATING  = ( 5, 'rotating' )  # delegated to RotationController

    def __init__(self, num, name):
        self._name = name

    @property
    def name(self):
        return self._name

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class MovementController(Component):
    NAME = 'movement-ctrl'
    '''
    Provides encoder-based linear movement control for the robot.

    Handles distance-based acceleration, constant-speed movement, and deceleration
    to achieve precise linear displacement. Supports forward, backward, and lateral
    (strafing) motion for Mecanum-wheeled robots.

    Can also incorporate rotational movements by delegating to RotationController.

    Accel and decel distances are computed per-movement as a ratio of the total
    distance, subject to a configured minimum. Both phases use a square-root
    velocity profile derived from remaining distance, giving a kinematically
    correct ramp that naturally approaches zero at the phase boundary.

    Intent vector is registered only during active movement and removed upon
    completion. The run() method drives the full movement loop internally.
    '''
    def __init__(self, config, motor_controller, rotation_controller=None, level=Level.INFO):
        self._log = Logger(MovementController.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        if motor_controller is None:
            raise ValueError('motor_controller cannot be None')
        self._motor_controller = motor_controller
        # get odometer for distance tracking
        self._odometer = self._motor_controller.odometer
        if self._odometer is None:
            raise ValueError('odometer not available from motor controller')
        # rotation controller for rotational movements
        self._rotation_controller = rotation_controller
        if self._rotation_controller:
            self._log.info('rotation controller available for combined movements')
        # configuration
        _cfg = config['kros'].get('movement_controller')
        self._movement_speed   = _cfg.get('default_speed', 200.0)    # mm/sec
        self._max_acceleration = _cfg.get('max_acceleration', 150.0) # mm/s²
        # phase distance is computed as this ratio of total distance, subject to minimum
        self._phase_ratio      = _cfg.get('phase_ratio', 0.10)        # 10% each for accel and decel
        self._min_phase_mm     = _cfg.get('min_phase_mm', 100.0)      # minimum phase distance in mm
        # minimum velocity floor during decel to keep intent vector non-zero until distance reached
        self._decel_min_speed  = _cfg.get('decel_min_speed', 10.0)    # mm/sec
        self._poll_rate_hz     = _cfg.get('poll_rate_hz', 20)
        self._log.info('movement speed: {:.1f}mm/sec'.format(self._movement_speed))
        self._log.info('max acceleration: {:.1f}mm/s²'.format(self._max_acceleration))
        self._log.info('phase ratio: {:.2f}; min phase: {:.1f}mm'.format(self._phase_ratio, self._min_phase_mm))
        # movement state
        self._movement_phase     = MovementPhase.INACTIVE
        self._movement_direction = Direction.STOPPED
        self._intent_vector      = (0.0, 0.0, 0.0)
        self._priority           = 0.0
        self._intent_vector_registered = False
        # phase change callbacks
        self._phase_change_callbacks = []
        # baseline pose for movement tracking
        self._baseline_x = 0.0
        self._baseline_y = 0.0
        self._start_time = None
        # movement tracking across phases (set per-movement in move())
        self._total_target_distance = 0.0
        self._accel_distance_mm     = 0.0  # computed per-movement
        self._decel_distance_mm     = 0.0  # computed per-movement
        self._move_target           = 0.0
        self._accel_distance        = 0.0  # actual distance covered during accel
        self._move_distance         = 0.0  # actual distance covered during move
        self._log.info('ready.')

    @property
    def motor_controller(self):
        '''
        Returns the backing MotorController.
        '''
        return self._motor_controller

    @property
    def is_moving(self):
        '''
        Returns True if currently executing a movement.
        '''
        return self._movement_phase not in (MovementPhase.INACTIVE, MovementPhase.IDLE)

    @property
    def movement_phase(self):
        '''
        Returns the current movement phase.
        '''
        return self._movement_phase

    @property
    def movement_direction(self):
        '''
        Returns the current movement direction.
        '''
        return self._movement_direction

    @property
    def intent_vector(self):
        '''
        Returns the current intent vector (for debugging/monitoring).
        '''
        return self._intent_vector

    @property
    def accel_distance_mm(self):
        '''
        Returns the accel distance for the current movement (computed per-movement).
        '''
        return self._accel_distance_mm

    @property
    def decel_distance_mm(self):
        '''
        Returns the decel distance for the current movement (computed per-movement).
        '''
        return self._decel_distance_mm

    def add_phase_change_callback(self, callback):
        '''
        Register a callback to be notified of phase transitions.
        Callback signature: callback(prev_phase, new_phase)
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

    def _notify_phase_change(self, prev_phase, new_phase):
        '''
        Notify all registered callbacks of a phase transition.
        '''
        for callback in self._phase_change_callbacks:
            try:
                callback(prev_phase, new_phase)
            except Exception as e:
                self._log.error('error in phase change callback: {}'.format(e))

    def _reset_baseline(self):
        '''
        Reset the odometer baseline to the current pose.
        '''
        x, y, theta = self._odometer.pose
        self._baseline_x = x
        self._baseline_y = y

    def _get_accumulated_distance(self):
        '''
        Calculate accumulated distance in mm from odometer pose changes.
        Distance is projected onto the movement direction axis.
        '''
        x, y, theta = self._odometer.pose
        dx = x - self._baseline_x
        dy = y - self._baseline_y
        if self._movement_direction == Direction.AHEAD:
            distance = dy
        elif self._movement_direction == Direction.ASTERN:
            distance = -dy
        elif self._movement_direction == Direction.STARBOARD:
            distance = dx
        elif self._movement_direction == Direction.PORT:
            distance = -dx
        else:
            distance = 0.0
        return abs(distance)

    def _set_intent(self, normalized_velocity):
        '''
        Set the intent vector from a normalised velocity (0.0–1.0).
        '''
        vx = self._movement_direction.vx_direction * normalized_velocity
        vy = self._movement_direction.vy_direction * normalized_velocity
        self._intent_vector = (vx, vy, 0.0)

    def _register_intent_vector(self):
        '''
        Register the intent vector with the motor controller if not already registered.
        '''
        if not self._intent_vector_registered:
            self._priority = 1.0
            self._motor_controller.add_intent_vector(
                MovementController.NAME,
                lambda: self._intent_vector,
                lambda: self._priority
            )
            self._intent_vector_registered = True
            self._log.info('intent vector registered with motor controller')

    def remove_intent_vector(self):
        '''
        Zero and remove the intent vector from the motor controller.
        '''
        self._intent_vector = (0.0, 0.0, 0.0)
        if self._intent_vector_registered:
            self._motor_controller.remove_intent_vector(MovementController.NAME)
            self._intent_vector_registered = False
            self._log.info('intent vector removed')

    def move(self, distance_mm, direction=Direction.AHEAD):
        '''
        Initiate movement of exactly distance_mm in the given direction.
        Phase transitions are determined dynamically from odometer and slew state.
        '''
        self._total_target_distance = distance_mm
        self._movement_direction    = direction
        self._accel_distance        = 0.0
        self._move_distance         = 0.0
        self._log.info(Fore.GREEN + 'initiating {:.1f}mm movement {}'.format(distance_mm, direction.label))
        prev_phase = self._movement_phase
        self._movement_phase = MovementPhase.ACCEL
        self._start_time = time.time()
        self._reset_baseline()
        self._register_intent_vector()
        self._notify_phase_change(prev_phase, self._movement_phase)
        self._log.info('baseline pose: x={:.1f}mm, y={:.1f}mm'.format(self._baseline_x, self._baseline_y))

    def run(self, distance_mm, direction=Direction.AHEAD, poll_rate_hz=None):
        '''
        Synchronously execute a complete movement: accel, move, decel.
        Blocks until the movement is complete and motors are stopped.
        '''
        if poll_rate_hz is None:
            poll_rate_hz = self._poll_rate_hz
        _poll_delay = 1.0 / poll_rate_hz
        self.move(distance_mm, direction)
        while self._movement_phase in (MovementPhase.ACCEL, MovementPhase.MOVE, MovementPhase.DECEL):
            current_time = time.time()
            elapsed      = current_time - self._start_time
            accumulated  = self._get_accumulated_distance()
            phase = self._movement_phase
            if phase == MovementPhase.ACCEL:
                self.handle_accel_phase(accumulated, current_time)
            elif phase == MovementPhase.MOVE:
                self.handle_move_phase(accumulated, current_time)
            elif phase == MovementPhase.DECEL:
                self.handle_decel_phase(accumulated)
            time.sleep(_poll_delay)
        self._log.info(Fore.GREEN + 'run complete.')

    def z_run(self, distance_mm, direction=Direction.AHEAD, poll_rate_hz=None):
        '''
        Synchronously execute a complete movement: accel, move, decel.
        Blocks until the movement is complete and motors are stopped.
        '''
        if poll_rate_hz is None:
            poll_rate_hz = self._poll_rate_hz
        _poll_delay = 1.0 / poll_rate_hz
        self.move(distance_mm, direction)
        while self._movement_phase in (MovementPhase.ACCEL, MovementPhase.MOVE, MovementPhase.DECEL):
            current_time = time.time()
            elapsed      = current_time - self._start_time
            accumulated  = self._get_accumulated_distance()
            phase = self._movement_phase
            if phase == MovementPhase.ACCEL:
                self.handle_accel_phase(elapsed, accumulated, current_time)
            elif phase == MovementPhase.MOVE:
                self.handle_move_phase(accumulated, current_time)
            elif phase == MovementPhase.DECEL:
                self.handle_decel_phase(accumulated)
            time.sleep(_poll_delay)
        self._log.info(Fore.GREEN + 'run complete.')

    def w_run(self, distance_mm, direction=Direction.AHEAD, poll_rate_hz=None):
        '''
        Synchronously execute a complete movement: accel, move, decel.
        Blocks until the movement is complete and motors are stopped.
        '''
        if poll_rate_hz is None:
            poll_rate_hz = self._poll_rate_hz
        _poll_delay = 1.0 / poll_rate_hz
        self.move(distance_mm, direction)
        while self._movement_phase in (MovementPhase.ACCEL, MovementPhase.MOVE, MovementPhase.DECEL):
            current_time = time.time()
            elapsed      = current_time - self._start_time
            accumulated  = self._get_accumulated_distance()
            phase = self._movement_phase
            if phase == MovementPhase.ACCEL:
                self.handle_accel_phase(elapsed, accumulated, current_time)
            elif phase == MovementPhase.MOVE:
                self.handle_move_phase(accumulated, current_time)
            elif phase == MovementPhase.DECEL:
                self.handle_decel_phase(accumulated)
            time.sleep(_poll_delay)
        while not self._motor_controller.all_motors_are_stopped:
            time.sleep(0.05)
        self._log.info(Fore.GREEN + 'run complete.')

    def x_run(self, distance_mm, direction=Direction.AHEAD, poll_rate_hz=None):
        '''
        Synchronously execute a complete movement: accel, move, decel.
        Blocks until the movement is complete and all motors are stopped.
        '''
        if poll_rate_hz is None:
            poll_rate_hz = self._poll_rate_hz
        _poll_delay = 1.0 / poll_rate_hz
        self.move(distance_mm, direction)
        while self._movement_phase in (MovementPhase.ACCEL, MovementPhase.MOVE, MovementPhase.DECEL):
            current_time = time.time()
            elapsed      = current_time - self._start_time
            accumulated  = self._get_accumulated_distance()
            phase = self._movement_phase
            if phase == MovementPhase.ACCEL:
                self.handle_accel_phase(elapsed, accumulated, current_time)
            elif phase == MovementPhase.MOVE:
                self.handle_move_phase(accumulated, current_time)
            elif phase == MovementPhase.DECEL:
                self.handle_decel_phase(accumulated)
            time.sleep(_poll_delay)
        while not self._motor_controller.all_motors_are_stopped:
            time.sleep(0.05)
        self._log.info(Fore.GREEN + 'run complete.')

    def z_rotate(self, degrees, direction=Rotation.CLOCKWISE):
        '''
        Delegate rotation to RotationController if available.
        Only callable when movement controller is IDLE.

        :param degrees:   rotation angle in degrees
        :param direction: Rotation.CLOCKWISE or Rotation.COUNTER_CLOCKWISE
        '''
        if self._rotation_controller is None:
            raise ValueError('rotation_controller not available')
        if self._movement_phase != MovementPhase.IDLE:
            raise ValueError('cannot rotate - movement controller not idle (phase: {})'.format(
                self._movement_phase.name))
        self._log.info('starting rotation: {:.1f}° {}'.format(degrees, direction.label))
        prev_phase = self._movement_phase
        self._movement_phase = MovementPhase.ROTATING
        self._notify_phase_change(prev_phase, self._movement_phase)
        self._rotation_controller.rotate(degrees, direction)
        self._log.info('rotation delegated to RotationController.')

    def poll(self):
        '''
        Execute one movement control step. Call repeatedly from an external loop.
        Returns: (current_time, elapsed, accumulated_distance)
        '''
        if self._movement_phase in (MovementPhase.INACTIVE, MovementPhase.IDLE):
            return (time.time(), 0.0, 0.0)
        if self._movement_phase == MovementPhase.ROTATING:
            if self._rotation_controller and not self._rotation_controller.is_rotating:
                prev_phase = self._movement_phase
                self._movement_phase = MovementPhase.IDLE
                self._notify_phase_change(prev_phase, self._movement_phase)
                self._log.info('rotation complete, returning to idle')
            return (time.time(), 0.0, 0.0)
        current_time = time.time()
        elapsed      = current_time - self._start_time
        accumulated  = self._get_accumulated_distance()
        return (current_time, elapsed, accumulated)

    def handle_accel_phase(self, accumulated_distance, current_time):
        '''
        Handle acceleration phase.
        Sets intent to 1.0 and lets the SlewLimiter ramp up naturally.
        Transitions to MOVE when forward_velocity reaches 1.0.
        '''
        self._set_intent(1.0)
        if isclose(self._motor_controller.forward_velocity, 1.0, abs_tol=0.02):
            self._accel_distance = accumulated_distance
            prev_phase = self._movement_phase
            self._movement_phase = MovementPhase.MOVE
            self._start_time = current_time
            self._log.info('acceleration complete at {:.1f}mm, starting constant movement'.format(
                self._accel_distance))
            self._notify_phase_change(prev_phase, self._movement_phase)

    def handle_move_phase(self, accumulated_distance, current_time):
        '''
        Handle constant velocity move phase.
        Dynamically determines when to trigger DECEL based on remaining distance
        and the distance required for the SlewLimiter to ramp down from current vy.
        Uses actual measured speed from odometer rather than configured speed.
        Returns: True if transitioning to decel, False otherwise
        '''
        self._set_intent(1.0)
        _current_vy        = self._motor_controller.forward_velocity
        _slew              = self._motor_controller.slew_limiter
        _, _actual_vy, _   = self._odometer.velocity
        _stopping_distance = 0.5 * _current_vy * abs(_actual_vy) / _slew.max_vy_rate
        _remaining         = self._total_target_distance - accumulated_distance
        if _remaining <= _stopping_distance:
            self._move_distance = accumulated_distance
            prev_phase = self._movement_phase
            self._movement_phase = MovementPhase.DECEL
            self._log.info('starting deceleration at {:.1f}mm: remaining={:.1f}mm, stopping_distance={:.1f}mm, vy={:.3f}, actual_vy={:.1f}mm/s'.format(
                accumulated_distance, _remaining, _stopping_distance, _current_vy, _actual_vy))
            self._notify_phase_change(prev_phase, self._movement_phase)
            return True
        return False

    def handle_decel_phase(self, accumulated_distance):
        '''
        Handle deceleration phase.
        On each tick, computes the exact SlewLimiter rate required to stop at
        the target distance, sets it, then sets intent to zero. The SlewLimiter
        ramps vy down at precisely the rate needed to consume the remaining distance.
        Phase completes when the odometer reports the robot has stopped.
        Returns: True if deceleration complete, False otherwise
        '''
        _slew     = self._motor_controller.slew_limiter
        _, _vy, _ = self._odometer.velocity
        _remaining = self._total_target_distance - accumulated_distance
        if _remaining > 0.0 and abs(_vy) > 0.0:
            # rate required to stop in exactly remaining distance:
            # remaining = 0.5 * vy_norm * actual_speed / rate  =>  rate = 0.5 * vy_norm * actual_speed / remaining
            _vy_norm = self._motor_controller.forward_velocity
            _required_rate = (0.5 * _vy_norm * abs(_vy)) / _remaining
            _slew.max_vy_rate = _required_rate
        self._set_intent(0.0)
        if not self._odometer.is_moving():
            _slew.reset()
            self._remove_intent_vector()
            self._log.info('movement complete: accel={:.1f}mm, move={:.1f}mm, total={:.1f}mm (target={:.1f}mm, error={:.1f}mm)'.format(
                self._accel_distance, self._move_distance, accumulated_distance,
                self._total_target_distance, accumulated_distance - self._total_target_distance))
            prev_phase = self._movement_phase
            self._movement_phase = MovementPhase.IDLE
            self._notify_phase_change(prev_phase, self._movement_phase)
            return True
        return False

    def x_handle_decel_phase(self, accumulated_distance):
        '''
        Handle deceleration phase.
        Sets intent to zero; the SlewLimiter ramps vy down to zero naturally.
        Phase completes when the odometer reports the robot has stopped.
        Returns: True if deceleration complete, False otherwise
        '''
        self._set_intent(0.0)
        if not self._odometer.is_moving():
            self._remove_intent_vector()
            self._log.info('movement complete: accel={:.1f}mm, move={:.1f}mm, total={:.1f}mm (target={:.1f}mm, error={:.1f}mm)'.format(
                self._accel_distance, self._move_distance, accumulated_distance,
                self._total_target_distance, accumulated_distance - self._total_target_distance))
            prev_phase = self._movement_phase
            self._movement_phase = MovementPhase.IDLE
            self._notify_phase_change(prev_phase, self._movement_phase)
            return True
        return False

    def _remove_intent_vector(self):
        '''
        Zero and remove the intent vector from the motor controller.
        '''
        self._intent_vector = (0.0, 0.0, 0.0)
        if self._intent_vector_registered:
            self._motor_controller.remove_intent_vector(MovementController.NAME)
            self._intent_vector_registered = False
            self._log.info('intent vector removed')

    def clear_intent_vector(self):
        '''
        Zero the intent vector.
        '''
        self._intent_vector = (0.0, 0.0, 0.0)

    def cancel_movement(self):
        '''
        Immediately cancel current movement, zeros intent vector and
        removes from MotorController.
        '''
        if self._movement_phase == MovementPhase.ROTATING:
            if self._rotation_controller:
                self._rotation_controller.cancel_rotation()
        if self._movement_phase not in (MovementPhase.IDLE, MovementPhase.INACTIVE):
            self._log.warning('movement cancelled')
            self._remove_intent_vector()
            prev_phase = self._movement_phase
            self._movement_phase = MovementPhase.IDLE
            self._notify_phase_change(prev_phase, self._movement_phase)

    def enable(self):
        if not self.enabled:
            super().enable()
            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def disable(self):
        '''
        Disable the movement controller.
        Removes and zeros intent vector in MotorController.
        '''
        if self.enabled:
            self._log.debug('disabling movement controller…')
            if self._movement_phase not in (MovementPhase.IDLE, MovementPhase.INACTIVE):
                self.cancel_movement()
            self._intent_vector = (0.0, 0.0, 0.0)
            if self._intent_vector_registered:
                try:
                    self._motor_controller.remove_intent_vector(MovementController.NAME)
                    self._log.info('intent vector removed (disable)')
                except Exception as e:
                    self._log.warning('could not remove intent vector: {} (may have been removed by MotorController)'.format(e))
                self._intent_vector_registered = False
            self._movement_phase = MovementPhase.INACTIVE
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
