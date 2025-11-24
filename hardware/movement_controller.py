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

import time
from enum import Enum
import numpy as np
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level
from core.orientation import Orientation
from core.direction import Direction
from core.rotation import Rotation
from hardware.rotation_controller import RotationController

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class MovementPhase(Enum):
    '''
    Represents the phases of a movement operation.
    '''
    INACTIVE  = ( 0, 'inactive'  )
    IDLE      = ( 1, 'idle'  )
    ACCEL     = ( 2, 'accel' )
    MOVE      = ( 3, 'move' )
    DECEL     = ( 4, 'decel' )
    ROTATING  = ( 5, 'rotating' )  # delegated to RotationController

    def __init__(self, num, name):
        self._name = name

    @property
    def name(self):
        return self._name

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class MovementController(Component):
    NAME = 'movement-ctrl'
    '''
    Provides encoder-based linear movement control for the robot.
    
    Handles distance-based acceleration, constant-speed movement, and deceleration
    to achieve precise linear displacement. Supports forward, backward, and lateral
    (strafing) motion for Mecanum-wheeled robots.
    
    Can also incorporate rotational movements by delegating to RotationController.
    
    Acceleration and deceleration use time-based ramping with configured maximum
    acceleration rate. Phase transitions use distance-based detection for precise
    positioning.
    
    Intent vector is registered only during active movement and removed immediately
    upon completion.
    '''
    def __init__(self, config, motor_controller, rotation_controller=None, level=Level.INFO):
        self._log = Logger(MovementController.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        if motor_controller is None:
            raise ValueError('motor_controller cannot be None')
        self._motor_controller = motor_controller
        # get odometer for distance tracking
        self._odometer = self._motor_controller.get_odometer()
        if self._odometer is None:
            raise ValueError('odometer not available from motor controller')
        # get motor references
        self._motor_pfwd = self._motor_controller.get_motor(Orientation.PFWD)
        self._motor_sfwd = self._motor_controller.get_motor(Orientation.SFWD)
        self._motor_paft = self._motor_controller.get_motor(Orientation.PAFT)
        self._motor_saft = self._motor_controller.get_motor(Orientation.SAFT)
        # rotation controller for rotational movements
        self._rotation_controller = rotation_controller
        if self._rotation_controller:
            self._log.info('rotation controller available for combined movements')
        # configuration
        _cfg = config['kros'].get('movement_controller')
        self._movement_speed = _cfg.get('default_speed', 20.0)  # cm/sec
        # maximum acceleration rate (protects motor gearboxes)
        self._max_acceleration = _cfg.get('max_acceleration', 15.0)  # cm/s²
        # distance-based accel/decel distances
        self._accel_distance_cm = _cfg.get('accel_distance_cm', 20.0)  # cm
        self._decel_distance_cm = _cfg.get('decel_distance_cm', 20.0)  # cm
        # calculate time required to reach v_max with constant acceleration
        # v_max = a * t  →  t = v_max / a
        self._accel_time = self._movement_speed / self._max_acceleration
        self._decel_time = self._movement_speed / self._max_acceleration
        self._log.info('movement speed: {:.1f}cm/sec'.format(self._movement_speed))
        self._log.info('max acceleration: {:.1f}cm/s²'.format(self._max_acceleration))
        self._log.info('accel: {:.1f}cm in {:.2f}s'.format(self._accel_distance_cm, self._accel_time))
        self._log.info('decel: {:.1f}cm in {:.2f}s'.format(self._decel_distance_cm, self._decel_time))
        # movement state
        self._movement_phase = MovementPhase.INACTIVE
        self._movement_direction = Direction.STOPPED
        self._intent_vector = (0.0, 0.0, 0.0)
        self._priority = 0.0
        self._intent_vector_registered = False
        # phase change callbacks
        self._phase_change_callbacks = []
        # baseline pose for movement tracking
        self._baseline_x = 0.0
        self._baseline_y = 0.0
        self._start_time = None
        # movement tracking across phases
        self._total_target_distance = 0.0  # total distance requested (including accel/decel)
        self._move_target = 0.0            # target for constant-speed MOVE phase
        self._accel_distance = 0.0         # actual distance moved during accel
        self._move_distance = 0.0          # actual distance moved during move
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

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
    def accel_distance_cm(self):
        '''
        Returns the configured acceleration distance in cm.
        '''
        return self._accel_distance_cm

    @property
    def decel_distance_cm(self):
        '''
        Returns the configured deceleration distance in cm.
        '''
        return self._decel_distance_cm

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

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

    def move(self, distance_cm, direction=Direction.AHEAD):
        '''
        Move exactly the specified total distance in the given direction.
        Total movement (including accel/decel) = distance_cm.
        
        Args:
            distance_cm: Total movement distance in cm (including accel and decel phases)
            direction: Direction enum value (AHEAD, ASTERN, PORT, STARBOARD)
        '''
        if distance_cm <= (self._accel_distance_cm + self._decel_distance_cm):
            raise ValueError('distance {:.1f}cm too small for accel ({:.1f}cm) + decel ({:.1f}cm)'.format(
                distance_cm, self._accel_distance_cm, self._decel_distance_cm))
        self._total_target_distance = distance_cm
        self._move_target = distance_cm - self._accel_distance_cm - self._decel_distance_cm
        self._movement_direction = direction
        self._log.info(Fore.GREEN + 'initiating {:.1f}cm movement {} (accel={:.1f}cm, move={:.1f}cm, decel={:.1f}cm)'.format(
            distance_cm, direction.label, self._accel_distance_cm, self._move_target, self._decel_distance_cm))
        prev_phase = self._movement_phase
        self._movement_phase = MovementPhase.ACCEL
        self._start_time = time.time()
        self._accel_distance = 0.0
        self._move_distance = 0.0
        # notify phase change
        self._notify_phase_change(prev_phase, self._movement_phase)
        # initialize baseline pose from odometer
        x, y, theta = self._odometer.get_pose()
        self._baseline_x = x
        self._baseline_y = y
        # register intent vector with motor controller
        if not self._intent_vector_registered:
            self._priority = 1.0
            self._motor_controller.add_intent_vector(
                MovementController.NAME,
                lambda: self._intent_vector,
                lambda: self._priority
            )
            self._intent_vector_registered = True
            self._log.info('intent vector registered with motor controller')
        self._log.info('baseline pose: x={:.1f}cm, y={:.1f}cm'.format(self._baseline_x, self._baseline_y))

    def rotate(self, degrees, direction=Rotation.CLOCKWISE):
        '''
        Delegate rotation to RotationController if available.
        Only works when movement is completely stopped (IDLE).
        '''
        if self._rotation_controller is None:
            raise ValueError('rotation_controller not available')
        # must be IDLE - not just "not moving"
        if self._movement_phase != MovementPhase.IDLE:
            raise ValueError('cannot rotate - not in IDLE state (current: {})'.format(
                self._movement_phase.name))
        # check that movement intent vector is not registered
        if self._intent_vector_registered:
            self._log.warning('movement intent vector still registered, removing before rotation')
            self._motor_controller.remove_intent_vector(MovementController.NAME)
            self._intent_vector_registered = False
        prev_phase = self._movement_phase
        self._movement_phase = MovementPhase.ROTATING
        self._notify_phase_change(prev_phase, self._movement_phase)
        self._rotation_controller.rotate(degrees, direction)
        self._log.info('rotation delegated to RotationController.')

    def z_rotate(self, degrees, direction=Rotation.CLOCKWISE):
        '''
        Delegate rotation to RotationController if available.
        Only callable when movement controller is IDLE.
        
        Args:
            degrees: Rotation angle in degrees
            direction: Rotation.CLOCKWISE or Rotation.COUNTER_CLOCKWISE
        '''
        if self._rotation_controller is None:
            raise ValueError('rotation_controller not available')
        # can only rotate when IDLE (not moving)
        if self._movement_phase != MovementPhase.IDLE:
            raise ValueError('cannot rotate - movement controller not idle (phase: {})'.format(
                self._movement_phase.name))
        self._log.info('starting rotation: {:.1f}° {}'.format(degrees, direction.label))
        prev_phase = self._movement_phase
        self._movement_phase = MovementPhase.ROTATING
        self._notify_phase_change(prev_phase, self._movement_phase)
        self._rotation_controller.rotate(degrees, direction)
        self._log.info('rotation delegated to RotationController.')

    def x_rotate(self, degrees, direction=Rotation.CLOCKWISE):
        '''
        TODO: this rotation doesn't check status.

        Delegate rotation to RotationController if available.
        
        Args:
            degrees: Rotation angle in degrees
            direction: Rotation.CLOCKWISE or Rotation.COUNTER_CLOCKWISE
        '''
        if self._rotation_controller is None:
            raise ValueError('rotation_controller not available')
        if self.is_moving:
            raise ValueError('cannot rotate while movement is in progress')
        prev_phase = self._movement_phase
        self._movement_phase = MovementPhase.ROTATING
        self._notify_phase_change(prev_phase, self._movement_phase)
        self._rotation_controller.rotate(degrees, direction)
        self._log.info('rotation delegated to RotationController.')

    def _get_accumulated_distance(self):
        '''
        Calculate accumulated distance in cm from odometer pose changes.
        Uses Euclidean distance which works regardless of robot heading.
        '''
        x, y, theta = self._odometer.get_pose()
        dx = x - self._baseline_x
        dy = y - self._baseline_y
        # Euclidean distance from baseline
        distance = (dx**2 + dy**2)**0.5
        return distance

    def x_get_accumulated_distance(self):
        '''
        Calculate accumulated distance in cm from odometer pose changes.
        Distance is calculated relative to baseline pose in the direction of movement.
        '''
        x, y, theta = self._odometer.get_pose()
        dx = x - self._baseline_x
        dy = y - self._baseline_y
        # project displacement onto movement direction
        # for AHEAD/ASTERN, use dy component
        # for PORT/STARBOARD, use dx component
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

    def poll(self):
        '''
        Execute one movement control step. Call repeatedly from async loop.
        Returns: (current_time, elapsed, accumulated_distance)
        '''
        if self._movement_phase == MovementPhase.INACTIVE or self._movement_phase == MovementPhase.IDLE:
            return (time.time(), 0.0, 0.0)
        if self._movement_phase == MovementPhase.ROTATING:
            # check if rotation is complete
            if self._rotation_controller and not self._rotation_controller.is_rotating:
                prev_phase = self._movement_phase
                self._movement_phase = MovementPhase.IDLE
                self._notify_phase_change(prev_phase, self._movement_phase)
                self._log.info('rotation complete, returning to idle')
            return (time.time(), 0.0, 0.0)
        current_time = time.time()
        elapsed = current_time - self._start_time
        accumulated_distance = self._get_accumulated_distance()
        return (current_time, elapsed, accumulated_distance)

    def handle_accel_phase(self, elapsed, accumulated_distance, current_time):
        '''
        Handle acceleration phase: ramp up to full movement speed.
        Uses time-based ramping with configured max acceleration rate.
        Transitions based on distance for precision.
        '''
        # time-based acceleration with max rate limit
        # v = a * t, capped at v_max
        velocity = min(self._max_acceleration * elapsed, self._movement_speed)
        # convert to normalized intent vector components
        # velocity is cm/s, need to normalize to motor controller scale
        normalized_velocity = velocity / self._movement_speed  # 0.0 to 1.0
        vx = self._movement_direction.vx_direction * normalized_velocity
        vy = self._movement_direction.vy_direction * normalized_velocity
        self._intent_vector = (vx, vy, 0.0)
        # distance-based transition detection
        if accumulated_distance >= self._accel_distance_cm:
            # transition to MOVE
            prev_phase = self._movement_phase
            self._movement_phase = MovementPhase.MOVE
            self._start_time = current_time
            # capture actual accel distance
            self._accel_distance = accumulated_distance
            # reset baseline - MOVE starts here
            x, y, theta = self._odometer.get_pose()
            self._baseline_x = x
            self._baseline_y = y
            self._log.info('acceleration complete at {:.1f}cm, starting constant movement'.format(
                self._accel_distance))
            # notify phase change
            self._notify_phase_change(prev_phase, self._movement_phase)

    def handle_move_phase(self, accumulated_distance, current_time):
        '''
        Handle move phase: maintain constant movement speed for move_target distance.
        Returns: True if movement target reached, False otherwise
        '''
        # maintain constant movement speed
        vx = self._movement_direction.vx_direction
        vy = self._movement_direction.vy_direction
        self._intent_vector = (vx, vy, 0.0)
        # check if we've completed the target distance
        if accumulated_distance >= self._move_target:
            prev_phase = self._movement_phase
            self._movement_phase = MovementPhase.DECEL
            self._start_time = time.time()
            # capture actual move distance
            self._move_distance = accumulated_distance
            # reset baseline for decel phase
            x, y, theta = self._odometer.get_pose()
            self._baseline_x = x
            self._baseline_y = y
            total_so_far = self._accel_distance + self._move_distance
            self._log.info('starting deceleration at {:.1f}cm (move target: {:.1f}cm, total target: {:.1f}cm)'.format(
                total_so_far, self._move_target, self._total_target_distance))
            # notify phase change
            self._notify_phase_change(prev_phase, self._movement_phase)
            return True
        return False

    def handle_decel_phase(self, elapsed, accumulated_distance):
        '''
        Handle deceleration phase: ramp down to stop.
        Uses time-based ramping with configured max acceleration rate.
        Completes based on distance or when velocity reaches zero.
        Returns: True if deceleration complete, False otherwise
        '''
        # time-based deceleration with max rate limit
        # v = v_max - a * t, floored at 0
        velocity = max(self._movement_speed - self._max_acceleration * elapsed, 0.0)
        # convert to normalized intent vector components
        normalized_velocity = velocity / self._movement_speed
        vx = self._movement_direction.vx_direction * normalized_velocity
        vy = self._movement_direction.vy_direction * normalized_velocity
        self._intent_vector = (vx, vy, 0.0)
        # complete when velocity reaches zero OR distance target reached
        if velocity == 0.0 or accumulated_distance >= self._decel_distance_cm:
            # stop - zero intent vector
            self._intent_vector = (0.0, 0.0, 0.0)
            # remove from MotorController immediately
            if self._intent_vector_registered:
                self._motor_controller.remove_intent_vector(MovementController.NAME)
                self._intent_vector_registered = False
                self._log.info('intent vector removed (movement complete)')
            # calculate total distance: accel + move + decel
            total_distance = self._accel_distance + self._move_distance + accumulated_distance
            self._log.info('movement complete: accel={:.1f}cm, move={:.1f}cm, decel={:.1f}cm, total={:.1f}cm (target={:.1f}cm, error={:.1f}cm)'.format(
                self._accel_distance, self._move_distance, accumulated_distance,
                total_distance, self._total_target_distance, total_distance - self._total_target_distance))
            # transition to idle
            prev_phase = self._movement_phase
            self._movement_phase = MovementPhase.IDLE
            # notify phase change
            self._notify_phase_change(prev_phase, self._movement_phase)
            return True
        return False

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
        if self._movement_phase != MovementPhase.IDLE and self._movement_phase != MovementPhase.INACTIVE:
            self._log.warning('movement cancelled')
            # zero intent vector first
            self._intent_vector = (0.0, 0.0, 0.0)
            # remove from motor controller
            if self._intent_vector_registered:
                self._motor_controller.remove_intent_vector(MovementController.NAME)
                self._intent_vector_registered = False
                self._log.info('intent vector removed (cancelled)')
            prev_phase = self._movement_phase
            self._movement_phase = MovementPhase.IDLE
            # notify phase change
            self._notify_phase_change(prev_phase, self._movement_phase)

    def enable(self):
        if self.enabled:
            self._log.warning('already enabled.')
            return
        Component.enable(self)
        self._log.info('enabled.')

    def disable(self):
        '''
        Disable the movement controller.
        
        Removes and zeros intent vector in MotorController.
        '''
        if not self.enabled:
            self._log.warning('already disabled.')
            return
        self._log.info('disabling movement controller…')
        # cancel any active movement (zeros vector and removes registration)
        if self._movement_phase != MovementPhase.IDLE and self._movement_phase != MovementPhase.INACTIVE:
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
        Component.disable(self)
        self._log.info('disabled.')

    def close(self):
        if not self.closed:
            self.disable()
            Component.close(self)
            self._log.info('closed.')

#EOF
