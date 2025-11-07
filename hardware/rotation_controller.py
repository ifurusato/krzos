#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-07
# modified: 2025-11-07

import time
from enum import Enum
import numpy as np
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level
from core.orientation import Orientation
from core.rotation import Rotation

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class RotationPhase(Enum):
    '''
    Represents the phases of a rotation operation.
    '''
    INACTIVE  = ( 0, 'inactive'  )
    IDLE      = ( 1, 'idle'  )
    ACCEL     = ( 2, 'accel' )
    ROTATE    = ( 3, 'rotate' )
    DECEL     = ( 4, 'decel' )

    def __init__(self, num, name):
        self._name = name

    @property
    def name(self):
        return self._name

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class RotationController(Component):
    NAME = 'rotation-ctrl'
    '''
    Provides encoder-based rotation control for the robot.
    
    Handles angle-based acceleration, constant-speed rotation, and deceleration
    to achieve precise total angular rotation. The rotate() method specifies the
    TOTAL rotation including accel and decel phases.
    
    Acceleration and deceleration use time-based ramping with configured maximum
    acceleration rate to protect motor gearboxes. Phase transitions use angle-based
    detection for precise positioning.
    
    Behaviors can register for phase change callbacks to coordinate with rotation phases.
    
    CRITICAL SAFETY: Intent vector is registered only during active rotation
    and removed immediately upon completion to prevent dangerous motor surges.
    '''
    def __init__(self, config, motor_controller, level=Level.INFO):
        self._log = Logger(RotationController.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        
        if motor_controller is None:
            raise ValueError('motor_controller cannot be None')
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
        self._log.info('accel: {:.1f}° in {:.2f}s'.format(self._accel_degrees, self._accel_time))
        self._log.info('decel: {:.1f}° in {:.2f}s'.format(self._decel_degrees, self._decel_time))
        self._log.info('steps_per_degree: {:.3f}'.format(self._steps_per_degree))
        
        # rotation state
        self._rotation_phase = RotationPhase.INACTIVE
        self._rotation_direction = Rotation.CLOCKWISE
        self._intent_vector = (0.0, 0.0, 0.0)
        self._priority = 0.0
        self._intent_vector_registered = False
        
        # phase change callbacks
        self._phase_change_callbacks = []
        
        # encoder baselines for rotation tracking
        self._baseline_pfwd = 0
        self._baseline_sfwd = 0
        self._baseline_paft = 0
        self._baseline_saft = 0
        self._start_time = None
        
        # rotation tracking across phases
        self._total_target_rotation = 0.0  # total rotation requested (including accel/decel)
        self._rotate_target = 0.0          # target for constant-speed ROTATE phase
        self._accel_rotation = 0.0         # actual degrees rotated during accel
        self._rotate_rotation = 0.0        # actual degrees rotated during rotate
        
        # cumulative heading state
        self._current_heading_offset = 0.0
        self._heading_markers = []
        
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
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

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def rotate(self, degrees, direction=Rotation.CLOCKWISE):
        '''
        Rotate exactly the specified total degrees in the given direction.
        Total rotation (including accel/decel) = degrees.
        
        Args:
            degrees: Total rotation angle (including accel and decel phases)
            direction: Rotation.CLOCKWISE or Rotation.COUNTER_CLOCKWISE
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
        
        self._log.info('encoder baselines: PFWD={}, SFWD={}, PAFT={}, SAFT={}'.format(
            self._baseline_pfwd, self._baseline_sfwd, self._baseline_paft, self._baseline_saft))

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
        
        # for CW rotation: port wheels go forward (+), starboard wheels go backward (-)
        # for CCW rotation: port wheels go backward (-), starboard wheels go forward (+)
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
        Returns: (current_time, elapsed, accumulated_rotation)
        '''
        if self._rotation_phase == RotationPhase.INACTIVE or self._rotation_phase == RotationPhase.IDLE:
            return (time.time(), 0.0, 0.0)
        
        current_time = time.time()
        elapsed = current_time - self._start_time
        accumulated_rotation = self._get_accumulated_rotation()
        
        return (current_time, elapsed, accumulated_rotation)

    def handle_accel_phase(self, elapsed, accumulated_rotation, current_time):
        '''
        Handle acceleration phase: ramp up to full rotation speed.
        Uses time-based ramping with configured max acceleration rate.
        Transitions based on angle for precision.
        '''
        # time-based acceleration with max rate limit
        # omega = a * t, capped at v_max
        omega = min(self._max_acceleration_rad * elapsed, self._rotation_speed_rad)
        
        # apply direction
        if self._rotation_direction == Rotation.COUNTER_CLOCKWISE:
            omega = -omega
        
        self._intent_vector = (0.0, 0.0, omega)
        
        # angle-based transition detection
        if accumulated_rotation >= self._accel_degrees:
            # transition to ROTATE
            old_phase = self._rotation_phase
            self._rotation_phase = RotationPhase.ROTATE
            self._start_time = current_time
            
            # capture actual accel rotation
            self._accel_rotation = accumulated_rotation
            
            # reset encoder baselines - ROTATE starts here
            self._baseline_pfwd = self._motor_pfwd.steps
            self._baseline_sfwd = self._motor_sfwd.steps
            self._baseline_paft = self._motor_paft.steps
            self._baseline_saft = self._motor_saft.steps
            
            self._log.info('acceleration complete at {:.1f}°, starting constant rotation'.format(
                self._accel_rotation))
            
            # notify phase change
            self._notify_phase_change(old_phase, self._rotation_phase)

    def handle_rotate_phase(self, accumulated_rotation, current_time):
        '''
        Handle rotate phase: maintain constant rotation speed for rotate_target degrees.
        Returns: True if rotation target reached, False otherwise
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
            
            # capture actual rotate rotation
            self._rotate_rotation = accumulated_rotation
            
            # reset baseline for decel phase
            self._baseline_pfwd = self._motor_pfwd.steps
            self._baseline_sfwd = self._motor_sfwd.steps
            self._baseline_paft = self._motor_paft.steps
            self._baseline_saft = self._motor_saft.steps
            
            total_so_far = self._accel_rotation + self._rotate_rotation
            self._log.info('starting deceleration at {:.1f}° (rotate target: {:.1f}°, total target: {:.1f}°)'.format(
                total_so_far, self._rotate_target, self._total_target_rotation))
            
            # notify phase change
            self._notify_phase_change(old_phase, self._rotation_phase)
            return True
        
        return False

    def handle_decel_phase(self, elapsed, accumulated_rotation):
        '''
        Handle deceleration phase: ramp down to stop.
        Uses time-based ramping with configured max acceleration rate.
        Completes based on angle or when omega reaches zero.
        Returns: True if deceleration complete, False otherwise
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
            
            # CRITICAL SAFETY: Remove from MotorController immediately
            if self._intent_vector_registered:
                self._motor_controller.remove_intent_vector(RotationController.NAME)
                self._intent_vector_registered = False
                self._log.info('intent vector removed (rotation complete)')
            
            # calculate total rotation: accel + rotate + decel
            total_rotation = self._accel_rotation + self._rotate_rotation + accumulated_rotation
            
            self._log.info('rotation complete: accel={:.1f}°, rotate={:.1f}°, decel={:.1f}°, total={:.1f}° (target={:.1f}°, error={:.1f}°)'.format(
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
        
        CRITICAL SAFETY: Zeros intent vector and removes from MotorController.
        '''
        if self._rotation_phase != RotationPhase.IDLE and self._rotation_phase != RotationPhase.INACTIVE:
            self._log.warning('rotation cancelled')
            
            # Zero intent vector first
            self._intent_vector = (0.0, 0.0, 0.0)
            
            # Remove from motor controller
            if self._intent_vector_registered:
                self._motor_controller.remove_intent_vector(RotationController.NAME)
                self._intent_vector_registered = False
                self._log.info('intent vector removed (cancelled)')
            
            old_phase = self._rotation_phase
            self._rotation_phase = RotationPhase.IDLE
            
            # notify phase change
            self._notify_phase_change(old_phase, self._rotation_phase)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        if self.enabled:
            self._log.warning('already enabled.')
            return
        Component.enable(self)
        self._log.info('enabled.')

    def disable(self):
        '''
        Disable the rotation controller.
        
        CRITICAL SAFETY: Ensures intent vector is zeroed and removed from
        MotorController to prevent dangerous motor surges during shutdown.
        '''
        if not self.enabled:
            self._log.warning('already disabled.')
            return
        
        self._log.info('disabling rotation controller…')
        
        # Cancel any active rotation (zeros vector and removes registration)
        if self._rotation_phase != RotationPhase.IDLE and self._rotation_phase != RotationPhase.INACTIVE:
            self.cancel_rotation()
        
        # Ensure intent vector is zeroed (defensive)
        self._intent_vector = (0.0, 0.0, 0.0)
        
        # Ensure unregistered from motor controller (defensive)
        if self._intent_vector_registered:
            try:
                self._motor_controller.remove_intent_vector(RotationController.NAME)
                self._log.info('intent vector removed (disable)')
            except Exception as e:
                self._log.warning('could not remove intent vector: {} (may have been removed by MotorController)'.format(e))
            self._intent_vector_registered = False
        
        self._rotation_phase = RotationPhase.INACTIVE
        
        Component.disable(self)
        self._log.info('disabled.')

    def close(self):
        if not self.closed:
            self.disable()
            Component.close(self)
            self._log.info('closed.')

#EOF
