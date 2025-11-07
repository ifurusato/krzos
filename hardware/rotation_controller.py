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

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class RotationPhase(Enum):
    '''
    Represents the phases of a rotation operation.
    '''
    INACTIVE  = ( 0, 'inactive'  )
    IDLE      = ( 1, 'idle'  )
    ACCEL     = ( 2, 'accel' )
    SCAN      = ( 3, 'scan'  )
    DECEL     = ( 4, 'decel' )

    def __init__(self, num, name):
        self._name = name

    @property
    def name(self):
        return self._name

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class RotationController(Component):
    NAME = 'rotation-controller'
    '''
    Provides encoder-based rotation control for the robot.
    
    This is an EXACT extraction of the rotation logic from Scan, moved into
    a reusable component. All rotation parameters, calculations, and phase
    transitions are identical to the working Scan implementation.
    
    CRITICAL SAFETY: Intent vector is registered only during active rotation
    and removed immediately upon completion to prevent dangerous motor surges.
    '''
    def __init__(self, config, motor_controller, level=Level.INFO):
        self._log = Logger(RotationController.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        
        if motor_controller is None:
            raise ValueError('motor_controller cannot be None')
        self._motor_controller = motor_controller
        
        # get motor references (EXACT COPY from Scan)
        self._motor_pfwd = self._motor_controller.get_motor(Orientation.PFWD)
        self._motor_sfwd = self._motor_controller.get_motor(Orientation.SFWD)
        self._motor_paft = self._motor_controller.get_motor(Orientation.PAFT)
        self._motor_saft = self._motor_controller.get_motor(Orientation.SAFT)
        
        # configuration (EXACT COPY from Scan)
        _cfg = config['kros'].get('rotation_controller')
        self._rotation_speed = _cfg.get('default_rotation_speed_deg_per_sec', 15.0)  # degrees/sec
        self._rotation_speed_rad = np.deg2rad(self._rotation_speed)
        self._accel_time = _cfg.get('accel_time', 2.0)  # seconds to reach full speed
        # empirical value from 10x 360° rotations on carpet with 11-roller mecanum wheels
        self._steps_per_degree = _cfg.get('steps_per_degree', 6.256)
        
        self._log.info('rotation speed: {:.1f}°/sec'.format(self._rotation_speed))
        self._log.info('acceleration time: {:.1f}s'.format(self._accel_time))
        self._log.info('steps_per_degree: {:.3f} (empirical from 10x 360° rotations)'.format(
            self._steps_per_degree))
        
        # rotation state (EXACT COPY from Scan)
        self._rotation_phase = RotationPhase.INACTIVE
        self._intent_vector = (0.0, 0.0, 0.0)
        self._priority = 0.0
        self._intent_vector_registered = False
        
        # encoder baselines for rotation tracking (EXACT COPY from Scan)
        self._baseline_pfwd = 0
        self._baseline_sfwd = 0
        self._baseline_paft = 0
        self._baseline_saft = 0
        self._start_time = None
        
        # rotation tracking for SCAN phase (EXACT COPY from Scan)
        self._scan_start_rotation = 0.0
        
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
    def intent_vector(self):
        '''
        Returns the current intent vector (for debugging/monitoring).
        '''
        return self._intent_vector

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
    def initiate_rotation(self):
        '''
        Begin rotation sequence (EXACT COPY of Scan._initiate_scan rotation setup).
        '''
        self._log.info(Fore.GREEN + 'initiating rotation…')
        self._rotation_phase = RotationPhase.ACCEL
        self._start_time = time.time()
        self._scan_start_rotation = 0.0
        
        # initialize encoder baselines (EXACT COPY from Scan)
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
        
        self._log.info(Fore.GREEN + 'rotation initiated, using encoder-based rotation tracking')
        self._log.info('encoder baselines: PFWD={}, SFWD={}, PAFT={}, SAFT={}'.format(
            self._baseline_pfwd, self._baseline_sfwd, self._baseline_paft, self._baseline_saft))

    def _get_accumulated_rotation(self):
        '''
        Calculate accumulated rotation in degrees from encoder deltas.
        Averages all four mecanum wheels for robust tracking.
        (EXACT COPY from Scan)
        '''
        current_pfwd = self._motor_pfwd.steps
        current_sfwd = self._motor_sfwd.steps
        current_paft = self._motor_paft.steps
        current_saft = self._motor_saft.steps
        
        # for CW rotation: port wheels go forward (+), starboard wheels go backward (-)
        # average all four to handle slip and variance
        rotation_steps = ((current_pfwd - self._baseline_pfwd) + 
                          (current_paft - self._baseline_paft) - 
                          (current_sfwd - self._baseline_sfwd) - 
                          (current_saft - self._baseline_saft)) / 4.0
        
        degrees = rotation_steps / self._steps_per_degree
        return degrees

    def poll(self):
        '''
        Execute one rotation control step. Call repeatedly from async loop.
        Returns: (current_time, elapsed, accumulated_rotation)
        (EXACT COPY of Scan._poll rotation logic)
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
        (EXACT COPY from Scan._handle_accel_phase)
        '''
        # linear acceleration to full speed
        progress = min(elapsed / self._accel_time, 1.0)
        omega = self._rotation_speed_rad * progress
        self._intent_vector = (0.0, 0.0, omega)
        
        if progress >= 1.0:
            self._rotation_phase = RotationPhase.SCAN
            self._start_time = current_time
            
            # reset encoder baselines - SCAN starts here
            self._baseline_pfwd = self._motor_pfwd.steps
            self._baseline_sfwd = self._motor_sfwd.steps
            self._baseline_paft = self._motor_paft.steps
            self._baseline_saft = self._motor_saft.steps
            
            accel_rotation = accumulated_rotation
            self._log.info('acceleration complete at {:.1f}°, resetting baseline for 360° scan'.format(
                accel_rotation))

    def handle_scan_phase(self, accumulated_rotation, current_time):
        '''
        Handle scan phase: maintain constant rotation speed.
        (EXACT COPY from Scan._handle_scan_phase, minus sensor capture)
        
        Returns: True if 360° complete, False otherwise
        '''
        # maintain constant rotation speed
        omega = self._rotation_speed_rad
        self._intent_vector = (0.0, 0.0, omega)
        
        # check if we've completed 360°
        if accumulated_rotation >= 360.0:
            self._rotation_phase = RotationPhase.DECEL
            self._start_time = time.time()
            self._log.info('360° complete, decelerating…')
            return True
        
        return False

    def handle_decel_phase(self, elapsed, accumulated_rotation):
        '''
        Handle deceleration phase: ramp down to stop.
        (EXACT COPY from Scan._handle_decel_phase, minus data processing)
        
        Returns: True if deceleration complete, False otherwise
        
        CRITICAL FIX: Removes intent vector from MotorController immediately
        upon completion to prevent dangerous motor surges during shutdown.
        '''
        # linear deceleration to stop
        progress = elapsed / self._accel_time
        if progress < 1.0:
            omega = self._rotation_speed_rad * (1.0 - progress)
            self._intent_vector = (0.0, 0.0, omega)
            return False
        else:
            # stop - zero intent vector
            self._intent_vector = (0.0, 0.0, 0.0)
            
            # CRITICAL SAFETY FIX: Remove from MotorController immediately
            # This prevents the intent vector lambda from being called with
            # stale non-zero omega values during shutdown or cancellation
            if self._intent_vector_registered:
                self._motor_controller.remove_intent_vector(RotationController.NAME)
                self._intent_vector_registered = False
                self._log.info('intent vector removed from motor controller (rotation complete)')
            
            self._log.info('rotation complete, total rotation: {:.1f}°'.format(
                accumulated_rotation))
            
            # update cumulative heading
            self._current_heading_offset = (self._current_heading_offset + accumulated_rotation) % 360.0
            
            # return to idle
            self._rotation_phase = RotationPhase.IDLE
            return True

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
                self._log.info('intent vector removed from motor controller (cancelled)')
            
            self._rotation_phase = RotationPhase.IDLE

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
                self._log.info('intent vector removed from motor controller (disable)')
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
