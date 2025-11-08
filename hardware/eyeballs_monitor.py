#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-08
# modified: 2025-11-08
#

from math import isclose
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from hardware.eyeballs import Eyeballs
from hardware.eyeball import Eyeball

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class EyeballsMonitor(Component):
    NAME = 'eyeballs-monitor'
    '''
    Monitors the robot's motion direction and displays corresponding eye expressions.
    
    Uses the motor controller's blended intent vector (vx, vy, omega) to determine
    direction and updates the eyeballs display only when direction changes.
    
    Direction mapping:

        - stopped:            BLANK
        - forward:            LOOK_DOWN
        - aft:                LOOK_UP
        - port:               LOOK_PORT
        - starboard:          LOOK_STBD
        - forward+port:       LOOK_PORT_FWD
        - forward+starboard:  LOOK_STBD_FWD
        - aft+port:           LOOK_PORT_AFT
        - aft+starboard:      LOOK_STBD_AFT
    
    :param motor_controller:  the MotorController instance to monitor
    :param level:             the logging Level
    '''
    def __init__(self, motor_controller, level=Level.INFO):
        self._log = Logger(EyeballsMonitor.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._motor_controller = motor_controller
        # get eyeballs from registry
        _component_registry = Component.get_registry()
        self._eyeballs = _component_registry.get(Eyeballs.NAME)
        if self._eyeballs is None:
            raise Exception('Eyeballs not available in registry.')
        # thresholds for detecting motion
        self._motion_threshold  = 0.05  # minimum value to consider as intentional motion
        self._lateral_threshold = 0.1   # minimum lateral (vx) for side indication
        self._previous_eyeball  = None  # track previous state to avoid redundant updates
        self._manual_eyeball    = None  # holds manual Eyeball enum when set, None = auto
        self._log.info('ready.')
    
    @property
    def name(self):
        return EyeballsMonitor.NAME

    def set_eyeballs(self, eyeball):
        '''
        Manually set the eyeball expression, overriding motor controller monitoring.
        
        Args:
            eyeball: Eyeball enum value to display
        '''
        if not isinstance(eyeball, Eyeball):
            raise TypeError('expected Eyeball enum, not {}'.format(type(eyeball)))
        self._manual_eyeball = eyeball
        self._display_eyeball(eyeball)
        self._log.info('manual override set: {}'.format(eyeball.name))

    def clear_eyeballs(self):
        '''
        Clear the manual eyeball override and resume motor controller monitoring.
        '''
        self._manual_eyeball   = None
        self._previous_eyeball = None  # force update on next motor tick
        self._log.info('manual override cleared, resuming motor monitoring.')
    
    def update(self):
        '''
        Check the motor controller's current intent vector and update eyeballs
        display if direction has changed.
        
        Called by MotorController during each motor tick.
        '''
        if not self.enabled or self._eyeballs is None or self._manual_eyeball is not None:
            return
        # get blended intent vector from motor controller
        vx, vy, omega = self._motor_controller._blended_intent_vector
        # determine current eyeball expression
        eyeball = self._calculate_eyeball(vx, vy, omega)
        # only update if expression changed
        if eyeball != self._previous_eyeball:
            self._display_eyeball(eyeball)
            self._previous_eyeball = eyeball
    
    def _calculate_eyeball(self, vx, vy, omega):
        '''
        Determine the appropriate eyeball expression based on intent vector components.
        
        Args:
            vx: lateral velocity (+ = starboard, - = port)
            vy: longitudinal velocity (+ = forward, - = aft)
            omega: rotational velocity (ignored for display purposes)
        
        Returns:
            Eyeball: the appropriate eyeball enum value
        '''
        # check if stopped (all components near zero)
        if (isclose(vx, 0.0, abs_tol=self._motion_threshold) and 
            isclose(vy, 0.0, abs_tol=self._motion_threshold)):
            return Eyeball.BLANK
        if self._motor_controller.is_stopped:
            return Eyeball.SLEEPY
        # determine primary forward/aft direction
        is_forward = vy > self._motion_threshold
        is_aft     = vy < -self._motion_threshold
        # determine lateral direction (only if significant)
        is_stbd    = vx > self._lateral_threshold
        is_port    = vx < -self._lateral_threshold
        # check direction combinations
        if is_forward and is_port:
            return Eyeball.LOOK_PORT_FWD
        elif is_forward and is_stbd:
            return Eyeball.LOOK_STBD_FWD
        elif is_aft and is_port:
            return Eyeball.LOOK_PORT_AFT
        elif is_aft and is_stbd:
            return Eyeball.LOOK_STBD_AFT
        elif is_forward:
            return Eyeball.LOOK_DOWN
        elif is_aft:
            return Eyeball.LOOK_UP
        elif is_port:
            return Eyeball.LOOK_PORT
        elif is_stbd:
            return Eyeball.LOOK_STBD
        else:
            # fallback for edge cases
            return Eyeball.SAD

    def _display_eyeball(self, eyeball):
        '''
        Update eyeballs display with the specified expression.
        
        Args:
            eyeball: Eyeball enum value
        '''
        self._eyeballs.set_matrix(eyeball.array, self._eyeballs._port_rgbmatrix, eyeball.color)
        self._eyeballs.set_matrix(eyeball.array, self._eyeballs._stbd_rgbmatrix, eyeball.color)
        self._eyeballs._show()
        
        self._log.debug('displaying eyeball: {}'.format(eyeball.name))
    
    def enable(self):
        if not self.enabled:
            Component.enable(self)
            if not self._eyeballs.enabled:
                self._eyeballs.enable()
            self._log.info('enabled.')
        else:
            self._log.debug('already enabled.')
    
    def disable(self):
        if self.enabled:
            Component.disable(self)
            self._eyeballs.blank()
            self._log.info('disabled.')
        else:
            self._log.debug('already disabled.')
    
    def close(self):
        if not self.closed:
            self.disable()
            Component.close(self)
            self._log.info('closed.')

#EOF
