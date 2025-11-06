#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-01-05
# modified: 2025-01-06

import sys
import time
import itertools
from enum import Enum
import numpy as np
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.event import Event
from behave.async_behaviour import AsyncBehaviour
from hardware.motor_controller import MotorController
from hardware.usfs import Usfs
from core.orientation import Orientation
from hardware.scout_sensor import ScoutSensor

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈[...]
class ScanPhase(Enum):
    INACTIVE  = ( 0, 'inactive'  )
    IDLE      = ( 1, 'idle'  )
    ACCEL     = ( 2, 'accel' )
    SCAN      = ( 3, 'scan'  )
    DECEL     = ( 4, 'decel' )

    def __init__(self, num, name):
        self._name  = name

    @property
    def name(self):
        return self._name

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈[...]
class Scan(AsyncBehaviour):
    NAME = 'scan'
    '''
    Performs a 360° rotational scan using VL53L5CX to build a complete
    environmental map, then orients robot toward most open direction.
    
    Uses encoder-based rotation tracking due to IMU gyroscope interference
    from motor operation.
    '''
    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Scan.NAME, level)
        _component_registry = Component.get_registry()
        _motor_controller = _component_registry.get(MotorController.NAME)
        if _motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        # IMU for absolute heading (used after scan completes)
        self._imu = _component_registry.get(Usfs.NAME)
        if self._imu is None:
            raise MissingComponentError('IMU not available for Scan.')
        if not self._imu.enabled:
            self._imu.enable()
        # configuration
        _cfg = config['kros'].get('behaviour').get('scan')
        self._counter = itertools.count()
        self._verbose = _cfg.get('verbose', False)
        # rotation parameters
        self._rotation_speed = _cfg.get('rotation_speed', 15.0)  # degrees/sec
        self._rotation_speed_rad = np.deg2rad(self._rotation_speed)
        self._accel_time = _cfg.get('accel_time', 2.0)  # seconds to reach full speed
        self._angular_resolution = _cfg.get('angular_resolution', 5.0)  # degrees
        # eyeballs
        self._eyeballs = None
        self._use_eyeballs = True
        if self._use_eyeballs:
            from hardware.eyeballs import Eyeballs
            self._eyeballs = _component_registry.get(Eyeballs.NAME)
        # scan state
        self._scan_phase = ScanPhase.INACTIVE
        # encoder baselines for rotation tracking
        self._baseline_pfwd = 0
        self._baseline_sfwd = 0
        self._baseline_paft = 0
        self._baseline_saft = 0
        self._start_time = None
        AsyncBehaviour.__init__(self, self._log, config, message_bus, message_factory, _motor_controller, level=level)
        self.add_event(Event.STUCK)
        # AsyncBehaviour provides: self._steps_per_degree (from config or theoretical)
        self._steps_per_degree = 6.256
        self._log.info('using steps_per_degree: {:.3f}'.format(self._steps_per_degree))
        self._log.info('ready.')

    @property
    def name(self):
        return Scan.NAME

    @property
    def is_ballistic(self):
        '''Ballistic when actively scanning (not INACTIVE or IDLE)'''
        return self._scan_phase not in (ScanPhase.INACTIVE, ScanPhase.IDLE)

    @property
    def priority(self):
        return 1.0 if self.is_ballistic else 0.0

    def callback(self):
        self._log.info('scan behaviour callback.')
        raise Exception('UNSUPPORTED callback')

    def execute(self, message):
        '''
        Called when STUCK message received via message bus.
        '''
        if message.event is Event.STUCK:
            self._log.info('STUCK event received, initiating scan…')
            if self._scan_phase == ScanPhase.INACTIVE or self._scan_phase == ScanPhase.IDLE:
                self._initiate_scan()
            else:
                self._log.warning('scan already in progress ({}), ignoring STUCK event'.format(
                    self._scan_phase.name))
        else:
            self._log.warning('unexpected message event: {}'.format(message.event))

    def _initiate_scan(self):
        '''
        Begin scan sequence using encoder-based rotation tracking.
        '''
        self._log.info(Fore.GREEN + 'initiating scan…')
        self._scan_phase = ScanPhase.ACCEL
        self._start_time = time.time()
        
        # Initialize encoder baselines
        self._baseline_pfwd = self._motor_pfwd.steps
        self._baseline_sfwd = self._motor_sfwd.steps
        self._baseline_paft = self._motor_paft.steps
        self._baseline_saft = self._motor_saft.steps
        
        self._log.info(Fore.GREEN + 'scan initiated, using encoder-based rotation tracking')
        self._log.info('encoder baselines: PFWD={}, SFWD={}, PAFT={}, SAFT={}'.format(
            self._baseline_pfwd, self._baseline_sfwd, self._baseline_paft, self._baseline_saft))

    def _get_accumulated_rotation(self):
        '''
        Calculate accumulated rotation in degrees from encoder deltas.
        Averages all four mecanum wheels for robust tracking.
        '''
        current_pfwd = self._motor_pfwd.steps
        current_sfwd = self._motor_sfwd.steps
        current_paft = self._motor_paft.steps
        current_saft = self._motor_saft.steps
        
        # For CW rotation: port wheels go forward (+), starboard wheels go backward (-)
        # Average all four to handle slip and variance
        rotation_steps = ((current_pfwd - self._baseline_pfwd) + 
                          (current_paft - self._baseline_paft) - 
                          (current_sfwd - self._baseline_sfwd) - 
                          (current_saft - self._baseline_saft)) / 4.0
        
        degrees = rotation_steps / self._steps_per_degree
        return degrees

    def start_loop_action(self):
        '''
        Called when async loop starts.
        '''
        self._log.info('scan loop started')

    def stop_loop_action(self):
        '''
        Called when async loop stops.
        '''
        self._log.info('scan loop stopped')

    async def _poll(self):
        '''
        Main scan control loop - uses encoders for rotation tracking.
        '''
        if self._scan_phase == ScanPhase.INACTIVE or self._scan_phase == ScanPhase.IDLE:
            return
        
        try:
            current_time = time.time()
            elapsed = current_time - self._start_time
            
            # Get accumulated rotation from encoders
            accumulated_rotation = self._get_accumulated_rotation()
            
            match self._scan_phase:
                case ScanPhase.ACCEL:
                    if self._eyeballs:
                        self._eyeballs.look_up()
                    # linear acceleration to full speed
                    progress = min(elapsed / self._accel_time, 1.0)
                    omega = self._rotation_speed_rad * progress
                    self._intent_vector = (0.0, 0.0, omega)
                    
                    if progress >= 1.0:
                        self._scan_phase = ScanPhase.SCAN
                        self._start_time = current_time
                        
                        # Reset encoder baselines - SCAN starts here
                        self._baseline_pfwd = self._motor_pfwd.steps
                        self._baseline_sfwd = self._motor_sfwd.steps
                        self._baseline_paft = self._motor_paft.steps
                        self._baseline_saft = self._motor_saft.steps
                        
                        accel_rotation = self._get_accumulated_rotation()  # for logging only
                        self._log.info('acceleration complete at {:.1f}°, resetting baseline for 360° scan'.format(
                            accel_rotation))

                case ScanPhase.SCAN:
                    if self._eyeballs:
                        self._eyeballs.look_stbd()
                    # maintain constant rotation speed
                    omega = self._rotation_speed_rad
                    self._intent_vector = (0.0, 0.0, omega)
                    
                    # check if we've completed 360° via encoder integration
                    if accumulated_rotation >= 360.0:
                        self._scan_phase = ScanPhase.DECEL
                        self._start_time = current_time
                        self._log.info('360° rotation complete ({:.1f}° by encoders), decelerating…'.format(
                            accumulated_rotation))

                case ScanPhase.DECEL:
                    if self._eyeballs:
                        self._eyeballs.look_down()
                    # linear deceleration to stop
                    progress = elapsed / self._accel_time
                    if progress < 1.0:
                        omega = self._rotation_speed_rad * (1.0 - progress)
                        self._intent_vector = (0.0, 0.0, omega)
                    else:
                        # Stop
                        self.clear_intent_vector()
                        if self._eyeballs:
                            self._eyeballs.normal()
                        self._scan_phase = ScanPhase.IDLE
                        self._log.info('scan complete, total rotation: {:.1f}°'.format(
                            accumulated_rotation))

                case _:
                    self._log.warning('unexpected phase: {}'.format(self._scan_phase.name))
                    if self._eyeballs:
                        self._eyeballs.confused()
            
            if self._verbose and next(self._counter) % 5 == 0:
                self._log.info('phase: {}; accumulated: {:.1f}°; omega: {:.3f}'.format(
                    self._scan_phase.name, accumulated_rotation, self._intent_vector[2]))
        
        except Exception as e:
            self._log.error("{} thrown while polling: {}".format(type(e), e))
            self.clear_intent_vector()
            self._scan_phase = ScanPhase.IDLE
            self.disable()

    def ping(self):
        self._log.warning('ping.')

    def enable(self):
        if self.enabled:
            self._log.warning('already enabled.')
            return
        self._log.info('enabling scan...')
        AsyncBehaviour.enable(self)
        if self._eyeballs:
            self._eyeballs.enable()
        self._log.info('scan enabled.')

    def disable(self):
        if not self.enabled:
            self._log.warning('already disabled.')
            return
        self._log.info('disabling scan…')
        self._scan_phase = ScanPhase.INACTIVE
        self.clear_intent_vector()
        AsyncBehaviour.disable(self)
        if self._eyeballs:
            self._eyeballs.disable()
        self._log.info('scan disabled.')

#EOF
