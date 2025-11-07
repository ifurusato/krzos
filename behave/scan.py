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

import sys
import time
import itertools
import traceback
from enum import Enum
import numpy as np
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.event import Event
from core.queue_publisher import QueuePublisher
from behave.async_behaviour import AsyncBehaviour
from hardware.motor_controller import MotorController
from hardware.rotation_controller import RotationController, RotationPhase
from hardware.usfs import Usfs
from core.orientation import Orientation
from hardware.vl53l5cx_sensor import Vl53l5cxSensor
from matrix11x7 import Matrix11x7
from matrix11x7.fonts import font3x5

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
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

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Scan(AsyncBehaviour):
    NAME = 'scan'
    '''
    Performs a 360° rotational scan using VL53L5CX to build a complete
    environmental map, then publishes the recommended heading.
    
    Uses RotationController for encoder-based rotation tracking (no IMU required).
    
    Triggered by STUCK event, publishes SCAN event with chosen heading.
    '''
    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Scan.NAME, level)
        _component_registry = Component.get_registry()
        _motor_controller = _component_registry.get(MotorController.NAME)
        if _motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        
        # rotation controller
        self._rotation_controller = _component_registry.get(RotationController.NAME)
        if self._rotation_controller is None:
            raise MissingComponentError('rotation controller not available for Scan.')
        
        # IMU (available but currently unused)
        self._imu = _component_registry.get(Usfs.NAME)
        if self._imu is None:
            self._log.warning('IMU not available for Scan (not required).')
        elif not self._imu.enabled:
            self._imu.enable()
        
        # configuration
        _cfg = config['kros'].get('behaviour').get('scan')
        self._counter = itertools.count()
        self._verbose = _cfg.get('verbose', False)
        
        # rotation parameters (kept for reference, actual values in RotationController)
        self._angular_resolution = _cfg.get('angular_resolution', 5.0)  # degrees
        
        # matrix11x7 display
        self._use_matrix = True
        if self._use_matrix:
            _low_brightness    = 0.15
            _medium_brightness = 0.25
            _high_brightness   = 0.45
            self._matrix11x7 = Matrix11x7()
            self._matrix11x7.set_brightness(_medium_brightness)
        
        # eyeballs
        self._eyeballs = None
        self._use_eyeballs = True
        if self._use_eyeballs:
            from hardware.eyeballs import Eyeballs
            self._eyeballs = _component_registry.get(Eyeballs.NAME)
        
        # scan state
        self._scan_phase = ScanPhase.INACTIVE
        
        # heading markers for coordinate calculation
        self._stuck_heading_marker = None
        self._scan_start_marker = None
        
        # queue publisher
        self._enable_publishing = False
        self._queue_publisher = _component_registry.get(QueuePublisher.NAME)
        if self._queue_publisher is None:
            raise MissingComponentError('queue publisher not available for Scan.')
        
        # data collection
        self._vl53l5cx = _component_registry.get(Vl53l5cxSensor.NAME)
        if self._vl53l5cx is None:
            raise MissingComponentError('VL53L5CX sensor not available for Scan.')
        
        # data structures
        self._scan_readings = []  # [{angle: deg, distance_mm: [64 values], timestamp: sec}]
        self._scan_slices = {}  # processed slice data
        
        # superclass
        AsyncBehaviour.__init__(self, self._log, config, message_bus, message_factory, 
                              _motor_controller, level=level)
        self.add_event(Event.STUCK)
        
        self._log.info('ready.')

    @property
    def name(self):
        return Scan.NAME

    @property
    def is_ballistic(self):
        '''ballistic when actively scanning (not INACTIVE or IDLE)'''
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
        Begin scan sequence using RotationController.
        '''
        self._log.info(Fore.GREEN + 'initiating scan…')
        
        # mark where we started
        self._stuck_heading_marker = self._rotation_controller.push_heading_marker('stuck')
        
        self._scan_readings = []
        
        # delegate rotation to RotationController
        self._rotation_controller.initiate_rotation()
        
        self._scan_phase = ScanPhase.ACCEL
        self._log.info(Fore.GREEN + 'scan initiated')

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
        Main scan control loop orchestrator - delegates rotation to RotationController.
        '''
        if self._scan_phase == ScanPhase.INACTIVE or self._scan_phase == ScanPhase.IDLE:
            return
        
        try:
            # poll rotation controller
            current_time, elapsed, accumulated_rotation = self._rotation_controller.poll()
            
            match self._scan_phase:
                case ScanPhase.ACCEL:
                    self._handle_accel_phase(elapsed, accumulated_rotation, current_time)
                case ScanPhase.SCAN:
                    self._handle_scan_phase(accumulated_rotation, current_time)
                case ScanPhase.DECEL:
                    self._handle_decel_phase(elapsed, accumulated_rotation)
                case _:
                    self._log.warning('unexpected phase: {}'.format(self._scan_phase.name))
                    if self._eyeballs:
                        self._eyeballs.confused()
            
            if self._verbose and next(self._counter) % 5 == 0:
                self._log.info('phase: {}; accumulated: {:.1f}°; omega: {:.3f}'.format(
                    self._scan_phase.name, accumulated_rotation, 
                    self._rotation_controller.intent_vector[2]))
        
        except Exception as e:
            self._log.error("{} thrown while polling: {}".format(type(e), e))
            self._rotation_controller.cancel_rotation()
            self._scan_phase = ScanPhase.IDLE
            self.disable()

    def _handle_accel_phase(self, elapsed, accumulated_rotation, current_time):
        '''
        Handle acceleration phase: delegate to RotationController.
        '''
        if self._eyeballs:
            self._eyeballs.look_up()
        
        # delegate to RotationController
        self._rotation_controller.handle_accel_phase(elapsed, accumulated_rotation, current_time)
        
        # check if transitioned to SCAN phase
        if self._rotation_controller.rotation_phase == RotationPhase.SCAN:
            self._scan_phase = ScanPhase.SCAN
            # mark where SCAN phase starts
            self._scan_start_marker = self._rotation_controller.push_heading_marker('scan_start')

    def _handle_scan_phase(self, accumulated_rotation, current_time):
        '''
        Handle scan phase: delegate rotation to RotationController, capture VL53L5CX data.
        '''
        if self._eyeballs:
            self._eyeballs.look_stbd()
        
        # delegate rotation control to RotationController
        scan_complete = self._rotation_controller.handle_scan_phase(accumulated_rotation, current_time)
        
        # capture VL53L5CX reading
        distance_mm = self._vl53l5cx.get_distance_mm()
        if distance_mm is not None:
            # log current scan angle
            if next(self._counter) % 10 == 0:
                self._log.info('🐹 scan at {:.1f}°'.format(accumulated_rotation))
            
            # compute representative distance for display
            non_floor_rows = self._vl53l5cx.non_floor_rows
            distances_array = np.array(distance_mm).reshape(8, 8)
            obstacle_distances = distances_array[non_floor_rows, :]
            avg_distance_mm = obstacle_distances.mean()
            avg_distance_cm = int(avg_distance_mm / 10.0)
            
            # display distance in cm
            self._display_distance(avg_distance_cm)
            
            # store reading with angle relative to scan start (0-360)
            scan_rotation = accumulated_rotation
            self._scan_readings.append({
                'angle': scan_rotation,
                'distance_mm': distance_mm,
                'timestamp': current_time
            })
        else:
            self._log.warning('VL53L5CX returned None at {:.1f}°'.format(accumulated_rotation))
        
        # check if scan phase complete
        if scan_complete or self._rotation_controller.rotation_phase == RotationPhase.DECEL:
            self._scan_phase = ScanPhase.DECEL

    def _handle_decel_phase(self, elapsed, accumulated_rotation):
        '''
        Handle deceleration phase: delegate to RotationController, then process data.
        '''
        if self._eyeballs:
            self._eyeballs.look_down()
        
        # delegate deceleration to RotationController
        decel_complete = self._rotation_controller.handle_decel_phase(elapsed, accumulated_rotation)
        
        if decel_complete:
            if self._eyeballs:
                self._eyeballs.normal()
            
            # process captured data
            self._scan_slices = self._process_scan_data()
            self._analyze_scan_and_choose_heading()
            
            self._log.info('scan complete, total rotation: {:.1f}°'.format(accumulated_rotation))
            
            # return to idle
            self._scan_phase = ScanPhase.IDLE

    def _process_scan_data(self):
        '''
        Bins raw readings into 5° slices and averages multiple samples per slice.
        Returns dict: {slice_index: {angle: deg, avg_distance: [8x8], sample_count: n}}
        '''
        if len(self._scan_readings) == 0:
            self._log.warning('no scan readings captured')
            return {}
        
        # initialize slices
        num_slices = int(360.0 / self._angular_resolution)
        slices = {i: {'samples': [], 'angles': []} for i in range(num_slices)}
        
        # bin readings into slices
        for reading in self._scan_readings:
            angle = reading['angle']
            slice_idx = int(angle / self._angular_resolution) % num_slices
            slices[slice_idx]['samples'].append(np.array(reading['distance_mm']).reshape(8, 8))
            slices[slice_idx]['angles'].append(angle)
        
        # average samples within each slice
        processed = {}
        for idx, data in slices.items():
            if len(data['samples']) > 0:
                avg_distance = np.mean(data['samples'], axis=0)
                processed[idx] = {
                    'angle': np.mean(data['angles']),
                    'avg_distance': avg_distance,
                    'sample_count': len(data['samples'])
                }
            else:
                self._log.warning('slice {} has no samples'.format(idx))
        
        self._log.info('processed {} slices from {} readings'.format(
            len(processed), len(self._scan_readings)))
        return processed

    def _analyze_scan_and_choose_heading(self):
        '''
        Analyzes 360° scan data to find best heading.
        Uses non_floor_rows from VL53L5CX calibration to focus on obstacles.
        Publishes SCAN message with chosen heading to message bus.
        '''
        non_floor_rows = self._vl53l5cx.non_floor_rows
        self._log.info('analyzing scan using non-floor rows: {}'.format(non_floor_rows))
        
        # find slice with maximum average distance
        best_slice_idx = None
        max_avg_distance = 0.0
        
        for idx, slice_data in self._scan_slices.items():
            angle = slice_data['angle']
            distances = slice_data['avg_distance'][non_floor_rows, :]
            avg_distance = distances.mean()
            
            if avg_distance > max_avg_distance:
                max_avg_distance = avg_distance
                best_slice_idx = idx
            
            if self._verbose:
                self._log.info('slice {}: angle={:.1f}°, avg_distance={:.0f}mm, samples={}'.format(
                    idx, angle, avg_distance, slice_data['sample_count']))
        
        if best_slice_idx is not None:
            # best angle relative to scan start (0-360)
            chosen_heading = self._scan_slices[best_slice_idx]['angle']
            
            self._log.info('chosen heading: {:.1f}° (slice {}, avg_distance={:.0f}mm)'.format(
                chosen_heading, best_slice_idx, max_avg_distance))
        else:
            # no valid slices - default to straight ahead
            chosen_heading = 0.0
            self._log.warning('no valid scan data, defaulting to 0° heading')
        
        # display final chosen heading on matrix
        self._display_distance(int(chosen_heading))
        
        # publish SCAN message with chosen heading
        self._publish_message(chosen_heading)

    def _publish_message(self, heading):
        '''
        Publishes a SCAN message containing the heading value.
        '''
        self._log.info("publishing scan message with heading '{:.1f}°'…".format(heading))
        if self._enable_publishing:
            try:
                _message = self._message_factory.create_message(Event.SCAN, heading)
                self._queue_publisher.put(_message)
            except Exception as e:
                self._log.error('{} encountered when publishing message: {}\n{}'.format(
                    type(e), e, traceback.format_exc()))
        else:
            self._log.info('publishing disabled.')

    def _display_distance(self, value):
        '''
        Display value on matrix11x7.
        '''
        if self._use_matrix:
            self._matrix11x7.clear()
            self._matrix11x7.write_string('{:>3}'.format(value), y=1, font=font3x5)
            self._matrix11x7.show()

    def enable(self):
        if self.enabled:
            self._log.warning('already enabled.')
            return
        self._vl53l5cx.enable()
        self._rotation_controller.enable()
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
        self._vl53l5cx.disable()
        self._scan_phase = ScanPhase.INACTIVE
        AsyncBehaviour.disable(self)
        if self._eyeballs:
            self._eyeballs.disable()
        if self._use_matrix:
            self._matrix11x7.clear()
            self._matrix11x7.show()
        self._log.info('scan disabled.')

#EOF
