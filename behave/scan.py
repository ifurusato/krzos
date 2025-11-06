#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-01-05
# modified: 2025-01-05

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
    environmental map, then orients robot toward most open direction.

    ARCHITECTURE: SEPARATE BEHAVIOR

    This is a separate behavior called Scan (or LidarScan), not part of Scout.
    Reasons:

    1. Different purpose: Scout is continuous reactive navigation. This is
       episodic problem-solving.
    2. Different lifecycle: Scout runs constantly. Scan runs once when
       triggered, then completes.
    3. Different priority: Scan needs to be ballistic (highest priority,
       suppresses everything) while Scout is cooperative.
    4. Reusability: Other behaviors might want to trigger a scan (not just
       Scout/Roam).

    STUCK DETECTION

    The "stuck" flag could come from multiple sources:
    - Scout: Detecting max_open_distance < threshold for N consecutive seconds
    - Roam: Detecting lack of forward progress (odometry shows no movement
      despite motor commands)
    - Both: Cross-correlation (trying to move but all directions blocked)

    Stuck detection should be in a separate monitor that watches both
    behaviors and raises a STUCK event when conditions are met.

    OPERATION

    1. Triggered by STUCK event
    2. Rotates 360° at controlled speed (maybe 0.2 rad/s for stable readings)
    3. Captures VL53L5CX readings at angular intervals (e.g., every 5-10°)
    4. Builds cylindrical occupancy grid (36-72 slices × 8 columns × relevant rows)
    5. Analyzes grid to find widest open corridor
    6. Rotates to that heading
    7. Sets a target heading for Scout/Roam
    8. Completes (becomes non-ballistic, allowing normal behaviors to resume)

    DATA STRUCTURE

    Resolution: 10° slices, 8 columns, 3-4 obstacle rows
    scan_data = np.zeros((36, 8, 4))  # [angle_slice, column, row]

    INTEGRATION POINTS

    In Scout (add stuck detection):
        if max_open_distance < 300 and self._stuck_counter > 50:  # ~10 seconds
            self._message_bus.publish(Event.STUCK)

    In Roam (add stuck detection):
        if commanded_vy > 0 and actual_distance_traveled < threshold:
            self._message_bus.publish(Event.STUCK)

    Scan subscribes to STUCK event and activates when received.

    ADVANTAGES OF THIS APPROACH

    1. Clean separation: Scout stays focused on reactive navigation
    2. Reusable: Any behavior can trigger a scan
    3. Testable: Can test scan independently
    4. Extensible: Could add other recovery behaviors (back up, try
       alternate path, etc.)
    5. Clear priority: Ballistic flag ensures scan runs uninterrupted

    IMPLEMENTATION COMPLEXITY ESTIMATE

    - Scan behavior core: 4-6 hours (rotation control, data capture, grid building)
    - Analysis algorithm: 2-4 hours (find widest corridor, handle edge cases)
    - Stuck detection: 2-3 hours (thresholds, counters, event publishing)
    - Integration/testing: 2-4 hours
    - Total: ~10-17 hours of focused work
    '''
    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Scan.NAME, level)
        _component_registry = Component.get_registry()
        _motor_controller = _component_registry.get(MotorController.NAME)
        if _motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        # IMU for heading tracking
        self._imu = _component_registry.get(Usfs.NAME)
        if self._imu is None:
            raise MissingComponentError('IMU not available for Scan.')
        if not self._imu.enabled:
            self._imu.enable()
#       self._imu.set_fixed_yaw_trim(-72.5)
        self._imu.set_verbose(True)
        # configuration
        _cfg = config['kros'].get('behaviour').get('scan')
        self._counter = itertools.count()
        self._verbose = _cfg.get('verbose', False)
        # rotation parameters
        self._rotation_speed = _cfg.get('rotation_speed', 15.0)  # degrees/sec, convert to rad/s
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
        self._use_corrected_yaw = True
        self._scanning = False
        self._scan_phase = ScanPhase.IDLE
        self._start_heading = None
        self._start_time = None
        AsyncBehaviour.__init__(self, self._log, config, message_bus, message_factory, _motor_controller, level=level)
        self.add_event(Event.STUCK)
        # AsyncBehaviour provides motors and step info
#       self._motor_pfwd
#       self._motor_sfwd
#       self._motor_paft
#       self._motor_saft
#       self._steps_per_rotation
#       self._steps_per_degree
        self._log.info('ready.')

    @property
    def name(self):
        return Scan.NAME

    @property
    def is_ballistic(self):
        return self._scanning  # Only ballistic when actively scanning

    @property
    def priority(self):
        return 1.0 if self._scanning else 0.0

    def callback(self):
        self._log.info('scan behaviour callback.')
        raise Exception('UNSUPPORTED callback')

    def execute(self, message):
        '''
        Called when STUCK message received via message bus.
        '''
        if message.event is Event.STUCK:
            self._log.info('STUCK event received, initiating scan…')
            if not self._scanning:
                self._initiate_scan()
            else:
                self._log.warning('scan already in progress, ignoring STUCK event')
        else:
            self._log.warning('unexpected message event: {}'.format(message.event))

    def _poll_imu(self):
        '''
        Calling this method updates self._
        '''
        self._imu.poll()
        _corrected_yaw = self._imu.corrected_yaw
        _yaw = self._imu.yaw
        self._log.info(Fore.MAGENTA + '👿 yaw: {:.1f}°; corrected: {:.1f}°'.format(_corrected_yaw, _yaw))
        if self._use_corrected_yaw:
            return _corrected_yaw
        else:
            return _yaw

    def _initiate_scan(self):
        '''
        Begin scan sequence by enabling AsyncBehaviour polling.
        '''
        self._log.info(Fore.GREEN + '🤢 initiating scan…')
        self._scanning = True
        self._scan_phase = ScanPhase.ACCEL
        self._start_time = time.time()
        for i in range(10):
            _heading = self._poll_imu()
            self._log.info(Fore.GREEN + '🤢 [{}] test scan: {:.1f}°'.format(i, _heading))
            time.sleep(0.25)
        self._start_heading = self._poll_imu()
        self._log.info(Fore.GREEN + '🤢 scan initiated from heading: {:.1f}°'.format(self._start_heading))

    def start_loop_action(self):
        '''
        Warm up the IMU at the beginning of the polling process.
        '''
        pass

    def stop_loop_action(self):
        pass

    def rotation_placeholder(self):
        '''
        A placeholder for future for rotation monitoring.
        '''
        pfwd_steps = self._motor_pfwd.steps
        sfwd_steps = self._motor_sfwd.steps
        paft_steps = self._motor_paft.steps
        saft_steps = self._motor_saft.steps
        rotation_steps = int((pfwd_steps - sfwd_steps + paft_steps - saft_steps) / 4)
        self._log.info(Fore.WHITE + '😡 pfwd={}; sfwd={}; paft={}; saft={}; rot: {} '.format(pfwd_steps, sfwd_steps, paft_steps, saft_steps, rotation_steps))
        # get raw gyroscope data
        gx, gy, gz = self._imu.gyroscope  # gz is yaw rate in degrees/sec
        self._log.info(Fore.WHITE + '😡 gx={:.2f}; gy={:.2f}; gz={:.2f}'.format(gx, gy, gz))

    async def _poll(self):
        '''
        Main scan control loop - handles acceleration, rotation, deceleration.
        '''
        if not self._scanning:
            return
        try:
            current_time = time.time()
            elapsed = current_time - self._start_time
            current_heading = self._poll_imu()
            if current_heading is None:
                self._log.warning('poll: IMU did not return heading')
                return
            self._log.info('🍏 phase: {}; heading: {:.1f}°'.format(self._scan_phase, current_heading))
            self.rotation_placeholder()
            match self._scan_phase:
                case ScanPhase.ACCEL:
                    self._log.info(Style.DIM + 'accelerating…')
                    if self._eyeballs:
                        self._eyeballs.look_up()
                    # linear acceleration to full speed
                    progress = min(elapsed / self._accel_time, 1.0)
                    omega = self._rotation_speed_rad * progress
                    self._intent_vector = (0.0, 0.0, omega)
                    if progress >= 1.0:
                        self._scan_phase = ScanPhase.SCAN
                        self._start_heading = current_heading  # reset reference after accel
                        self._start_time = current_time
                        self._log.info('acceleration complete, beginning 360° rotation from {:.1f}°'.format(current_heading))

                case ScanPhase.SCAN:
                    self._log.info(Style.DIM + 'scanning…')
                    if self._eyeballs:
                        self._eyeballs.look_stbd()
                    # maintain constant rotation speed
                    omega = self._rotation_speed_rad
                    self._intent_vector = (0.0, 0.0, omega)
                    # check if we've completed 360°
                    heading_delta = (current_heading - self._start_heading + 180.0) % 360.0 - 180.0
                    if abs(heading_delta) < 5.0 and elapsed > 20.0:  # Allow minimum time, avoid false trigger
                        self._scan_phase = ScanPhase.DECEL
                        self._start_time = current_time
                        self._log.info('360° rotation complete at {:.1f}°, decelerating…'.format(current_heading))
                    else:
                        self._log.info(Fore.BLUE + '📘 scanning: {:.1f}° to {:.1f}°…'.format(current_heading, self._start_heading))

                case ScanPhase.DECEL:
                    self._log.info(Style.DIM + 'decelerating…')
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
                        self._scanning = False
                        self._scan_phase = ScanPhase.IDLE
                        self._log.info('Scan complete, robot stopped.')

                case ScanPhase.IDLE:
                    self._log.warning('unexpected IDLE phase in poll.')
                    if self._eyeballs:
                        self._eyeballs.confused()

            if self._verbose and next(self._counter) % 5 == 0:
                self._log.info('scan phase: {}; heading: {:.1f}°; omega: {:.3f}'.format(
                    self._scan_phase, current_heading, self._intent_vector[2]))

        except Exception as e:
            self._log.error("{} thrown while polling: {}".format(type(e), e))
            self.clear_intent_vector()
            self._scanning = False
            self.disable()

    def ping(self):
        self._log.warning('🐹 ping.')

    def enable(self):
        if self.enabled:
            self._log.warning('already enabled.')
            return
        self._log.info('enabling scan...')
        AsyncBehaviour.enable(self)
        current_heading = self._poll_imu()
        self._log.info('🍎 enable: start loop action with heading: {}'.format(current_heading))
        time.sleep(0.5)
        if self._eyeballs:
            self._eyeballs.enable()
        self._log.info('scan enabled.')

    def disable(self):
        if not self.enabled:
            self._log.warning('already disabled.')
            return
        self._log.info('disabling scan…')
        self._scanning = False
        self._scan_phase = ScanPhase.IDLE
        self.clear_intent_vector()
        AsyncBehaviour.disable(self)
        if self._eyeballs:
            self._eyeballs.disable()
        self._log.info('scan disabled.')

#EOF
