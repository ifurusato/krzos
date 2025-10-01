#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# HeadingTask: A Component to rotate and hold a robot at a specified heading,
# with optional persistent mode and hysteresis.
#
# Copyright 2025 by Murray Altheim. All rights reserved.
#
# author:   Murray Altheim
# created:  2025-09-30

import time
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.orientation import Orientation
from core.steering_mode import SteeringMode

class HeadingTask(Component):
    '''
    A Component that rotates the robot to a specified heading using a proportional controller,
    responding in real time to IMU trim changes via a DigitalPotentiometer if in persistent mode.
    
    - With persistent=True, the task runs until explicitly disabled, always responding to heading/trim changes.
    - With persistent=False, the task automatically disables itself once the heading has been achieved and held
      within deadband for a given number of cycles.

    Supports hysteresis to prevent repeated re-activation from small IMU drift.
    '''
    def __init__(self,
                 motor_controller, 
                 imu,
                 target_heading,
                 kp=0.02,
                 deadband=2.0,
                 hysteresis=4.0,
                 max_speed=0.4,
                 settle_cycles=6,
                 persistent=True,
                 name='heading-task',
                 level=Level.INFO):
        self._log = Logger(name, level)
        super().__init__(self._log, suppressed=False, enabled=False)
        self._motor_controller = motor_controller
        self._imu = imu
        self._target_heading = target_heading
        self._kp = kp
        self._deadband = deadband
        self._hysteresis = hysteresis
        self._max_speed = max_speed
        self._settle_cycles = settle_cycles
        self._persistent = persistent
        self._running = False
        self._in_deadband = False
        self._log.info('ready.')

    def shortest_angle_diff(self, target, current):
        '''
        Returns the shortest angle difference in degrees (-180 to 180).
        '''
        diff = (target - current + 180) % 360 - 180
        return diff

    def enable(self):
        '''
        Enable the HeadingTask and start the control loop.
        '''
        if self.enabled:
            self._log.warning('already enabled.')
            return
        self._log.info(Fore.CYAN + f"Enabling HeadingTask to {self._target_heading:.1f}° (persistent={self._persistent})...")
        super().enable()
        self._running = True
        self._run_loop()

    def _run_loop(self):
        '''
        Main control loop: rotate to and hold the target heading (persistent or non-persistent).
        '''
        self._motor_controller.set_steering_mode(SteeringMode.ROTATE)
        self._in_deadband = False
        settled = 0
        while self._running and not self.closed:
            self._imu.poll()
            current_yaw = self._imu.corrected_yaw
            _yaw_trim = self._imu.yaw_trim
            error = self.shortest_angle_diff(self._target_heading, current_yaw)
            self._log.info('Current: {:6.2f} | Target: {:6.2f} | Error: {:+6.2f}'.format(
                current_yaw, self._target_heading, error) + Fore.GREEN + ' | {:+6.2f}'.format(_yaw_trim))

            if self._in_deadband:
                if abs(error) > self._hysteresis:
                    self._in_deadband = False  # Resume control if error exceeds hysteresis
                    settled = 0
                    self._log.info(Fore.MAGENTA + f"Error left deadband (>{self._hysteresis:.2f}), resuming control.")
                else:
                    self._motor_controller.set_speed(Orientation.PORT, 0.0)
                    self._motor_controller.set_speed(Orientation.STBD, 0.0)
                    time.sleep(0.05)
                    continue

            if abs(error) < self._deadband:
                settled += 1
                self._motor_controller.set_speed(Orientation.PORT, 0.0)
                self._motor_controller.set_speed(Orientation.STBD, 0.0)
                self._in_deadband = True
                if not self._persistent and settled >= self._settle_cycles:
                    self._log.info(Fore.GREEN + f"Heading achieved for {settled} cycles, disabling.")
                    self.disable()  # Will break the loop and cleanup
                    break
            else:
                settled = 0
                output = max(min(self._kp * error, self._max_speed), -self._max_speed)
                self._motor_controller.set_speed(Orientation.PORT, output)
                self._motor_controller.set_speed(Orientation.STBD, output)
            time.sleep(0.05)
        self._cleanup()

    def disable(self):
        '''
        Disable the HeadingTask, stop motors, and cleanup.
        '''
        self._log.info(Fore.YELLOW + "Disabling HeadingTask.")
        self._running = False
        super().disable()
        self._cleanup()

    def suppress(self):
        '''
        Suppress the HeadingTask and cleanup.
        '''
        self._log.info(Fore.YELLOW + "Suppressing HeadingTask.")
        self._running = False
        super().suppress()
        self._cleanup()

    def _cleanup(self):
        '''
        Cleanup any motor lambdas/settings used by the task.
        '''
        self._log.info(Fore.CYAN + "Cleaning up HeadingTask (removing rotation lambdas).")
        self._motor_controller.reset_rotating()

    def close(self):
        '''
        Permanently close and disable the HeadingTask.
        '''
        self._log.info(Fore.YELLOW + "Closing HeadingTask.")
        self._running = False
        self._cleanup()
        super().close()

#EOF
