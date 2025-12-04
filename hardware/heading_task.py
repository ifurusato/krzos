#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# HeadingTask: A Component to rotate and hold a robot at a specified heading,
# with optional persistent mode and hysteresis. Now uses external clock for PID timing.
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
    now using external clock timing for the PID loop.

    - With persistent=True, the task runs until explicitly disabled, always responding to heading/trim changes.
    - With persistent=False, the task automatically disables itself once the heading has been achieved and held
      within deadband for a given number of cycles.

    Supports hysteresis to prevent repeated re-activation from small IMU drift.
    '''
    def __init__(self,
                 config,
                 motor_controller,
                 external_clock,
                 imu,
                 target_heading,
                 kp=0.01,
                 deadband=3.0,
                 hysteresis=5.0,
                 max_speed=0.4,
                 settle_cycles=6,
                 persistent=True,
                 name='heading-task',
                 level=Level.INFO):
        self._log = Logger(name, level)
        super().__init__(self._log, suppressed=False, enabled=False)
        self._config = config
        self._motor_controller = motor_controller
        self._external_clock = external_clock
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
        self._settled = 0
        self._log.info('ready.')

    def shortest_angle_diff(self, target, current):
        '''
        Returns the shortest angle difference in degrees (-180 to 180).
        '''
        diff = (target - current + 180) % 360 - 180
        return diff

    def enable(self):
        '''
        Enable the HeadingTask and start the control loop using the external clock.
        '''
        if self.enabled:
            self._log.warning('already enabled.')
            return
        self._log.info("enabling HeadingTask to {:.1f}° (persistent={})…".format(self._target_heading, self._persistent))
        super().enable()
        self._running = True
        self._motor_controller.set_steering_mode(SteeringMode.ROTATE)
        self._in_deadband = False
        self._settled = 0
        self._log.info("using external clock for HeadingTask timing.")
        self._external_clock.add_callback(self._external_clock_callback)

    def _external_clock_callback(self):
        if not self._running or self.closed:
            return
        self._run_loop_tick()

    def _run_loop_tick(self):
        '''
        One tick of the PID loop, called by external clock.
        '''
        self._log.info("_run_loop_tick")
        self._imu.poll()
        current_yaw = self._imu.corrected_yaw
        _yaw_trim = self._imu.yaw_trim
        error = self.shortest_angle_diff(self._target_heading, current_yaw)
        self._log.info(          'current: {:6.2f}'.format(current_yaw)
                + Fore.MAGENTA + ' | target: {:6.2f}'.format(self._target_heading)
                + Fore.RED     + ' | error: {:+6.2f}'.format(error)
                + Fore.GREEN   + ' | trim: {:+6.2f}'.format(_yaw_trim))

        if self._in_deadband:
            if abs(error) > self._hysteresis:
                self._in_deadband = False  # resume control if error exceeds hysteresis
                self._settled = 0
                self._log.error("error left deadband (>{:.2f}), resuming control.".format(self._hysteresis))
            else:
                self._motor_controller.set_speed(Orientation.PORT, 0.0)
                self._motor_controller.set_speed(Orientation.STBD, 0.0)
                return

        if abs(error) < self._deadband:
            self._settled += 1
            self._motor_controller.set_speed(Orientation.PORT, 0.0)
            self._motor_controller.set_speed(Orientation.STBD, 0.0)
            self._in_deadband = True
            if not self._persistent and self._settled >= self._settle_cycles:
                self._log.info("heading achieved for {} cycles, disabling.".format(self._settled))
                self.disable() # will cleanup
        else:
            self._settled = 0
            output = max(min(self._kp * error, self._max_speed), -self._max_speed)
            self._motor_controller.set_speed(Orientation.PORT, output)
            self._motor_controller.set_speed(Orientation.STBD, output)

    def disable(self):
        '''
        Disable the HeadingTask, stop motors, cleanup, and remove external clock callback.
        '''
        self._log.info("disabling HeadingTask…")
        self._running = False
        self._external_clock.remove_callback(self._external_clock_callback)
        super().disable()
        self._cleanup()

    def suppress(self):
        '''
        Suppress the HeadingTask and cleanup.
        '''
        self._log.info("suppressing HeadingTask…")
        self._running = False
        self._external_clock.remove_callback(self._external_clock_callback)
        super().suppress()
        self._cleanup()

    def _cleanup(self):
        '''
        Cleanup any motor lambdas/settings used by the task.
        '''
        self._log.info("cleaning up HeadingTask (removing rotation lambdas)…")
        self._motor_controller.reset_rotating()

    def close(self):
        '''
        Permanently close and disable the HeadingTask.
        '''
        self._log.info("closing HeadingTask…")
        self._running = False
        self._external_clock.remove_callback(self._external_clock_callback)
        self._cleanup()
        super().close()

#EOF
