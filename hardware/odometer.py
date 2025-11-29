#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-11
# modified: 2025-11-29
#
# Odometer for Mecanum-based robot chassis.
#
# This class provides robot-level velocity and odometry calculation for a
# four-motor Mecanum drive. It tracks changes in step counts from each wheel
# and computes:
#
#  - Instantaneous longitudinal (v_x), lateral (v_y), and rotational (omega)
#    velocity,
#  - Cumulative pose: (x, y, theta) displacement of the robot since
#    initialization or last reset.
#
# The kinematic calculations follow standard Mecanum equations for velocity and
# odometry. Wheel geometry (diameter) and encoder properties (steps per revolution)
# must be provided.
#
# Usage:
#
#   odom = Odometer(config)
#   odom.update({'pfwd': pf, 'sfwd': sf, 'paft': pa, 'saft': sa}, timestamp)
#   vx, vy, omega = odom.get_velocity()
#   x, y, theta = odom.get_pose()
#   odom.reset()
#
# See also the legacy Velocity class for notes on robot geometry.
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

import math
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component

class Odometer(Component):
    NAME = 'odometer'
    '''
    Computes robot velocity and pose (odometry) from the step counts of all four drive motors on a Mecanum robot.

    - update():        Call with dict of {motor: absolute_steps} and timestamp (seconds, e.g. time.monotonic()).
    - get_velocity():  Returns (vx, vy, omega) where vx = forward cm/s, vy = lateral cm/s, omega = rad/s.
    - get_pose():      Returns (x, y, theta) where x = forward cm, y = lateral cm, theta = heading in radians.
    - reset():         Resets cumulative pose and previous readings.
    '''
    def __init__(self, config, suppressed=False, enabled=True, level=Level.INFO):
        self._config = config
        self._log = Logger(Odometer.NAME, level)
        Component.__init__(self, self._log, suppressed=suppressed, enabled=enabled)
        # obtain geometry from config (use 'kros.geometry')
        _cfg = config['kros'].get('geometry')
        self._wheel_diameter_mm      = _cfg.get('wheel_diameter')
        self._steps_per_rotation     = _cfg.get('steps_per_rotation')
        self._wheel_base_mm          = _cfg.get('wheel_base')
        self._wheel_track_mm         = _cfg.get('wheel_track')
        self._wheel_circumference_cm = self._wheel_diameter_mm * math.pi / 10.0
        self._steps_per_cm = self._steps_per_rotation / self._wheel_circumference_cm
        self._log.info('wheel base:             ' + Fore.GREEN + ' {:4.1f}mm'.format(self._wheel_base_mm))
        self._log.info('wheel track:            ' + Fore.GREEN + ' {:4.1f}mm'.format(self._wheel_track_mm))
        self._log.info('wheel diameter:         ' + Fore.GREEN + ' {:4.1f}mm'.format(self._wheel_diameter_mm))
        self._log.info('wheel circumference:    ' + Fore.GREEN + ' {:7.4f}cm'.format(self._wheel_circumference_cm))
        self._log.info('encoder steps/rotation: ' + Fore.GREEN + ' {:7.2f}'.format(self._steps_per_rotation))
        self._log.info('conversion constant:    ' + Fore.GREEN + ' {:7.4f} steps/cm'.format(self._steps_per_cm))
        _test_velocity = self.steps_to_cm(self._steps_per_rotation)
        self._log.info('example conversion:     ' + Fore.GREEN + ' {:7.4f}cm/rotation'.format(_test_velocity))
        # convert to cm for computation
        self._wheel_diameter_cm  = self._wheel_diameter_mm / 10.0
        self._steps_per_rev      = float(self._steps_per_rotation) / 4.0
        self._wheelbase_cm       = self._wheel_base_mm / 10.0
        self._track_cm           = self._wheel_track_mm / 10.0
        self._wheel_radius_cm    = self._wheel_diameter_cm / 2.0
        self._step_cm            = math.pi * self._wheel_diameter_cm / self._steps_per_rev
        self._log.info(
            "wheel diameter: {:.2f}cm, wheelbase: {:.2f}cm, track: {:.2f}cm, steps/rev: {:.2f}".format(
                self._wheel_diameter_cm, self._wheelbase_cm, self._track_cm, self._steps_per_rev
            )
        )
        # internal state
        self._last_steps = None  # dict of last {motor: step_count}
        self._last_time  = None  # float (seconds)
        self._x     = 0.0   # forward position (cm)
        self._y     = 0.0   # lateral position (cm)
        self._theta = 0.0   # heading (radians)
        self._vx    = 0.0   # forward velocity (cm/s)
        self._vy    = 0.0   # lateral velocity (cm/s)
        self._omega = 0.0   # yaw rate (rad/s)
        self._log.info('ready.')

    @property
    def steps_per_rotation(self):
        return self._steps_per_rotation

    @property
    def steps_per_cm(self):
        return self._steps_per_cm

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def set_pose(self, x, y, theta):
        '''
        Set the robot's pose to a specific position and heading.
        Used for initialization or resetting odometry.
        
        :param x:     x position in cm
        :param y:     y position in cm  
        :param theta: heading in radians
        '''
        self._x = x
        self._y = y
        self._theta = theta
        self._log.info('pose set to: x={:.2f}cm, y={:.2f}cm, theta={:.2f}rad ({:.1f}°)'.format(
            x, y, theta, math.degrees(theta)))

    def steps_to_cm(self, steps):
        return steps / self._steps_per_cm

    def reset(self):
        '''
        Resets the odometer/cumulative pose and last-step state.
        '''
        self._last_steps = None
        self._last_time = None
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0
        self._vx    = 0.0
        self._vy    = 0.0
        self._omega = 0.0
        self._log.info('odometry reset.')

    def update(self, step_counts: dict, timestamp: float):
        '''
        Call at regular intervals with latest step counts for all four motors and time (seconds).

        step_counts:  dict with keys: 'pfwd', 'sfwd', 'paft', 'saft'
        timestamp:    floating-point time in seconds (e.g. from time.monotonic())
        '''
        if not self.enabled:
            self._log.warning('disabled.')
            return
        if self._last_steps is not None and self._last_time is not None:
            dt = timestamp - self._last_time
            if dt <= 0.0:
                return  # ignore if no real elapsed time
            # step deltas
            ds_pfwd  = step_counts['pfwd']  - self._last_steps['pfwd']
            ds_sfwd  = step_counts['sfwd']  - self._last_steps['sfwd']
            ds_paft  = step_counts['paft']  - self._last_steps['paft']
            ds_saft  = step_counts['saft']  - self._last_steps['saft']
            # convert to per-wheel distances in cm
            d_pfwd = ds_pfwd * self._step_cm
            d_sfwd = ds_sfwd * self._step_cm
            d_paft = ds_paft * self._step_cm
            d_saft = ds_saft * self._step_cm

            # mecanum chassis kinematics - velocities in ROBOT BODY FRAME
            vx = (d_pfwd - d_sfwd - d_paft + d_saft) / 4.0 / dt   # lateral (right+)
            vy = (d_pfwd + d_sfwd + d_paft + d_saft) / 4.0 / dt   # longitudinal (forward+)
            omega = (d_pfwd - d_sfwd + d_paft - d_saft) / (4.0 * ((self._wheelbase_cm + self._track_cm) / 2.0)) / dt

            # store body frame velocities
            self._vx = vx
            self._vy = vy
            self._omega = omega
            
            # integrate pose: transform body frame velocities to odometry frame
            cos_t = math.cos(self._theta)
            sin_t = math.sin(self._theta)
            
            # transform body velocity to odometry frame displacement
            dx = (vx * cos_t + vy * sin_t) * dt
            dy = (-vx * sin_t + vy * cos_t) * dt
            dtheta = omega * dt
            
            self._x += dx
            self._y += dy
            self._theta += dtheta
            
            # normalize theta to [-π, π]
            while self._theta > math.pi:
                self._theta -= 2 * math.pi
            while self._theta < -math.pi:
                self._theta += 2 * math.pi
                
        # save per-tick state
        self._last_steps = dict(step_counts)
        self._last_time = timestamp

    def print_info(self):
        '''
        Prints the current intent vector and pose.
        ''' 
        vx, vy, omega = self.get_velocity()
        x, y, theta = self.get_pose()
        self._log.info(Style.DIM + 'intent: ({:.2f}, {:.2f}, {:.2f});\tpose: ({:.2f}cm, {:.2f}cm, {:.2f}rad)'.format(vx, vy, omega, x, y, theta))

    def get_velocity(self):
        '''
        Returns (vx, vy, omega): vx (forward cm/s), vy (lateral cm/s), omega (rad/s).
        '''
        return self._vx, self._vy, self._omega

    def get_pose(self):
        '''
        Returns (x, y, theta): x (forward cm), y (lateral cm), theta (radians).
        '''
        return self._x, self._y, self._theta

    def enable(self):
        if not self.closed:
            self._enabled = True
            self._log.info('enabled.')

    def disable(self):
        if self.enabled:
            self._enabled = False
            self._log.info('disabled.')

    def close(self):
        self.disable()
        self._closed = True
        self._log.info('closed.')

#EOF
