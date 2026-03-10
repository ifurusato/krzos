#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-11
# modified: 2026-03-08
#
# Odometer for Mecanum-based robot chassis.
#
# This class provides robot-level velocity and odometry calculation for a
# four-motor Mecanum drive. It tracks changes in step counts from each wheel
# and computes:
#
#  - Instantaneous lateral (v_x), longitudinal (v_y), and rotational (omega)
#    velocity,
#  - Cumulative pose: (x, y, theta) displacement of the robot since
#    initialization or last reset.
#
# The kinematic calculations follow standard Mecanum equations for velocity and
# odometry. Wheel geometry (diameter) and encoder properties (steps per revolution)
# must be provided.
#
# Robot body frame convention:
#   X: port-starboard (lateral), starboard positive
#   Y: forward-backward (longitudinal), forward positive
#
# All distances and positions are in millimetres.
#
# Usage:
#
#   odom = Odometer(config)
#   odom.update({'pfwd': pf, 'sfwd': sf, 'paft': pa, 'saft': sa}, timestamp)
#   vx, vy, omega = odom.velocity
#   x, y, theta = odom.pose
#   odom.reset()
#
# See also the legacy Velocity class for notes on robot geometry.
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

from math import pi as π
from math import isclose, degrees, cos, sin
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.util import Util
from core.component import Component

class Odometer(Component):
    NAME = 'odometer'
    '''
    Computes robot velocity and pose (odometry) from the step counts of all
    four drive motors on a Mecanum robot.

    Robot body frame convention:
      X: port-starboard (lateral), starboard positive
      Y: forward-backward (longitudinal), forward positive

    All distances and positions are in millimetres.

    - update():    Call with dict of {motor: absolute_steps} and timestamp
                   (seconds, e.g. time.monotonic()).
    - velocity:    Returns (vx, vy, omega) where vx = lateral mm/s,
                   vy = longitudinal mm/s, omega = rad/s.
    - pose:        Returns (x, y, theta) where x = lateral mm,
                   y = longitudinal mm, theta = heading in radians.
    - reset():     Resets cumulative pose and previous readings.
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
        self._wheel_circumference_mm = self._wheel_diameter_mm * π
        self._steps_per_mm           = self._steps_per_rotation / self._wheel_circumference_mm
        self._log.info('wheel base:             ' + Fore.GREEN + ' {:4.1f}mm'.format(self._wheel_base_mm))
        self._log.info('wheel track:            ' + Fore.GREEN + ' {:4.1f}mm'.format(self._wheel_track_mm))
        self._log.info('wheel diameter:         ' + Fore.GREEN + ' {:4.1f}mm'.format(self._wheel_diameter_mm))
        self._log.info('wheel circumference:    ' + Fore.GREEN + ' {:7.4f}mm'.format(self._wheel_circumference_mm))
        self._log.info('encoder steps/rotation: ' + Fore.GREEN + ' {:7.2f}'.format(self._steps_per_rotation))
        self._log.info('conversion constant:    ' + Fore.GREEN + ' {:7.4f} steps/mm'.format(self._steps_per_mm))
        _test_velocity = self.steps_to_mm(self._steps_per_rotation)
        self._log.info('example conversion:     ' + Fore.GREEN + ' {:7.4f}mm/rotation'.format(_test_velocity))
        # derived geometry in mm
        self._wheel_radius_mm    = self._wheel_diameter_mm / 2.0
        self._steps_per_rev      = float(self._steps_per_rotation) / 4.0
        self._step_mm            = π * self._wheel_diameter_mm / self._steps_per_rev
        self._log.info(
            "wheel diameter: {:.2f}mm, wheelbase: {:.2f}mm, track: {:.2f}mm, steps/rev: {:.2f}".format(
                self._wheel_diameter_mm, self._wheel_base_mm, self._wheel_track_mm, self._steps_per_rev
            )
        )
        self.__callbacks = []
        # internal state
        self._last_steps = None  # dict of last {motor: step_count}
        self._last_time  = None  # float (seconds)
        self._x     = 0.0   # lateral position (mm)
        self._y     = 0.0   # longitudinal position (mm)
        self._theta = 0.0   # heading (radians)
        self._vx    = 0.0   # lateral velocity (mm/s)
        self._vy    = 0.0   # longitudinal velocity (mm/s)
        self._omega = 0.0   # yaw rate (rad/s)
        self._log.info('ready.')

    @property
    def steps_per_rotation(self):
        '''
        Returns the configured steps per wheel rotation value.
        '''
        return self._steps_per_rotation

    @property
    def steps_per_mm(self):
        '''
        Returns the configured steps per mm value, calculated from the robot geometry.
        '''
        return self._steps_per_mm

    @property
    def pose(self):
        '''
        Returns (x, y, theta): x (lateral mm), y (longitudinal mm), theta (radians).
        '''
        return self._x, self._y, self._theta

    @property
    def velocity(self):
        '''
        Returns (vx, vy, omega): vx (lateral mm/s), vy (longitudinal mm/s), omega (rad/s).
        '''
        return self._vx, self._vy, self._omega

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def add_callback(self, callback):
        '''
        Adds a callback to those triggered by clock ticks when the robot is moving.
        '''
        if not callable(callback):
            raise Exception('callback argument is not a function.')
        if callback:
            if callback in self.__callbacks:
                raise Exception('callback already exists.')
            self._log.info('added callback: {}.{}()'.format(Util.get_class_name_of_method(callback), callback.__name__))
            self.__callbacks.append(callback)
        else:
            raise TypeError('null callback argument')

    def remove_callback(self, callback):
        '''
        Removes a callback from the internal list.
        '''
        if callback:
            if callback in self.__callbacks:
                self.__callbacks.remove(callback)
        else:
            raise TypeError('null callback argument')

    def _callback_method(self):
        '''
        The internal method called upon each tick, when the robot is moving.
        This executes any extant callbacks.
        '''
        if self.enabled:
            for callback in self.__callbacks:
                callback()

    def set_pose(self, x, y, theta, use_radians=True):
        '''
        Set the robot's pose to a specific position and heading.
        Used for initialisation or correcting odometry from an external fix.

        :param x:            lateral position in mm
        :param y:            longitudinal position in mm
        :param theta:        heading in radians (default)
        :param use_radians:  if False use degrees; default True
        '''
        self._x = x
        self._y = y
        self._theta = theta if use_radians else math.radians(theta)
        self._log.info('pose set to: x={:.2f}mm, y={:.2f}mm, theta={:.2f}rad ({:.1f}°)'.format(
            x, y, theta, degrees(theta)))

    def steps_to_mm(self, steps):
        return steps / self._steps_per_mm

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
        Call at regular intervals with latest step counts for all four motors
        and time (seconds).

        This triggers any callbacks at the end of each cycle if there is
        any movement.

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
            # convert to per-wheel distances in mm
            d_pfwd = ds_pfwd * self._step_mm
            d_sfwd = ds_sfwd * self._step_mm
            d_paft = ds_paft * self._step_mm
            d_saft = ds_saft * self._step_mm
            # mecanum chassis kinematics - velocities in ROBOT BODY FRAME
            vx = (d_pfwd - d_sfwd - d_paft + d_saft) / 4.0 / dt   # lateral (starboard+)
            vy = (d_pfwd + d_sfwd + d_paft + d_saft) / 4.0 / dt   # longitudinal (forward+)
            omega = (d_pfwd - d_sfwd + d_paft - d_saft) / (4.0 * ((self._wheel_base_mm + self._wheel_track_mm) / 2.0)) / dt
            # store body frame velocities
            self._vx = vx
            self._vy = vy
            self._omega = omega
            # integrate pose: transform body frame velocities to odometry frame
            cos_t = cos(self._theta)
            sin_t = sin(self._theta)
            # transform body velocity to odometry frame displacement
            dx = (vx * cos_t + vy * sin_t) * dt
            dy = (-vx * sin_t + vy * cos_t) * dt
            dtheta = omega * dt
            self._x += dx
            self._y += dy
            self._theta += dtheta
            # normalize theta to [-π, π]
            while self._theta > π:
                self._theta -= 2 * π
            while self._theta < -π:
                self._theta += 2 * π
        # save per-tick state
        self._last_steps = dict(step_counts)
        self._last_time = timestamp
        if self.is_moving():
            self._callback_method()

    def print_info(self):
        '''
        Prints the current velocity and pose.
        '''
        self._log.info(Style.DIM + 'velocity: ({:.2f}, {:.2f}, {:.2f});\tpose: ({:.2f}mm, {:.2f}mm, {:.2f}rad)'.format(
                self._vx, self._vy, self._omega, self._x, self._y, self._theta))

    def is_moving(self):
        '''
        Returns True if any of (vx, vy, omega) is non-zero using a deadband.
        '''
        if not isclose(self._vy, 0.0, abs_tol=1e-2):
            return True
        if not isclose(self._vx, 0.0, abs_tol=1e-2):
            return True
        if not isclose(self._omega, 0.0, abs_tol=1e-2):
            return True
        return False

    def enable(self):
        if not self.enabled:
            super().enable()
            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def disable(self):
        if self.enabled:
            super().disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

    def close(self):
        if not self.closed:
            super().close()
            self._log.info('closed.')
        else:
            self._log.warning('already closed.')

#EOF
