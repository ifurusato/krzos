#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-10
# modified: 2025-11-10

import time
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.component import Component

class SlewLimiter(Component):
    '''
    A time-based slew limiter that limits the rate of change of intent vectors
    (vx, vy, omega) to smooth the robot's motion. Operates at the MotorController
    level to coordinate all four motors' acceleration/deceleration together.

    Rate limits are specified per second and scaled by actual elapsed time between
    calls, making this robust to variable motor loop timing.

    :param config:       application configuration
    :param level:        the logging Level
    '''
    def __init__(self, config, level=Level.INFO):
        self._log = Logger('slew-limiter', level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)

        # read configuration - rates are per second
        _cfg = config['kros'].get('motor_controller').get('slew_limiter')
        self._max_vx_rate    = _cfg.get('max_vx_rate')      # lateral motion rate limit (per second)
        self._max_vy_rate    = _cfg.get('max_vy_rate')      # forward motion rate limit (per second)
        self._max_omega_rate = _cfg.get('max_omega_rate')   # rotation rate limit (per second)
        self._log.info('rate limits (per second): vx={:.3f}, vy={:.3f}, omega={:.3f}'.format(
            self._max_vx_rate, self._max_vy_rate, self._max_omega_rate))
        # state tracking
        self._last_intent = (0.0, 0.0, 0.0)
        self._last_time   = time.perf_counter()
        self._fixed_rate  = True
        self._verbose     = False
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def limit(self, target_intent):
        if self._fixed_rate:
            return self._fixed_limit(target_intent)
        else:
            return self._variable_limit(target_intent)

    def _fixed_limit(self, target_intent):
        if not self.enabled or self.suppressed:
            return target_intent
        curr_vx, curr_vy, curr_omega = self._last_intent
        tgt_vx, tgt_vy, tgt_omega = target_intent
        # use fixed per-call limits (assume 20Hz operation)
        NOMINAL_PERIOD_SEC = 0.05
        max_vx_change = self._max_vx_rate * NOMINAL_PERIOD_SEC
        max_vy_change = self._max_vy_rate * NOMINAL_PERIOD_SEC
        max_omega_change = self._max_omega_rate * NOMINAL_PERIOD_SEC
        limited_vx = self._clamp_change(curr_vx, tgt_vx, max_vx_change)
        limited_vy = self._clamp_change(curr_vy, tgt_vy, max_vy_change)
        limited_omega = self._clamp_change(curr_omega, tgt_omega, max_omega_change)
        limited_intent = (limited_vx, limited_vy, limited_omega)
        self._last_intent = limited_intent
        self._last_time = time.perf_counter() # track for diagnostics
        return limited_intent

    def _variable_limit(self, target_intent):
        '''
        Limit the rate of change to target_intent based on elapsed time.

        Args:
            target_intent: tuple (vx, vy, omega) - desired intent vector

        Returns:
            tuple (vx, vy, omega) - rate-limited intent vector
        '''
        if not self.enabled:
            # when disabled, pass through target unchanged and don't update state
            return target_intent
        if self.suppressed:
            return target_intent
        # measure elapsed time since last call
        now = time.perf_counter()
        elapsed_sec = now - self._last_time
        # guard against division by zero or negative time (shouldn't happen but...)
        if elapsed_sec <= 0.0:
            elapsed_sec = 0.001  # minimum 1ms
        curr_vx, curr_vy, curr_omega = self._last_intent
        tgt_vx, tgt_vy, tgt_omega = target_intent
        # calculate maximum allowed change based on elapsed time and rate limits
        max_vx_change = self._max_vx_rate * elapsed_sec
        max_vy_change = self._max_vy_rate * elapsed_sec
        max_omega_change = self._max_omega_rate * elapsed_sec
        # clamp each component independently
        limited_vx = self._clamp_change(curr_vx, tgt_vx, max_vx_change)
        limited_vy = self._clamp_change(curr_vy, tgt_vy, max_vy_change)
        limited_omega = self._clamp_change(curr_omega, tgt_omega, max_omega_change)
        limited_intent = (limited_vx, limited_vy, limited_omega)

        # DIAGNOSTIC:
        if limited_intent != target_intent:
            self._log.info(Fore.BLUE + 'slewed.')

        # update internal state
        self._last_intent = limited_intent
        self._last_time = now
        if self._verbose:
            # log if any component was limited
            if limited_intent != target_intent:
                self._log.info('slew limited (Δt={:.1f}ms): target=({:.3f}, {:.3f}, {:.3f}) → limited=({:.3f}, {:.3f}, {:.3f})'.format(
                    elapsed_sec * 1000.0, tgt_vx, tgt_vy, tgt_omega, limited_vx, limited_vy, limited_omega))
        return limited_intent

    def _clamp_change(self, current, target, max_change):
        '''
        Clamp the change from current to target by max_change.

        Args:
            current:    current value
            target:     desired value
            max_change: maximum allowed change (based on elapsed time)

        Returns:
            clamped value
        '''
        change = target - current
        if abs(change) <= max_change:
            # change is within limit, return target
            return target
        # clamp the change
        if change > 0:
            return current + max_change
        else:
            return current - max_change

    def reset(self):
        '''
        Reset internal state to zero. Useful when stopping or changing modes.
        '''
        self._last_intent = (0.0, 0.0, 0.0)
        self._last_time = time.perf_counter()
        self._log.debug('reset to zero state.')

    def enable(self):
        if not self.enabled:
            super().enable()
            # reset timing on enable to avoid large initial elapsed time
            self._last_time = time.perf_counter()
            self._log.info('enabled.')

    def disable(self):
        if self.enabled:
            super().disable()
            self._log.info('disabled.')

#EOF
