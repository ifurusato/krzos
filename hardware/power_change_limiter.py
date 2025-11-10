#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved.
#
# author:   Murray Altheim
# created:  2025-11-10
# modified: 2025-11-10

import time
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.component import Component

class PowerChangeLimiter(Component):
    '''
    A time-based power change limiter that prevents rapid changes in motor power
    to protect gearboxes from mechanical shock. Operates at the Motor level as
    the final safety check before power reaches the motor controller hardware.
    
    Rate limits are specified per second and scaled by actual elapsed time.

    Start with 2.0/sec (0.10/tick @ 20Hz)
    If still too abrupt, decrease to 1.5/sec (0.075/tick)
    If too sluggish, increase to 3.0/sec (0.15/tick)
    
    :param config:       application configuration
    :param orientation:  motor orientation (for logging)
    :param level:        the logging Level
    '''
    def __init__(self, config, orientation, level=Level.INFO):
        self._log = Logger('power-limiter:{}'.format(orientation.label), level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        # read configuration
        _cfg = config['kros'].get('motor').get('power_change_limiter')
        self._max_power_change_rate = _cfg.get('max_power_change_rate')  # per second
        self._log.info('power change rate limit: {:.3f}/sec'.format(self._max_power_change_rate))
        # state tracking
        self._last_power = 0.0
        self._last_time  = time.perf_counter()
        self._verbose    = True
        self._log.info('ready.')

    def limit(self, current_power, target_power):
        '''
        Limit the rate of change from current_power to target_power.
        
        Args:
            current_power: current motor power (-1.0 to 1.0)
            target_power:  desired motor power (-1.0 to 1.0)
        
        Returns:
            float - rate-limited motor power
        '''
        if not self.enabled:
            return target_power
        if self.suppressed:
            return target_power
        # measure elapsed time
        now = time.perf_counter()
        elapsed_sec = now - self._last_time
        if elapsed_sec <= 0.0:
            elapsed_sec = 0.001  # minimum 1ms
        # calculate maximum allowed change
        max_change = self._max_power_change_rate * elapsed_sec
        # clamp the change
        power_change = target_power - current_power
        if abs(power_change) <= max_change:
            limited_power = target_power
        else:
            if power_change > 0:
                limited_power = current_power + max_change
            else:
                limited_power = current_power - max_change
        # update state
        self._last_power = limited_power
        self._last_time = now
        if self._verbose:
            # log if limited
            if abs(limited_power - target_power) > 0.001:
                self._log.debug('power limited (Δt={:.1f}ms): target={:.3f} → limited={:.3f}'.format(
                    elapsed_sec * 1000.0, target_power, limited_power))
        return limited_power

    def reset(self):
        '''
        Reset to zero state.
        '''
        self._last_power = 0.0
        self._last_time = time.perf_counter()
        self._log.debug('reset.')

    def enable(self):
        if not self.enabled:
            Component.enable(self)
            self._last_time = time.perf_counter()
            self._log.info('enabled.')

    def disable(self):
        if self.enabled:
            Component.disable(self)
            self._log.info('disabled.')

#EOF
