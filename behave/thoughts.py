#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-05-19
# modified: 2025-11-09
#

import itertools
import asyncio
from math import isclose
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.event import Event, Group
from behave.behaviour import Behaviour
from hardware.odometer import Odometer

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Thoughts(Behaviour):
    NAME = 'thoughts'
    _LISTENER_LOOP_NAME = '__thoughts_listener_loop'
    '''
    Monitors robot activity and plays sounds ("random thoughts") via the
    TinyFX at roughly a frequency corresponding to the message traffic
    and motor speed, monitored via the Odometer.

    :param config:          the application configuration
    :param message_bus:     the asynchronous message bus
    :param level:           the optional log level
    '''
    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        if message_bus is None:
            raise ValueError('no message bus argument.')
        Behaviour.__init__(self, Thoughts.NAME, config, message_bus, message_factory,
                          suppressed=True, enabled=False, level=level)
        # idle configuration
        _idle_cfg = config['kros']['behaviour']['idle']
        self._idle_threshold_sec = _idle_cfg.get('idle_threshold_sec')
        self._loop_freq_hz = _idle_cfg.get('loop_freq_hz', 1)
        self._idle_loop_delay_sec = 1.0 / self._loop_freq_hz
        self._log.info('idle threshold: {:d}s; loop freq: {:d}Hz'.format(
            self._idle_threshold_sec, self._loop_freq_hz))
        # thoughts configuration
        _cfg = config['kros']['behaviour']['thoughts']
        self._verbose  = _cfg.get('verbose', False)
        self._priority = _cfg.get('priority', 0.2)
        # TBD
        # components
        _registry = Component.get_registry()
        self._odometer = _registry.get(Odometer.NAME)
        if self._odometer is None:
            self._log.info('odometer not available; relying on message activity alone.')
        else:
            self._log.info('odometer available; will monitor for movement.')
        # state tracking
        self._counter = itertools.count()
        self._last_activity_time = None # datetime of last message
        # subscribe to all non-IDLE events to detect activity
        self._thoughts_task = None
        self._loop_running = False
        self.add_events([member for member in Group
                        if member not in (Group.NONE, Group.IDLE, Group.OTHER)])
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def name(self):
        return Thoughts.NAME

    @property
    def using_dynamic_priority(self):
        return False

    @property
    def priority(self):
        '''
        Returns the current priority for this behaviour.
        Subclasses can override to provide dynamic priority.
        '''
        return self._priority

    @property
    def elapsed_seconds(self):
        '''
        Return the number of elapsed seconds since the last activity.
        '''
        if self._last_activity_time is None:
            return 0.0
        return (dt.now() - self._last_activity_time).total_seconds()

    def enable(self):
        if self.enabled:
            self._log.debug('already enabled.')
            return
        self._log.info('enabling random thoughts…')
        Behaviour.enable(self)
        # initialize activity timer
        self._last_activity_time = dt.now()
        self._last_idle_publish_time = None
        # create async listener loop task
        if self._message_bus.get_task_by_name(Thoughts._LISTENER_LOOP_NAME):
            self._log.warning('loop task already exists.')
        else:
            self._log.info('creating loop task…')
            self._loop_running = True
            self._thoughts_task = self._message_bus.loop.create_task(
                self._thoughts_listener_loop(lambda: self.enabled),
                name=Thoughts._LISTENER_LOOP_NAME
            )
        self._log.info('enabled.')

    async def _thoughts_listener_loop(self, f_is_enabled):
        '''
        Main loop: monitors time since last activity.

        Activity is detected from:
        - Motor movement (checked every iteration if motor controller available)
        - Message bus traffic (via process_message())
        '''
        self._log.info('loop started (threshold: {:d}s)'.format(self._idle_threshold_sec))
        try:
            while f_is_enabled():
                self._log.info(Fore.BLUE + '🐟 loop')
                _count = next(self._counter)
                # check motor controller first - movement = activity
                if self._odometer:
                    vx, vy, omega = self._odometer.get_velocity()
                    activity = 0.0
                    if isclose(vx, 0.0, abs_tol=0.001) \
                            and isclose(vy, 0.0, abs_tol=0.001) \
                            and isclose(omega, 0.0, abs_tol=0.001):
                        self._log.info(Fore.BLUE + Style.DIM + '🐟 [{:05d}] motors idle'.format(_count))
                    else:
                        activity = abs(vx) + abs(vy) + abs(omega)
                        self._log.info(Fore.BLUE + '🐟 [{:05d}] motors active; level: {}'.format(_count, activity))
                    self._last_activity_time = dt.now()
                    self._last_idle_publish_time = None
                    await asyncio.sleep(self._idle_loop_delay_sec)
                    continue
                if self.suppressed:
#                   if _count % 10 == 0:
                    self._log.info(Fore.BLUE + '🐟 [{:05d}] suppressed.'.format(_count))
                    await asyncio.sleep(self._idle_loop_delay_sec)
                    continue
                # calculate elapsed time since last activity
                elapsed_sec = self.elapsed_seconds
                if elapsed_sec >= self._idle_threshold_sec:
                    # robot is idle
                    self._log.info(Fore.BLUE + Style.DIM + '🐟 [{:05d}] idle ({:.1f}s)'.format(_count, elapsed_sec))
                else:
                    # robot is active
                    self._log.info(Fore.BLUE + '🐟 [{:05d}] active ({:.1f}s since last activity)'.format(_count, elapsed_sec))
                await asyncio.sleep(self._idle_loop_delay_sec)

        except asyncio.CancelledError:
            self._log.info('idle listener loop cancelled.')
            raise # important: re-raise so task completes
        finally:
            self._log.info('idle listener loop complete.')

    async def process_message(self, message):
        '''
        Process non-IDLE messages.
        '''
        if message.gcd:
            raise GarbageCollectedError('cannot process message: garbage collected.')
        _event = message.event
        if _event.group == Group.IDLE:
            # ignore IDLE messages
            pass
        else:
            # any non-IDLE message indicates activity
            was_idle = self.elapsed_seconds >= self._idle_threshold_sec
            # reset activity timer
            self._last_activity_time = dt.now()
            self._last_idle_publish_time = None  # reset publish tracker
        await Subscriber.process_message(self, message)

    def execute(self, message):
        '''
        Unused by Thoughts.
        '''
        pass

    def disable(self):
        if not self.enabled:
            self._log.debug('already disabled.')
            return
        self._log.info('disabling idle…')
        # cancel the async task before calling Behaviour.disable()
        if self._thoughts_task and not self._thoughts_task.done():
            self._log.debug('cancelling idle listener loop task…')
            self._thoughts_task.cancel()
            # don't wait for it - just cancel and move on
        self._loop_running = False
        Behaviour.disable(self)
        self._log.info('disabled.')

    def close(self):
        '''
        Permanently close the behaviour.
        '''
        if not self.closed:
            self._log.info('closing idle…')
            self.disable()  # will cancel task
            Behaviour.close(self)
            self._log.info('closed.')

#EOF
