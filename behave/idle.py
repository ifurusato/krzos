#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2020-05-19
# modified: 2025-11-17

import itertools
import asyncio
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.event import Event, Group
from behave.behaviour import Behaviour
from core.publisher import Publisher
from hardware.eyeballs_monitor import EyeballsMonitor
from hardware.eyeball import Eyeball
from hardware.motor_controller import MotorController

class Idle(Behaviour, Publisher):
    NAME = 'idle'
    _LISTENER_LOOP_NAME = '__idle_listener_loop'
    '''
    Monitors robot activity and publishes IDLE events when the robot
    has been inactive for a threshold period.

    Activity detection sources:
    - MessageBus traffic (any non-IDLE event)
    - Motor movement (via MotorController.is_stopped property)

    This behavior acts as a persistent "nudge" mechanism: if the robot sits
    idle (no sensor activity, no movement), Idle publishes an Event.IDLE
    message every threshold period (e.g., every 20 seconds) until activity
    resumes. This triggers BehaviourManager to release suppressed behaviors,
    creating conditions where movement becomes possible.

    Idle does not guarantee robot movement - it only signals inactivity and
    requests behavior activation. Movement depends on:

      - Roam being enabled/released
      - Speed control (digital pot) being non-zero
      - Robot not being physically stuck
      - Clear path forward

    IDLE message payload: elapsed time in seconds since last activity (float)

    When idle state is detected, sets EyeballsMonitor to SLEEPY expression.
    When activity resumes, clears EyeballsMonitor override to resume motor
    direction display.

    :param config:          the application configuration
    :param message_bus:     the asynchronous message bus
    :param message_factory: the factory for messages
    :param level:           the optional log level
    '''
    def __init__(self, config, message_bus=None, message_factory=None, level=Level.INFO):
        if message_bus is None:
            raise ValueError('no message bus argument.')
        if message_factory is None:
            raise ValueError('no message factory argument.')
        Behaviour.__init__(self, Idle.NAME, config, message_bus, message_factory,
                          suppressed=True, enabled=False, level=level)
        Publisher.__init__(self, self._log, config, message_bus, message_factory,
                          suppressed=True, enabled=False, level=level)
        # configuration
        _cfg = config['kros']['behaviour']['idle']
        self._idle_threshold_sec = _cfg.get('idle_threshold_sec')
        self._loop_freq_hz = _cfg.get('loop_freq_hz', 1)
        self._idle_loop_delay_sec = 1.0 / self._loop_freq_hz
        self._log.info('idle threshold: {:d}s; loop freq: {:d}Hz'.format(
            self._idle_threshold_sec, self._loop_freq_hz))
        # components
        _component_registry = Component.get_registry()
        self._eyeballs_monitor = _component_registry.get(EyeballsMonitor.NAME)
        if self._eyeballs_monitor is None:
            self._log.warning('eyeballs monitor not available.')
        self._motor_controller = _component_registry.get(MotorController.NAME)
        if self._motor_controller is None:
            self._log.info('motor controller not available; relying on message activity alone.')
        else:
            self._log.info('motor controller available; will monitor for movement.')
        # state tracking
        self._counter = itertools.count()
        self._last_activity_time = None      # datetime of last non-IDLE message
        self._last_idle_publish_time = None  # datetime of last IDLE message published
        self._idle_loop_running = False
        # subscribe to all non-IDLE events to detect activity
        self.add_events([member for member in Group
                        if member not in (Group.NONE, Group.IDLE, Group.OTHER)])
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def using_dynamic_priority(self):
        return False

    @property
    def elapsed_seconds(self):
        '''
        Return the number of elapsed seconds since the last activity.
        '''
        if self._last_activity_time is None:
            return 0.0
        return (dt.now() - self._last_activity_time).total_seconds()

    def enable(self):
        if not self.enabled:
            self._log.debug('enabling idle…')
            super().enable()
            # initialize activity timer
            self._last_activity_time = dt.now()
            self._last_idle_publish_time = None
            # create async listener loop task
            if self._message_bus.get_task_by_name(Idle._LISTENER_LOOP_NAME):
                self._log.warning('idle listener loop task already exists.')
            else:
                self._log.info('creating idle listener loop task…')
                self._idle_loop_running = True
                self._idle_task = self._message_bus.loop.create_task(
                    self._idle_listener_loop(lambda: self.enabled),
                    name=Idle._LISTENER_LOOP_NAME
                )
            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def reset_activity_timer(self):
        '''
        Reset the activity timer to now, preventing IDLE messages from being published
        until the threshold period has elapsed again.
        '''
        self._last_activity_time = dt.now()
        self._last_idle_publish_time = None
        if self._eyeballs_monitor:
            self._eyeballs_monitor.clear_eyeballs()
        self._log.debug('activity timer reset by external call.')

    async def _idle_listener_loop(self, f_is_enabled):
        '''
        Main loop: monitors time since last activity and publishes IDLE events
        when threshold is exceeded. Continues publishing every threshold period
        until activity resumes.

        Activity is detected from:
        - Motor movement (checked every iteration if motor controller available)
        - Message bus traffic (via process_message())
        '''
        self._log.info('idle listener loop started (threshold: {:d}s)'.format(self._idle_threshold_sec))
        try:
            while f_is_enabled():
                _count = next(self._counter)
                # check motor controller first - movement = activity
                if self._motor_controller and not self._motor_controller.is_stopped:
                    self._last_activity_time = dt.now()
                    self._last_idle_publish_time = None
                    # clear eyeballs if we were idle
                    if self._eyeballs_monitor and self._last_idle_publish_time is not None:
                        self._eyeballs_monitor.clear_eyeballs()
                    if _count % 20 == 0:
                        self._log.debug('[{:05d}] motors active - not idle'.format(_count))
                    await asyncio.sleep(self._idle_loop_delay_sec)
                    continue

                # Check suppression
                if self.suppressed:
                    if _count % 10 == 0:
                        self._log.debug('[{:05d}] idle suppressed.'.format(_count))
                    await asyncio.sleep(self._idle_loop_delay_sec)
                    continue

                # Calculate elapsed time since last activity
                elapsed_sec = self.elapsed_seconds
                if elapsed_sec >= self._idle_threshold_sec:
                    # robot is idle - check if we should publish
                    if self._should_publish_idle():
                        await self._publish_idle(elapsed_sec)
                    elif _count % 10 == 0:
                        self._log.debug('[{:05d}] idle ({:.1f}s)'.format(_count, elapsed_sec))
                else:
                    # robot is active
                    if _count % 20 == 0:
                        self._log.debug('[{:05d}] active ({:.1f}s since last activity)'.format(
                            _count, elapsed_sec))

                await asyncio.sleep(self._idle_loop_delay_sec)

        except asyncio.CancelledError:
            self._log.info('idle listener loop cancelled.')
            raise # important: re-raise so task completes
        finally:
            self._log.info('idle listener loop complete.')

    def _should_publish_idle(self):
        '''
        Return True if we should publish an IDLE message.

        Publish on first idle detection, then every threshold period while
        idle condition persists.
        '''
        if self._last_idle_publish_time is None:
            return True  # first time crossing threshold
        elapsed_since_publish = (dt.now() - self._last_idle_publish_time).total_seconds()
        return elapsed_since_publish >= self._idle_threshold_sec

    async def _publish_idle(self, elapsed_sec):
        '''
        Publish IDLE message with elapsed time as payload.
        Sets eyeballs to SLEEPY on first publish.
        '''
        self._log.info('publishing IDLE message ({:.1f}s elapsed)'.format(elapsed_sec))
        # set eyeballs to sleepy (only on first idle detection)
        if self._last_idle_publish_time is None and self._eyeballs_monitor:
            self._eyeballs_monitor.set_eyeballs(Eyeball.SLEEPY)
            self._log.debug('eyeballs set to SLEEPY')
        # publish IDLE message with elapsed time as payload
        _message = self._message_factory.create_message(Event.IDLE, elapsed_sec)
        await self.publish(_message)
        # track when we published
        self._last_idle_publish_time = dt.now()

    async def process_message(self, message):
        '''
        Process non-IDLE messages: any activity resets the idle timer.
        If transitioning from idle to active, clear eyeballs override.
        '''
        if message.gcd:
            raise GarbageCollectedError('cannot process message: garbage collected.')
        _event = message.event
        if _event.group == Group.IDLE:
            # ignore our own IDLE messages (prevent feedback loop)
            pass
        else:
            # any non-IDLE message indicates activity
            was_idle = self.elapsed_seconds >= self._idle_threshold_sec
            # reset activity timer
            self._last_activity_time = dt.now()
            self._last_idle_publish_time = None  # reset publish tracker
            # if transitioning from idle to active, clear eyeballs
            if was_idle:
                if self._eyeballs_monitor:
                    self._eyeballs_monitor.clear_eyeballs()
                    self._log.debug('eyeballs override cleared')
                self._log.info('activity resumed after {:.1f}s idle'.format(
                    self.elapsed_seconds))
        await Subscriber.process_message(self, message)

    def execute(self, message):
        '''
        Behaviour.execute() - not used by Idle.

        Idle operates via process_message() as a Subscriber and publishes
        directly via Publisher.publish(), bypassing the execute pattern.
        '''
        pass

    def disable(self):
        if self.enabled:
            self._log.debug('disabling idle…')
            # cancel the async task before calling Behaviour.disable()
            if self._idle_task and not self._idle_task.done():
                self._log.debug('cancelling idle listener loop task…')
                self._idle_task.cancel()
                # don't wait for it - just cancel and move on
            # clear eyeballs if currently showing SLEEPY
            if self._eyeballs_monitor and self._last_idle_publish_time is not None:
                self._eyeballs_monitor.clear_eyeballs()
            self._idle_loop_running = False
            super().disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

    def close(self):
        '''
        Permanently close the Idle behaviour.
        '''
        if not self.closed:
            self._log.debug('closing idle…')
            super().close()
            self._log.info('closed.')
        else:
            self._log.warning('already closed.')

#EOF
