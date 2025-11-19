#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-05-19
# modified: 2025-11-18

import itertools
import asyncio
import time
from math import isclose
from random import choice, expovariate, randint
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.event import Event, Group
from core.rate_limited import rate_limited
from behave.behaviour import Behaviour
from hardware.odometer import Odometer
from hardware.tinyfx_controller import TinyFxController

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
        self._verbose        = _cfg.get('verbose', False)
        self._priority       = _cfg.get('priority', 0.2)
        self._max_activity   = _cfg.get('max_activity', 4.0)
        self._activity_scale = _cfg.get('activity_scale', 1.0)
        self._jitter_factor  = _cfg.get('jitter_factor', 2.5)
        self._max_frequency  = _cfg.get('max_frequency', 0.8)
        self._rate_delay_ms  = _cfg.get('rate_limit_ms', 1500)
        self._bored_sound    = _cfg.get('bored_sound')
        self._sleeping_sound = _cfg.get('sleeping_sound')
        self._sounds         = _cfg.get('sounds', [])
        if not self._sounds:
            raise ValueError('no sounds configured.')
        self._log.info('configured {} sounds; max activity: {:4.2f}; activity scale: {:4.2f}; jitter: {:4.2f}; max freq: {:4.2f}Hz; rate limit: {:d}ms'.format(
            len(self._sounds), self._max_activity, self._activity_scale, self._jitter_factor, self._max_frequency, self._rate_delay_ms))
        # components
        _registry = Component.get_registry()
        self._odometer = _registry.get(Odometer.NAME)
        if self._odometer is None:
            self._log.info('odometer not available; relying on message activity alone.')
        else:
            self._log.info('odometer available; will monitor for movement.')
        self._tinyfx_controller = TinyFxController(config, level=level)
        self._log.info('TinyFX controller instantiated.')
        # state tracking
        self._counter = itertools.count()
        self._last_activity_time = None
        self._last_called = 0
        self._last_name = None
        self._next_play_time = 0.0
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
        # enable TinyFX controller
        self._tinyfx_controller.enable()
        # initialize activity timer
        self._last_activity_time = dt.now()
        self._last_idle_publish_time = None
        self._next_play_time = time.monotonic()
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
        Main loop: monitors activity and plays sounds organically.
        Goes from boredom to sleep.

        Activity is detected from:
        - Motor movement (checked every iteration if odometer available)
        - Message bus traffic (via process_message())
        '''
        self._log.info('loop started (threshold: {:d}s)'.format(self._idle_threshold_sec))
        try:
            self._idle_count  = 0
            self._bored       = False
            self._sleeping    = False
            self._bored_count = randint(10, 20)
            self._sleep_count = randint(21, 31)
            self._snore_rate  = randint(8, 11)
            while f_is_enabled():
                _count = next(self._counter)
                # check odometer for movement activity
                if self._odometer:
                    vx, vy, omega = self._odometer.get_velocity()
                    _raw_activity = abs(vx) + abs(vy) + abs(omega)
                    if isclose(_raw_activity, 0.0, abs_tol=0.001):
                        _style = Style.DIM
                        _raw_activity = 0.0
                        self._idle_count += 1
                    else:
                        _style = Style.NORMAL
                        self._last_activity_time = dt.now()
                        self._last_idle_publish_time = None
                        self._sleeping   = False
                        self._bored      = False
                        self._idle_count = 0
                    # normalize and scale activity
                    _normalized = _raw_activity / self._max_activity
                    _activity = _normalized * self._activity_scale
                    _activity = min(_activity, 1.0)
                    if self._verbose:
                        _idle_style     = Style.NORMAL if ( not self._bored and not self._sleeping ) else Style.DIM
                        _bored_style    = Style.NORMAL if ( self._bored and not self._sleeping ) else Style.DIM
                        _sleeping_style = Style.NORMAL if self._sleeping else Style.DIM
                        self._log.info(Fore.BLUE + _style + '[{:05d}] activity: {:4.2f} (raw: {:4.2f});'.format(
                               _count, _activity, _raw_activity)  
                               + Style.NORMAL
                               + Fore.WHITE   + _idle_style     + ' idle: {};'.format(self._idle_count)
                               + Fore.YELLOW  + _bored_style    + ' bored: {};'.format(self._bored_count)
                               + Fore.BLUE    + _sleeping_style + ' sleep: {};'.format(self._sleep_count)
                               + Fore.BLACK   + ' snore: {}'.format(self._snore_rate)
                        )

                    if self._idle_count == 0: # activity ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                        self._maybe_play_sound(_activity)

                    elif self._sleeping: # sleeping ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                        if _count % self._snore_rate == 0: # every n seconds, snore
                            self._log.info(Fore.BLUE + Style.DIM + 'still sleeping…')
                            self._play_sound('snore')
                            self._snore_rate = randint(8, 11) # change snore rate

                    elif self._idle_count % self._sleep_count == 0: # start sleeping ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                        self._log.info(Fore.BLUE + 'sleeping…')
                        self._sleeping    = True
                        self._sleep_count = randint(21, 31) # reset to different threshold
#                       self._log.info(Fore.BLUE + 'sleeping at: {}.'.format(self._sleep_count))
                        self._play_sound(self._sleeping_sound)

                    elif not self._bored and self._idle_count % self._bored_count == 0: # bored ┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                        self._log.info(Fore.BLUE + 'bored…')
                        self._bored = True
                        self._bored_count = randint(11, 17)
#                       self._log.info(Fore.BLUE 'bored at: {}.'.format(self._bored_count))
                        self._play_sound(self._bored_sound)

                    else:
#                       self._log.info(Fore.BLUE + 'something else…')
                        pass

                if self.suppressed:
                    if self._verbose:
                        self._log.info(Fore.BLUE + '[{:05d}] suppressed.'.format(_count))
                    await asyncio.sleep(self._idle_loop_delay_sec)
                    continue
                # calculate elapsed time since last activity
                elapsed_sec = self.elapsed_seconds
                if elapsed_sec >= self._idle_threshold_sec:
                    if self._verbose:
                        self._log.info(Fore.BLUE + Style.DIM + '[{:05d}] idle ({:.1f}s)'.format(_count, elapsed_sec))
                else:
                    if self._verbose:
                        self._log.info(Fore.BLUE + '[{:05d}] active ({:.1f}s since last activity)'.format(_count, elapsed_sec))
                await asyncio.sleep(self._idle_loop_delay_sec)

        except asyncio.CancelledError:
            self._log.info('thoughts listener loop cancelled.')
            raise
        finally:
            self._log.info('thoughts listener loop complete.')

    def _maybe_play_sound(self, activity):
        '''
        Determine if it's time to play a sound based on activity level.

        :param activity: normalized activity level (0.0-1.0)
        '''
        now = time.monotonic()
        frequency = self._max_frequency * activity
        if frequency > 0:
            if now >= self._next_play_time:
                name = choice([n for n in self._sounds if n != self._last_name])
                self._log.info(Style.DIM + 'calling play: ' + Fore.WHITE + Style.BRIGHT + "'{}'".format(name) + Style.RESET_ALL)
                self._play_sound(name)
                self._last_name = name
                interval = expovariate(frequency / self._jitter_factor)
                self._next_play_time = now + interval

    @rate_limited(3000)
    def _play_sound(self, name):
        '''
        Play the specified sound via TinyFX controller.
        Rate-limited to prevent sounds from overlapping.

        :param name: the sound name to play
        '''
        self._log.info('play: ' + Fore.WHITE + Style.BRIGHT + "'{}'".format(name) + Style.RESET_ALL)
        response = self._tinyfx_controller.send('play {}'.format(name))
        if self._verbose:
            self._log.info('response: {}'.format(response))

    async def process_message(self, message):
        '''
        Process non-IDLE messages.
        '''
        if message.gcd:
            raise GarbageCollectedError('cannot process message: garbage collected.')
        _event = message.event
        if _event.group == Group.IDLE:
            pass
        else:
            # any non-IDLE message indicates activity
            was_idle = self.elapsed_seconds >= self._idle_threshold_sec
            self._last_activity_time = dt.now()
            self._last_idle_publish_time = None
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
        self._log.info('disabling thoughts…')
        # cancel the async task before calling Behaviour.disable()
        if self._thoughts_task and not self._thoughts_task.done():
            self._log.debug('cancelling thoughts listener loop task…')
            self._thoughts_task.cancel()
        self._loop_running = False
        # disable TinyFX controller
        self._tinyfx_controller.disable()
        Behaviour.disable(self)
        self._log.info('disabled.')

    def close(self):
        '''
        Permanently close the behaviour.
        '''
        if not self.closed:
            self._log.info('closing thoughts…')
            self.disable()
            self._tinyfx_controller.close()
            Behaviour.close(self)
            self._log.info('closed.')

#EOF
