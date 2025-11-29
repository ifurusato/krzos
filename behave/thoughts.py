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
from random import choice, expovariate, randint, uniform
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.event import Event, Group
from core.orientation import Orientation
from core.rate_limited import rate_limited
from core.subscriber import Subscriber
from behave.behaviour import Behaviour
from hardware.odometer import Odometer
from hardware.eyeball import Eyeball
from hardware.eyeballs_monitor import EyeballsMonitor
from hardware.motor_controller import MotorController
from hardware.player import Player
from hardware.tinyfx_controller import TinyFxController

class Thoughts(Behaviour):
    NAME = 'thoughts'
    _LISTENER_LOOP_NAME = '__thoughts_listener_loop'
    '''
    Monitors robot activity and plays sounds ("random thoughts") via the TinyFX
    (using Player) at roughly a frequency corresponding to the message traffic
    and motor speed, monitored via the Odometer.

    Because this Behaviour is tasked with expression, it is also responsible
    for the robot's running lights.

    Gamepad controls:

        A:             unused
        B:             unused
        Y:             exit kros
        L1:            toggle running lights
        R1:            play sound
        R2:            toggle sound suppression
        D-Pad LEFT:    nudge left (from Nudge)
        D-Pad RIGHT:   nudge right (from Nudge)

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
        self._loop_delay_sec = 1.0 / self._loop_freq_hz
        self._log.info('idle threshold: {:d}s; loop freq: {:d}Hz'.format(
            self._idle_threshold_sec, self._loop_freq_hz))
        # thoughts configuration
        _cfg = config['kros']['behaviour']['thoughts']
        self._verbose           = _cfg.get('verbose', False)
        self._priority          = _cfg.get('priority', 0.2)
        self._max_activity      = _cfg.get('max_activity', 4.0)
        self._activity_scale    = _cfg.get('activity_scale', 1.0)
        self._jitter_factor     = _cfg.get('jitter_factor', 2.5)
        self._max_frequency     = _cfg.get('max_frequency', 0.8)
        self._rate_delay_ms     = _cfg.get('rate_limit_ms', 1500)
        self._bored_sound       = _cfg.get('bored_sound')
        self._sleeping_sound    = _cfg.get('sleeping_sound')
        self._active_sounds     = _cfg.get('active_sounds', [])
        self._enable_light_ctrl = _cfg.get('enable_light_ctrl')
        if not self._active_sounds:
            raise ValueError('no sounds configured.')
        self._log.info('configured {} sounds; max activity: {:4.2f}; activity scale: {:4.2f}; jitter: {:4.2f}; max freq: {:4.2f}Hz; rate limit: {:d}ms'.format(
            len(self._active_sounds), self._max_activity, self._activity_scale, self._jitter_factor, self._max_frequency, self._rate_delay_ms))
        self._poll_pir = False # not currently functional
        # components
        _component_registry = Component.get_registry()
        self._eyeballs_monitor = _component_registry.get(EyeballsMonitor.NAME)
        if self._eyeballs_monitor is None:
            self._log.warning('eyeballs monitor not available.')
        self._odometer = _component_registry.get(Odometer.NAME)
        if self._odometer is None:
            self._log.info('odometer not available; relying on message activity alone.')
        else:
            self._log.info('odometer available; will monitor for movement.')
        self._motor_controller = _component_registry.get(MotorController.NAME)
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
        # get tinyfx
        self._tinyfx = None
        # behavioural states
        self._idle_count       = 0
        self._snore_count      = 0
        self._sleep_count      = 0
        self._bored            = False
        self._sleeping         = False
        self._bored_limit      = randint(10, 16)  # how long before we get bored?
        self._sleep_limit      = randint(17, 23)  # how many cycles before falling asleep?
        self._deep_sleep_limit = randint( 5,  8)  # how many cycles asleep before falling into deep sleep?
        self._snore_limit      = randint( 7, 10)  # how many snores?
        self._snore_rate       = randint( 8, 11)  # how fast to snore?
        self._reverse_count    = 0
        self._reverse_limit    = 2                # how many cycles backwards before we notice?
        self._reversing        = False
        self._suppress_sounds  = False
        self._suppress_random_sounds = False
        self._interior_light_state  = False
        self._running_lights_state  = False
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

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
        self._mark_activity()
        self._next_play_time = time.monotonic()
        _component_registry = Component.get_registry()
        self._tinyfx = _component_registry.get(TinyFxController.NAME)
        self._set_running_lights(True)
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

    def _play_sound(self, name):
        if not self._suppress_sounds:
            Player.play(name)
        else:
            self._log.info(Style.DIM + "not playing sound '{}'".format(name))

    def _mark_activity(self):
        self._idle_count  = 0
        self._snore_count = 0
        self._sleep_count = 0
        self._last_activity_time = dt.now()
        self._last_idle_publish_time = None
        self._sleeping    = False
        self._bored       = False
        self._sleep_count = 0
        self._idle_count  = 0
        self._snore_count = 0
        self._reset_eyeballs()

    def _toggle_interior_light(self):
        self._set_interior_light(not self._interior_light_state)

    def _set_interior_light(self, enable):
        if self._enable_light_ctrl:
            self._log.info('set interior light: ' + Fore.GREEN +'{}'.format('on' if enable else 'off'))
            _saved = self._suppress_random_sounds
            self._suppress_random_sounds = True
            try:
                if self._tinyfx:
                    time.sleep(0.33)
                    self._tinyfx.light(Orientation.INT, enable)
                self._interior_light_state = enable
            finally:
                self._suppress_random_sounds = _saved
        else:
            self._log.info('light control disabled.')

    def _toggle_running_lights(self):
        self._set_running_lights(not self._running_lights_state)

    def _set_running_lights(self, enable):
        if self._enable_light_ctrl:
            self._log.info('set running lights: ' + Fore.GREEN +'{}'.format('on' if enable else 'off'))
            _saved = self._suppress_random_sounds
            self._suppress_random_sounds = True
            try:
                if self._tinyfx:
                    time.sleep(0.33)
                    self._tinyfx.running_lights(enable)
                self._running_lights_state = enable
                if self._motor_controller:
                    # we shut this off and don't turn it on again until disabled
                    self._motor_controller.set_show_battery(False)
            finally:
                self._suppress_random_sounds = _saved
        else:
            self._log.info('light control disabled.')

    def _reset_eyeballs(self):
        _eyeballs = self._eyeballs_monitor.get_eyeballs()
        if _eyeballs == Eyeball.NEUTRAL \
                or _eyeballs == Eyeball.BORED \
                or _eyeballs == Eyeball.SLEEPY \
                or _eyeballs == Eyeball.DEEP_SLEEP:
            self._eyeballs_monitor.clear_eyeballs()

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
            while f_is_enabled():
                _count = next(self._counter)
#               self._log.info("is '{}' released by toggle? {}".format(self.name, self.is_released_by_toggle()))
                if self.has_toggle_assignment():
                    if self.suppressed and self.is_released_by_toggle():
                        self._log.info(Fore.WHITE + Style.BRIGHT + 'releasing…')
                        self.release()
                    elif self.released and not self.is_released_by_toggle():
                        self._log.info(Fore.WHITE + Style.BRIGHT + 'suppressing…')
                        self.suppress()
                if self.suppressed:
                    if self._verbose:
                        self._log.info(Fore.BLUE + '[{:05d}] suppressed.'.format(_count))
                    await asyncio.sleep(self._loop_delay_sec)
                    continue
                # check odometer for movement activity
                if self._odometer:
                    vx, vy, omega = self._odometer.get_velocity()
                    _raw_activity = abs(vx) + abs(vy) + abs(omega)
                    if isclose(_raw_activity, 0.0, abs_tol=0.001):
                        _style = Style.DIM
                        _raw_activity = 0.0
                        self._idle_count += 1
                        self._reverse_count = 0
                        self._update_reverse_state()
                    else:
                        if vy < 0:
                            self._reverse_count += 1
                            self._update_reverse_state()
                        elif vy > 0:
                            self._reverse_count = 0
                            self._update_reverse_state()
                        # reset, there's been activity
                        _style = Style.NORMAL
                        self._mark_activity()
                    # normalize and scale activity
                    _normalized = _raw_activity / self._max_activity
                    _activity = _normalized * self._activity_scale
                    _activity = min(_activity, 1.0)
#                   if self._verbose:
                    _idle_style     = Style.NORMAL if ( not self._bored and not self._sleeping ) else Style.DIM
                    _bored_style    = Style.NORMAL if ( self._bored and not self._sleeping ) else Style.DIM
                    _sleeping_style = Style.NORMAL if self._sleeping else Style.DIM
                    self._log.info(Fore.BLUE + _style + '[{:05d}] activity: {:4.2f} (raw: {:4.2f});'.format(
                           _count, _activity, _raw_activity)
                           + Style.NORMAL
                           + Fore.WHITE   + _idle_style     + ' idle: {};'.format(self._idle_count)
                           + Fore.YELLOW  + _bored_style    + ' bored: {};'.format(self._bored_limit)
                           + Fore.BLUE    + _sleeping_style + ' sleep: {};'.format(self._sleep_limit)
                           + Fore.BLACK   + ' snore: {}'.format(self._snore_rate)
                    )
                    if self._idle_count == 0: # activity ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                        self._maybe_play_random_sound(_activity)
                        self._reset_eyeballs()

                    elif self._sleeping: # sleeping ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                        if _count % self._snore_rate == 0: # every n seconds, snore
                            self._log.info(Fore.BLUE + Style.DIM + 'still sleeping… [{}]'.format(_count))
                            self._snore_count += 1
                            if self._snore_count < self._snore_limit:
                                self._play_sound('snore')
                                self._snore_rate = randint(8, 11) # change snore rate
                            else:
                                if self._poll_pir: # check for human/cat activity
                                    self.pir()
                                elif self._sleep_count > self._deep_sleep_limit:
                                    self._eyeballs_monitor.set_eyeballs(Eyeball.DEEP_SLEEP)
                                    self._play_sound('quiet-breathing')
                                else:
                                    self._sleep_count += 1
                                    self._set_running_lights(False)
                                    time.sleep(0.1)
                                    self._play_sound('breathing')

                    elif self._idle_count % self._sleep_limit == 0: # start sleeping ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                        self._log.info(Fore.BLUE + 'sleeping…')
                        self._sleeping    = True
                        self._sleep_limit = randint(21, 31) # reset to different threshold
                        self._log.info(Fore.BLUE + 'sleeping at: {}.'.format(self._sleep_limit))
                        self._play_sound(self._sleeping_sound)
                        self._eyeballs_monitor.set_eyeballs(Eyeball.SLEEPY)

                    elif not self._bored and self._idle_count % self._bored_limit == 0: # bored ┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                        self._log.info(Fore.BLUE + 'bored…')
                        self._bored = True
                        self._bored_limit = randint(11, 17)
#                       self._log.info(Fore.BLUE 'bored at: {}.'.format(self._bored_limit))
                        self._play_sound(self._bored_sound)
                        self._eyeballs_monitor.set_eyeballs(Eyeball.BORED)

                    else:
#                       self._log.info('something else…')
                        pass
                # calculate elapsed time since last activity
                elapsed_sec = self.elapsed_seconds
                if elapsed_sec >= self._idle_threshold_sec:
                    if self._verbose:
                        self._log.info(Fore.BLUE + Style.DIM + '[{:05d}] idle ({:.1f}s)'.format(_count, elapsed_sec))
                else:
                    if self._verbose:
                        self._log.info(Fore.BLUE + '[{:05d}] active ({:.1f}s since last activity)'.format(_count, elapsed_sec))
                if self._sleep_count > 0:
                    # randomly longer
                    await asyncio.sleep(self._loop_delay_sec * uniform(1.2, 2.0))
                else:
                    await asyncio.sleep(self._loop_delay_sec)

        except asyncio.CancelledError:
            self._log.info('thoughts listener loop cancelled.')
            raise
        finally:
            self._log.info('f_enabled on finally? {}'.format(f_is_enabled()))
            self._log.info('self.enabled on finally? {}'.format(self.enabled))
            self._log.info('thoughts listener loop complete.')

    def _update_reverse_state(self):
        '''
        If the robot has been reversing for enough loop cycles, turn on the backup lights.
        '''
        # if we're potentially reversing, don't play random sounds
        self._suppress_random_sounds = self._reverse_count > 0
        reversing = self._reverse_count >= self._reverse_limit
        if reversing == self._reversing:
            return # no change of state
        elif reversing:
            if self._tinyfx:
                self._tinyfx.light(Orientation.AFT, True)
                time.sleep(0.1)
                self._play_sound('backing')
            else:
                self._log.warning('no tinyfx.')
            self._log.info(Fore.GREEN + 'reversing… ({})'.format(self._reverse_count))
        else:
            if self._tinyfx:
                self._tinyfx.light(Orientation.AFT, False)
            else:
                self._log.warning('no tinyfx.')
            self._log.info(Fore.GREEN + Style.DIM + 'no longer reversing… ({})'.format(self._reverse_count))
        self._reversing = reversing

    def _maybe_play_random_sound(self, activity):
        '''
        Determine if it's time to play a random sound based on activity level.

        :param activity: normalized activity level (0.0-1.0)
        '''
        if not self._suppress_random_sounds:
            now = time.monotonic()
            frequency = self._max_frequency * activity
            if frequency > 0:
                if now >= self._next_play_time:
                    name = choice([n for n in self._active_sounds if n != self._last_name])
                    self._log.info(Style.DIM + 'calling play: ' + Fore.WHITE + Style.BRIGHT + "'{}'".format(name) + Style.RESET_ALL)
                    self._play_sound(name)
                    self._last_name = name
                    interval = expovariate(frequency / self._jitter_factor)
                    self._next_play_time = now + interval

    async def process_message(self, message):
        '''
        Process non-IDLE messages.
        '''
        self._log.info(Style.DIM + 'is message already garbage collected? {}'.format(message.gcd))
        if message.gcd:
            raise GarbageCollectedError('cannot process message: garbage collected.')
        _event = message.event
#       if _event.group == Group.IDLE:
#           return
        if message.event.group is Group.GAMEPAD:
            self._mark_activity()
#           self.execute(message)
#       else:
#           self._mark_activity()
#           return
        await super().process_message(message)
#       await Subscriber.process_message(self, message)

    def execute(self, message):
        '''
        Nudge operates via process_message() as a Subscriber.
        '''
        self._log.info(Style.DIM + '🌼 execute message.')
        event = message.event
        match(event):
            case Event.A_BUTTON:
                self._log.info(Style.DIM + 'A_BUTTON.')
            case Event.B_BUTTON:
                self._log.info(Style.DIM + 'B_BUTTON.')
            case Event.X_BUTTON:
                self._log.info(Style.DIM + 'X_BUTTON.')
            case Event.Y_BUTTON:
                self._log.info(Style.DIM + 'Y_BUTTON. exit.')
            case Event.L1_BUTTON:
                self._log.info(Fore.MAGENTA + 'L1_BUTTON. toggle running lights')
                self._toggle_running_lights()
            case Event.L2_BUTTON:
                self._log.info(Style.DIM + 'L2_BUTTON. toggle interior light')
                self._toggle_interior_light()
            case Event.R1_BUTTON:
                self._log.info(Fore.MAGENTA + 'R1_BUTTON. play sound ugh')
                self._play_sound('ugh')
            case Event.R2_BUTTON:
                self._suppress_sounds = not self._suppress_sounds
                if self._suppress_sounds:
                    self._log.info(Fore.GREEN + 'R2_BUTTON. sounds are suppressed.')
                else:
                    self._log.info(Fore.GREEN + 'R2_BUTTON. sounds are not suppressed.')
            case Event.START_BUTTON:
                self._log.info(Style.DIM + 'START_BUTTON.')
            case Event.SELECT_BUTTON:
                self._log.info(Style.DIM + 'SELECT_BUTTON.')
            case Event.HOME_BUTTON:
                self._log.info(Style.DIM + 'HOME_BUTTON.')
            case Event.DPAD_HORIZONTAL:
                self._log.info(Style.DIM + 'DPAD_HORIZONTAL.')
            case Event.DPAD_LEFT:
                self._log.info(Style.DIM + 'DPAD_LEFT. (used by nudge)')
                # used by nudge
            case Event.DPAD_RIGHT:
                self._log.info(Style.DIM + 'DPAD_RIGHT. (used by nudge)')
                # used by nudge
            case Event.DPAD_VERTICAL:
                self._log.info(Style.DIM + 'DPAD_VERTICAL.')
            case Event.DPAD_UP:
                self._log.info(Style.DIM + 'DPAD_UP. (used by nudge)')
                # used by nudge
            case Event.DPAD_DOWN:
                self._log.info(Style.DIM + 'DPAD_DOWN. (used by nudge)')
                # used by nudge
            case Event.L3_VERTICAL:
                self._log.info(Style.DIM + 'L3_VERTICAL.')
            case Event.L3_HORIZONTAL:
                self._log.info(Style.DIM + 'L3_HORIZONTAL.')
            case Event.R3_VERTICAL:
                self._log.info(Style.DIM + 'R3_VERTICAL.')
            case Event.R3_HORIZONTAL:
                self._log.info(Style.DIM + 'R3_HORIZONTAL.')
            case _:
                self._log.info(Style.DIM + 'UNRECOGNISED.')

    def pir(self):
        try:
            if self._tinyfx:
                elapsed_sec = self._tinyfx.pir()
                self._log.info(Fore.WHITE + 'pir elapsed: {} sec'.format(elapsed_sec))
            else:
                self._log.warning('no tinyfx available.')
        except Exception as e:
            self._log.error('{} raised in pir: {}'.format(type(e), e))

    def disable(self):
        if not self.enabled:
            self._log.debug('already disabled.')
            return
        self._log.info('disabling thoughts…')
        self._set_running_lights(False)
        self._motor_controller.set_show_battery(True)
        # cancel the async task before calling Behaviour.disable()
        if self._thoughts_task and not self._thoughts_task.done():
            self._log.debug('cancelling thoughts listener loop task…')
            self._thoughts_task.cancel()
        self._loop_running = False
        Behaviour.disable(self)
        self._log.info('disabled.')

    def close(self):
        '''
        Permanently close the behaviour.
        '''
        if not self.closed:
            self._log.debug('closing thoughts…')
            Behaviour.close(self)
            self._log.info('closed.')

#EOF
