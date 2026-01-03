#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2020-05-19
# modified: 2026-01-03

import itertools
import asyncio
import time
import traceback
from threading import Event as ThreadEvent
from math import isclose
from random import choice, expovariate, randint, uniform
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component, MissingComponentError
from core.event import Event, Group
from core.orientation import Orientation
from core.rate_limited import rate_limited
from core.subscriber import Subscriber
from core.queue_publisher import QueuePublisher
from behave.behaviour import Behaviour
from hardware.odometer import Odometer
from hardware.eyeball import Eyeball
from hardware.eyeballs_monitor import EyeballsMonitor
from hardware.lux_sensor import LuxSensor
from hardware.motor_controller import MotorController
from hardware.rotation_controller import RotationController
from hardware.stm32_controller import Stm32Controller
from hardware.tinyfx_controller import TinyFxController
from hardware.imu import IMU

class Thoughts(Behaviour):
    NAME = 'thoughts'
    _LISTENER_LOOP_NAME = '__thoughts_listener_loop'
    '''
    Monitors robot activity and plays sounds ("random thoughts") via the TinyFX
    at roughly a frequency corresponding to the message traffic and motor speed,
    monitored via the Odometer.

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
        self._verbose           = True #_cfg.get('verbose', False)
        self._priority          = _cfg.get('priority', 0.2)
        self._max_activity      = _cfg.get('max_activity', 4.0)
        self._activity_scale    = _cfg.get('activity_scale', 1.0)
        self._jitter_factor     = _cfg.get('jitter_factor', 2.5)
        self._max_frequency     = _cfg.get('max_frequency', 0.8)
        self._rate_delay_ms     = _cfg.get('rate_limit_ms', 1500)
        self._bored_sound       = _cfg.get('bored_sound')
        self._sleeping_sound    = _cfg.get('sleeping_sound')
        self._enable_sound_fx   = True
        self._active_sounds     = _cfg.get('active_sounds', [])
        self._enable_light_ctrl = _cfg.get('enable_light_ctrl')
        self._enable_head_light = _cfg.get('enable_head_light')
        # TODO config
        self._bored_limit_min   = 10   # how long before we get bored?
        self._sleep_limit_min   = 17   # how many cycles before falling asleep?
        self._deep_sleep_limit_min = 5 # how many cycles asleep before falling into deep sleep?
        self._snore_limit_min   = 7    # how many snores?
        self._snore_rate_min    = 8    # how fast to snore?
        if not self._active_sounds:
            raise ValueError('no sounds configured.')
        self._log.info('configured {} sounds; max activity: {:4.2f}; activity scale: {:4.2f}; jitter: {:4.2f}; max freq: {:4.2f}Hz; rate limit: {:d}ms'.format(
            len(self._active_sounds), self._max_activity, self._activity_scale, self._jitter_factor, self._max_frequency, self._rate_delay_ms))
        self._poll_pir = False # not currently functional
        # components
        self._component_registry = Component.get_registry()
        # queue publisher
        self._enable_publishing = True
        self._queue_publisher = self._component_registry.get(QueuePublisher.NAME)
        if self._queue_publisher is None:
            raise MissingComponentError('queue publisher not available.')
        # others…
        self._imu = self._component_registry.get(IMU.NAME)
        if not self._imu:
            self._log.warning('IMU not available.' )
        self._eyeballs_monitor = self._component_registry.get(EyeballsMonitor.NAME)
        if self._eyeballs_monitor is None:
            self._log.warning('eyeballs monitor not available.')
        self._odometer = self._component_registry.get(Odometer.NAME)
        if self._odometer is None:
            self._log.info('odometer not available; relying on message activity alone.')
        else:
            self._log.info('odometer available; will monitor for movement.')
            self._odometer.add_callback(self._odometer_callback)
        self._lux_threshold = _cfg.get('lux_threshold')
        self._lux_sensor = self._component_registry.get(LuxSensor.NAME)
        if self._enable_head_light and self._lux_sensor is None:
            try:
                self._lux_sensor = LuxSensor(config)
            except Exception:
                self._log.warning('unable to open lux sensor.')
        self._darkness_state = False
        self._motor_controller = self._component_registry.get(MotorController.NAME)
        self._rotation_controller = self._component_registry.get(RotationController.NAME)
        # state tracking
        self._count = 0
        self._counter = itertools.count()
        self._last_activity_time = None
        self._last_called = 0
        self._last_name = None
        self._next_play_time = 0.0
        self._stop_event = ThreadEvent()
        self._max_normalised = 0.0
        # subscribe to all non-IDLE events to detect activity
        self._thoughts_task = None
        self._loop_running = False
        self.add_events([member for member in Group
                        if member not in (Group.NONE, Group.IDLE, Group.OTHER)])
        # external controllers
        self._tinyfx = None
        self._stm32  = None
        # behavioural states
        self._enable_imu_poll  = False
        self._bored            = False
        self._sleeping         = False
        self._idle_count       = 0
        self._snore_count      = 0
        self._sleep_count      = 0
        self._bored_limit      = 0 # how long before we get bored?
        self._sleep_limit      = 0 # how many cycles before falling asleep?
        self._deep_sleep_limit = 0 # how many cycles asleep before falling into deep sleep?
        self._snore_limit      = 0 # how many snores?
        self._snore_rate       = 0 # how fast to snore?
        self._reset_sleepiness()   # set to initial values
        self._reverse_count    = 0
        self._reverse_limit    = 2 # how many cycles backwards before we notice?
        self._reversing        = False
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
        if not self.enabled:
            self._log.info('enabling random thoughts…')
            self._stop_event.clear()
            super().enable()
            # initialize activity timer
            self._mark_activity()
            self._next_play_time = time.monotonic()
            self._tinyfx = self._component_registry.get(TinyFxController.NAME)
            self._stm32  = self._component_registry.get(Stm32Controller.NAME)
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
        else:
            self._log.warning('already enabled.')

    def _odometer_callback(self):
        if self._verbose:
            if self._count % 10 == 0: # every 10 seconds
                self._log.info(Fore.BLUE + 'odometer reports movement (10x); idle count: {}'.format(self._idle_count))
            else:
                self._log.debug(Fore.BLACK + 'odometer reports movement; idle count: {}'.format(self._idle_count))

    def _reset_sleepiness(self):
        '''
        Resets all state variables related to sleepiness.
        '''
        self._last_activity_time = dt.now()
        self._last_idle_publish_time = None
        self._sleeping    = False
        self._bored       = False
        self._idle_count  = 0
        self._snore_count = 0
        self._sleep_count = 0
        self._idle_count  = 0
        self._snore_count = 0
        self._bored_limit = randint(self._bored_limit_min, self._bored_limit_min + 6)
        self._sleep_limit = randint(self._sleep_limit_min, self._sleep_limit_min + 6)
        self._deep_sleep_limit = randint(self._deep_sleep_limit_min, self._deep_sleep_limit_min + 3)
        self._snore_limit = randint(self._snore_limit_min, self._snore_limit_min + 3)
        self._snore_rate  = randint(self._snore_rate_min, self._snore_rate_min + 3)

    def _mark_activity(self):
        '''
        Called to mark any activity and keep the robot awake.
        '''
        self._log.info(Fore.GREEN + 'marked activity.')
        self._reset_sleepiness()
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

    def _rotate_to_heading(self):
        if self._rotation_controller:
            self._rotation_controller.rotate_absolute(0)

    def _toggle_running_lights(self):
        self._set_running_lights(not self._running_lights_state)

    def _set_running_lights(self, enable):
        if self._enable_light_ctrl:
            if enable == self._running_lights_state:
                self._log.info('running lights already set to: ' + Style.DIM +'{}'.format('on' if enable else 'off'))
                return
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
        if _eyeballs is None:
            self._log.debug('no manual setting for eyeballs, setting blank…')
            self._eyeballs_monitor.set_eyeballs(Eyeball.BLANK, temporary=True)
        elif _eyeballs == Eyeball.NEUTRAL \
                or _eyeballs == Eyeball.BORED \
                or _eyeballs == Eyeball.SLEEPY \
                or _eyeballs == Eyeball.DEEP_SLEEP:
            self._log.info('clearing eyeballs…')
            self._eyeballs_monitor.set_eyeballs(Eyeball.BLANK, temporary=True)
        else:
            self._log.warning('did not reset eyeballs from {}'.format(_eyeballs))

    async def _thoughts_listener_loop(self, f_is_enabled):
        '''
        Main loop: monitors activity and plays sounds organically.
        Goes from boredom to sleep.

        Activity is detected from:
        - Motor movement (checked every iteration if odometer available)
        - Message bus traffic (via process_message())
        '''
        self._log.info('loop started (threshold: {:d}s)'.format(self._idle_threshold_sec))
        while f_is_enabled() and not self._stop_event.is_set():
            try:
                self._count = next(self._counter)
                if self.has_toggle_assignment():
                    if self.suppressed and self.is_released_by_toggle():
                        self._log.info(Fore.WHITE + Style.BRIGHT + 'releasing…')
                        self.release()
                    elif self.released and not self.is_released_by_toggle():
                        self._log.info(Fore.WHITE + Style.BRIGHT + 'suppressing…')
                        self.suppress()
                if self.suppressed:
                    if self._verbose:
                        self._log.info(Fore.BLUE + '[{:05d}] suppressed.'.format(self._count))
                    await asyncio.sleep(self._loop_delay_sec)
                    continue
                if self._enable_imu_poll and self._imu:
                    self.poll_imu()
                # check odometer for movement activity
                if self._odometer:
                    vx, vy, omega = self._odometer.velocity
                    _raw_activity = abs(vx) + abs(vy) + abs(omega)
                    self._log.info(Fore.GREEN + '🤢 vx: {:4.2f}; vy: {:4.2f}; omega: {:4.2f}'.format(vx, vy, omega))
                    if isclose(_raw_activity, 0.0, abs_tol=0.001):
                        _style = Style.DIM
                        _raw_activity = 0.0
                        self._idle_count += 1
                        self._reverse_count = 0
                        self._update_reverse_state()
                    else:
                        self._idle_count = 0
                        if vy < 0:
                            self._reverse_count += 1
                            self._update_reverse_state()
                        elif vy > 0:
                            self._reverse_count = 0
                            self._update_reverse_state()
                        # reset, there's been activity
                        _style = Style.NORMAL
                    # normalize and scale activity
                    _normalised = _raw_activity / self._max_activity
                    _activity = _normalised * self._activity_scale
                    self._max_normalised = max(self._max_normalised, _normalised)
                    self._log.info(Fore.GREEN + '🤢 activity raw: {:4.2f}; normalised: {:4.2f}; max: {:4.2f}'.format(_raw_activity, _normalised, self._max_normalised))
                    _activity = min(_activity, 1.0)
                    if self._verbose:
                        _idle_style     = Style.NORMAL if ( not self._bored and not self._sleeping ) else Style.DIM
                        _bored_style    = Style.NORMAL if ( self._bored and not self._sleeping ) else Style.DIM
                        _sleeping_style = Style.NORMAL if self._sleeping else Style.DIM
                        _deep_sleep_style = Style.NORMAL if self._sleep_count > self._deep_sleep_limit else Style.DIM
                        self._log.info(Fore.BLUE + _style + '[{:05d}] activity: {:4.2f} (raw: {:4.2f});'.format(
                               self._count, _activity, _raw_activity)
                               + Style.NORMAL
                               + Fore.WHITE   + _idle_style       + ' idle: {:4d};'.format(self._idle_count)
                               + Fore.YELLOW  + _bored_style      + ' bored: {};'.format(self._bored_limit)
                               + Fore.BLUE    + _sleeping_style   + ' sleep: {};'.format(self._sleep_limit)
                               + Fore.MAGENTA + _deep_sleep_style + ' deep sleep: {};'.format(self._deep_sleep_limit)
                               + Fore.BLACK   + Style.DIM         + ' snore: {}'.format(self._snore_rate)
                        )
                    if self._idle_count == 0: # activity ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                        self._maybe_play_random_sound(_activity)
                        self._mark_activity()

                    elif self._sleeping: # sleeping ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                        if self._count % self._snore_rate == 0: # every n seconds, snore
                            self._log.info(Fore.BLUE + Style.DIM + 'still sleeping… [{}]'.format(self._count))
                            self._snore_count += 1
                            if self._snore_count < self._snore_limit:
                                self.play_sound('snore')
                                self._snore_rate = randint(8, 11) # change snore rate
                            else:
                                if self._poll_pir: # check for human/cat activity
                                    self.pir()
                                elif self._sleep_count > self._deep_sleep_limit:
                                    self._eyeballs_monitor.set_eyeballs(Eyeball.DEEP_SLEEP)
                                    self.play_sound('quiet-breathing')
                                else:
                                    self._sleep_count += 1
                                    self._set_running_lights(False)
                                    time.sleep(0.1)
                                    self.play_sound('breathing')

                    elif self._idle_count % self._sleep_limit == 0: # start sleeping ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                        self._log.info(Fore.BLUE + 'sleeping…')
                        self._sleeping    = True
                        self._sleep_limit = randint(21, 31) # reset to different threshold
                        self._log.info(Fore.BLUE + 'sleeping at: {}.'.format(self._sleep_limit))
                        self.play_sound(self._sleeping_sound)
                        self._eyeballs_monitor.set_eyeballs(Eyeball.SLEEPY)

                    elif not self._bored and self._idle_count % self._bored_limit == 0: # bored ┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                        self._log.info(Fore.BLUE + 'bored…')
                        self._bored = True
                        self._bored_limit = randint(11, 17)
#                       self._log.info(Fore.BLUE 'bored at: {}.'.format(self._bored_limit))
                        self.play_sound(self._bored_sound)
                        self._eyeballs_monitor.set_eyeballs(Eyeball.BORED)

                    else:
#                       self._log.info('something else…')
                        pass
                # calculate elapsed time since last activity
                elapsed_sec = self.elapsed_seconds
                if elapsed_sec >= self._idle_threshold_sec:
                    if self._verbose:
                        self._log.info(Fore.BLUE + Style.DIM + '[{:05d}] idle ({:.1f}s)'.format(self._count, elapsed_sec))
                else:
                    if self._verbose:
                        self._log.info(Fore.BLUE + '[{:05d}] active ({:.1f}s since last activity)'.format(self._count, elapsed_sec))
                self._check_light_level()
                if self._sleep_count > 0:
                    # randomly longer
                    await asyncio.sleep(self._loop_delay_sec * uniform(1.2, 2.0))
                else:
                    await asyncio.sleep(self._loop_delay_sec)

            except asyncio.CancelledError:
                self._log.info('thoughts listener loop cancelled.')
                return
            except Exception as e:
                self._log.error('{} raised in loop: {}\n{}'.format(type(e), e, traceback.format_exc()))
                raise
        self._log.info(Style.DIM + 'exited thoughts listener loop; f_enabled? {}; stop event set? {}'.format(f_is_enabled(), self._stop_event.is_set()))

    def _check_light_level(self):
        '''
        After two ticks, begins checking the lux level. If it has changed,
        enable/disable the headlight.
        '''
        if self._tinyfx and self._lux_sensor and self._count > 2:
            _lux_level = self._lux_sensor.lux
            _is_dark = _lux_level < self._lux_threshold
            if not self._running_lights_state:
                _is_dark = False
            if self._sleeping:
                _is_dark = False
            if self._darkness_state != _is_dark: # things have changed
                self._log.info('lux level: {}; is dark? {}; was dark? {}'.format(
                        _lux_level, _is_dark, self._darkness_state))
                _saved = self._suppress_random_sounds
                self._suppress_random_sounds = True
                try:
                    time.sleep(0.33)
                    self._tinyfx.light(Orientation.FWD, _is_dark)
                finally:
                    self._suppress_random_sounds = _saved
                    self._darkness_state = _is_dark

    def _publish_message(self, message):
        '''
        Publishes the message.
        '''
        if self._enable_publishing:
            self._log.info("publishing {} message…".format(message.event))
            try:
                self._queue_publisher.put(message)
                self._log.info("{} message published.".format(message.event))
            except Exception as e:
                self._log.error('{} encountered when publishing message: {}\n{}'.format(
                    type(e), e, traceback.format_exc()))
        else:
            self._log.warning('publishing disabled.')

    def _publish_stuck(self):
        self._log.info('publish STUCK message.')
        message = self._message_factory.create_message(Event.STUCK, 'thoughts')
        self._publish_message(message)

    def _update_reverse_state(self):
        '''
        If the robot has been reversing for enough loop cycles, turn on the backup lights.
        '''
        # if we're potentially reversing, don't play random sounds
        self._suppress_random_sounds = self._reverse_count > 0
        reversing = self._reverse_count >= self._reverse_limit
        if reversing == self._reversing:
            if self._motor_controller and self._motor_controller.is_stopped:
                self._log.debug('update reversing: motor stopped.')
                reversing = False
            else:
                self._log.debug('update reversing; motor not stopped.')
                return # no change of state
        if reversing:
            self._log.debug('update reversing.')
            if self._tinyfx:
                self._tinyfx.light(Orientation.AFT, True)
                time.sleep(0.1)
                self.play_sound('backing')
            else:
                self._log.warning('no tinyfx.')
            self._log.debug(Fore.GREEN + 'reversing… ({})'.format(self._reverse_count))
        else:
            self._log.debug('update reversing.')
            if self._tinyfx:
                self._tinyfx.light(Orientation.AFT, False)
            else:
                self._log.warning('no tinyfx.')
            self._log.debug(Fore.GREEN + Style.DIM + 'no longer reversing… ({})'.format(self._reverse_count))
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
                    self.play_sound(name)
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

        Mapping:
            A_BUTTON:   no action
            B_BUTTON:   no action
            X_BUTTON:   no action
            Y_BUTTON:   shut down robot
            L1_BUTTON:  toggle running lights
            L2_BUTTON:  toggle interior light
            R1_BUTTON:  play 'ugh'
            R2_BUTTON:  suppress/release sound effects
            DPAD_LEFT:  used by Nudge
            DPAD_RIGHT: used by Nudge
        '''
        event = message.event
        self._log.info(Style.DIM + 'execute message with event: {}'.format(event))
        match(event):
            case Event.A_BUTTON:
                self._log.info(Style.DIM + 'A_BUTTON.')
                self._publish_stuck()
            case Event.B_BUTTON:
                self._log.debug(Style.DIM + 'B_BUTTON. rotate north')
                self._rotate_to_heading()
            case Event.X_BUTTON:
                self._enable_imu_poll = not self._enable_imu_poll  
                self._log.info(Fore.YELLOW + 'X_BUTTON. poll IMU? {}'.format(self._enable_imu_poll))
            case Event.Y_BUTTON:
                self._log.info(Style.DIM + 'Y_BUTTON. exit')
            case Event.L1_BUTTON:
                self._log.info(Fore.MAGENTA + 'L1_BUTTON. toggle running lights')
                self._toggle_running_lights()
            case Event.L2_BUTTON:
                self._log.info(Style.DIM + 'L2_BUTTON. toggle interior light')
                self._toggle_interior_light()
            case Event.R1_BUTTON:
                self._log.info(Fore.MAGENTA + 'R1_BUTTON. play sound ugh')
                self.play_sound('ugh')
            case Event.R2_BUTTON:
                self._suppress_sounds = not self._suppress_sounds
                if self._suppress_sounds:
                    self._log.info(Fore.GREEN + 'R2_BUTTON. sounds are suppressed.')
                else:
                    self._log.info(Fore.GREEN + 'R2_BUTTON. sounds are not suppressed.')
            case Event.START_BUTTON:
                self._log.debug(Style.DIM + 'START_BUTTON.')
            case Event.SELECT_BUTTON:
                self._log.debug(Style.DIM + 'SELECT_BUTTON.')
            case Event.HOME_BUTTON:
                self._log.info(Fore.GREEN + 'HOME_BUTTON. print registry.')
                self._component_registry.print_registry()
            case Event.DPAD_HORIZONTAL:
                self._log.debug(Style.DIM + 'DPAD_HORIZONTAL.')
            case Event.DPAD_LEFT:
                self._log.debug(Style.DIM + 'DPAD_LEFT. (used by nudge)')
                # used by nudge
            case Event.DPAD_RIGHT:
                self._log.debug(Style.DIM + 'DPAD_RIGHT. (used by nudge)')
                # used by nudge
            case Event.DPAD_VERTICAL:
                self._log.debug(Style.DIM + 'DPAD_VERTICAL.')
            case Event.DPAD_UP:
                self._log.debug(Style.DIM + 'DPAD_UP. (used by nudge)')
                # used by nudge
            case Event.DPAD_DOWN:
                self._log.debug(Style.DIM + 'DPAD_DOWN. (used by nudge)')
                # used by nudge
            case Event.L3_VERTICAL:
                self._log.debug(Style.DIM + 'L3_VERTICAL.')
            case Event.L3_HORIZONTAL:
                self._log.debug(Style.DIM + 'L3_HORIZONTAL.')
            case Event.R3_VERTICAL:
                self._log.debug(Style.DIM + 'R3_VERTICAL.')
            case Event.R3_HORIZONTAL:
                self._log.debug(Style.DIM + 'R3_HORIZONTAL.')
            case _:
#               self._log.debug('unrecognised event: {}'.format(event))
                pass

    def poll_imu(self):
        self._log.debug('polling IMU… ')
        self._imu.poll()
        self._imu.show_info()

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
        if self.enabled:
            self._log.debug('disabling…')
            self._stop_event.set()
            self._set_running_lights(False)
            self._motor_controller.set_show_battery(True)
            # cancel the async task before calling Behaviour.disable()
            if self._thoughts_task and not self._thoughts_task.done():
                self._log.debug('cancelling thoughts listener loop task…')
                self._thoughts_task.cancel()
            self._loop_running = False
            super().disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

    def close(self):
        if not self.closed:
            self._log.debug('closing…')
            super().close()
            self._log.info('closed.')
        else:
            self._log.warning('already closed.')

#EOF
