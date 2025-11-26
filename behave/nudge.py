#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-26
# modified: 2025-11-26
#
# Nudge: temporary lateral/rotational nudges (port/starboard) and
# speed multiplier nudges (fore/aft).
#
# Public API:
#   nudge(self, orientation, time_ms)   # synchronous
#   cancel(self)                        # synchronous

import time
import asyncio
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.orientation import Orientation
from core.subscriber import Subscriber
from behave.async_behaviour import AsyncBehaviour
from core.event import Event, Group
from hardware.motor_controller import MotorController

class Nudge(AsyncBehaviour):
    NAME = 'nudge'

    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Nudge.NAME, level)
        _component_registry = Component.get_registry()
        _motor_controller = _component_registry.get(MotorController.NAME)
        if _motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        AsyncBehaviour.__init__(self, self._log, config, message_bus, message_factory, _motor_controller, level=level)
        # configuration (kros.behaviour.nudge)
        _cfg = config['kros'].get('behaviour').get('nudge')
        self._verbose               = _cfg.get('verbose', False)
        self._lateral_max           = _cfg.get('lateral_max', 0.5)
        self._omega_max             = _cfg.get('omega_max', 0.5)
        self._speed_multiplier_max  = _cfg.get('speed_multiplier_max', 2.0)
        self._enable_negative_speed = _cfg.get('enable_negative_speed', False)
        self._hold_ms_default       = _cfg.get('hold_ms_default', 2000)
        self._ramp_up_ms            = _cfg.get('ramp_up_ms', 1500)
        self._ramp_down_ms          = _cfg.get('ramp_down_ms', 1500)
        self._priority              = _cfg.get('priority', 0.3)
        # internal state
        self._current_vx            = 0.0
        self._current_omega         = 0.0
        self._target_vx             = 0.0
        self._target_omega          = 0.0
        self._current_speed_multiplier = 1.0
        self._target_speed_multiplier  = 1.0
        self._hold_start_time       = None
        self._hold_end_time         = None
        self._pending_hold_ms       = 0
        self._speed_modifier_registered = False
        self._speed_modifier_name   = 'nudge-speed'
        self._active = False
        self._eps = 1e-3
        self.add_events(Event.by_groups([Group.GAMEPAD]))
        self._log.info('ready.')

    @property
    def name(self):
        return Nudge.NAME

    @property
    def is_ballistic(self):
        return False

    @property
    def priority(self):
        return self._priority

    # public API ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def nudge(self, orientation, time_ms):
        '''
        Synchronous API to request a temporary nudge.

        orientation: Orientation.PORT / STBD / FWD / AFT
        time_ms: hold time in milliseconds (0 => cancel)
        '''
        if time_ms == 0:
            self.cancel()
            return

        hold_ms = int(time_ms) if time_ms and time_ms > 0 else self._hold_ms_default

        if orientation is Orientation.PORT or orientation is Orientation.STBD:
            # determine sign: PORT -> -1, STBD -> +1
            sign = -1.0 if orientation is Orientation.PORT else 1.0
            # target lateral and rotation (omega). omega sign chosen to provide heading change complementary to lateral move.
            self._target_vx = sign * self._lateral_max
            self._target_omega = -sign * self._omega_max
            # set pending hold; actual hold timer starts when peak reached
            self._pending_hold_ms = hold_ms
            # reset hold markers so we ramp up then hold
            self._hold_start_time = None
            self._hold_end_time = None
            self._active = True
            self._log.info('nudge requested: {} for {}ms -> target vx={:.3f}, omega={:.3f}'.format(
                orientation.name, hold_ms, self._target_vx, self._target_omega))

        elif orientation is Orientation.FWD or orientation is Orientation.AFT:
            # speed multiplier nudges: FWD -> boost up to max; AFT -> stop or reverse (if enabled)
            if orientation is Orientation.FWD:
                self._target_speed_multiplier = float(self._speed_multiplier_max)
            else: # AFT
                if self._enable_negative_speed:
                    # full reverse
                    self._target_speed_multiplier = -1.0
                else:
                    # reduce to zero (stop)
                    self._target_speed_multiplier = 0.0
            self._pending_hold_ms = hold_ms
            self._hold_start_time = None
            self._hold_end_time = None
            self._active = True
            self._log.info('nudge requested: {} for {}ms -> speed multiplier target={:.3f}'.format(
                orientation.name, hold_ms, self._target_speed_multiplier))
            # ensure speed modifier present
            if not self._speed_modifier_registered:
                try:
                    # lambda must be a <lambda> per MotorController enforcement; closure reads current multiplier
                    modifier = (lambda speeds, _self=self: _self._nudge_speed_modifier(speeds))
                    self._motor_controller.add_speed_modifier(self._speed_modifier_name, modifier, exclusive=False)
                    self._speed_modifier_registered = True
                    self._log.info('speed modifier registered: {}'.format(self._speed_modifier_name))
                except Exception as e:
                    # fail fast per your preference
                    raise

        else:
            raise ValueError('unsupported orientation for nudge: {}'.format(orientation))

    def cancel(self):
        '''
        Cancel any active nudge immediately: ramp targets to zero and remove speed modifier.
        '''
        self._log.info('cancel called: clearing targets and scheduling ramp down.')
        # set targets to zero so poll ramps down
        self._target_vx = 0.0
        self._target_omega = 0.0
        self._target_speed_multiplier = 1.0
        # clear hold timers so we immediately enter ramp-down
        self._hold_start_time = None
        self._hold_end_time = None
        self._pending_hold_ms = 0
        self._active = True  # keep loop active until values ramp down to zero/1.0
        # removal of modifier will happen once multiplier returns to 1.0 in the poll loop,
        # but force removal if currently registered and multiplier already 1.0
        if self._speed_modifier_registered and abs(self._current_speed_multiplier - 1.0) <= self._eps:
            try:
                self._motor_controller.remove_speed_modifier(self._speed_modifier_name)
            finally:
                self._speed_modifier_registered = False

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    async def process_message(self, message):
        '''
        Process the message.

        :param message:  the message to process.
        '''
        if message.gcd:
            raise GarbageCollectedError('cannot process message: message has been garbage collected. [3]')
        _event = message.event
        self._log.info(Fore.MAGENTA + '💜 pre-processing message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.label))
        if _event.group is Group.GAMEPAD:
            self._log.info(Fore.MAGENTA + '💜 processing system message {}'.format(message.name))
            self.execute(message)
        else:
            self._log.warning('unrecognised event on message {}'.format(message.name) + ''.format(message.event.label))
        await Subscriber.process_message(self, message)
        self._log.info(Fore.MAGENTA + '💜 post-processing message {}'.format(message.name))

    def execute(self, message):
        '''
        Behaviour.execute() - not used by Nudge.

        Nudge operates via process_message() as a Subscriber.
        '''
        self._log.info(Fore.MAGENTA + '💜 processing system message {}'.format(message.name))
        event = message.event
        match(event):
            case Event.A_BUTTON:
                self._log.info(Style.DIM + '🌼 A_BUTTON.')
            case Event.B_BUTTON:
                self._log.info(Style.DIM + '🌼 B_BUTTON.')
            case Event.X_BUTTON:
                self._log.info(Style.DIM + '🌼 X_BUTTON.')
            case Event.Y_BUTTON:
                self._log.info(Style.DIM + '🌼 Y_BUTTON.')
            case Event.L1_BUTTON:
                self._log.info(Style.DIM + 'L1_BUTTON.')
            case Event.L2_BUTTON:
                self._log.info(Style.DIM + 'L2_BUTTON.')
            case Event.R1_BUTTON:
                self._log.info(Style.DIM + 'R1_BUTTON.')
            case Event.R2_BUTTON:
                self._log.info(Style.DIM + 'R2_BUTTON.')
            case Event.START_BUTTON:
                self._log.info(Style.DIM + 'START_BUTTON.')
            case Event.SELECT_BUTTON:
                self._log.info(Style.DIM + 'SELECT_BUTTON.')
            case Event.HOME_BUTTON:
                self._log.info(Style.DIM + 'HOME_BUTTON.')
            case Event.DPAD_HORIZONTAL:
                self._log.info(Style.DIM + '🌸 DPAD_HORIZONTAL.')
            case Event.DPAD_LEFT:
                self._log.info(Style.DIM + '🌸 DPAD_LEFT.')
                self.nudge(Orientation.PORT, self._hold_ms_default)
            case Event.DPAD_RIGHT:
                self._log.info(Style.DIM + '🌸 DPAD_RIGHT.')
                self.nudge(Orientation.STBD, self._hold_ms_default)
            case Event.DPAD_VERTICAL:
                self._log.info(Style.DIM + '🌸 DPAD_VERTICAL.')
            case Event.DPAD_UP:
                self._log.info(Style.DIM + '🌸 DPAD_UP.')
                self.nudge(Orientation.FWD, self._hold_ms_default)
            case Event.DPAD_DOWN:
                self._log.info(Style.DIM + '🌸 DPAD_DOWN.')
                self.nudge(Orientation.AFT, self._hold_ms_default)
            case Event.L3_VERTICAL:
                self._log.info(Style.DIM + '🌸 L3_VERTICAL.')
            case Event.L3_HORIZONTAL:
                self._log.info(Style.DIM + '🌸 L3_HORIZONTAL.')
            case Event.R3_VERTICAL:
                self._log.info(Style.DIM + '🌸 R3_VERTICAL.')
            case Event.R3_HORIZONTAL:
                self._log.info(Style.DIM + '🌸 R3_HORIZONTAL.')
            case _:
                self._log.info(Style.DIM + 'UNRECOGNISED.')


    def _nudge_speed_modifier(self, speeds):
        '''
        MotorController speed modifier function. Called on every motor tick.
        It scales all speeds by the current speed multiplier and requests its
        own removal (by returning its name) when the multiplier is effectively 1.0
        and no nudge is active.
        '''
        factor = float(self._current_speed_multiplier)
        modified = [s * factor for s in speeds]
        # if no active nudge and factor nearly 1.0, request removal
        if (not self._active) and abs(factor - 1.0) <= self._eps:
            return self._speed_modifier_name
        return modified

    async def _poll(self):
        '''
        Called by AsyncBehaviour loop. Returns the current intent vector (vx, vy, omega).
        This method drives ramping logic and the transient hold timing.
        '''
        # quick aliases
        poll_ms = float(self._poll_delay_ms)
        now = time.monotonic()

        # compute per-tick max changes (linear)
        # lateral & omega per-tick rate (toward a non-zero target) uses ramp_up_ms; ramp to zero uses ramp_down_ms
        def per_tick_rate(current, target, ramp_ms):
            if ramp_ms <= 0:
                return abs(target - current)
            return abs(target - current) * (poll_ms / float(ramp_ms))

        # HANDLE SPEED MULTIPLIER RAMPING
        if abs(self._current_speed_multiplier - self._target_speed_multiplier) > self._eps:
            # determine whether ramping up (toward non-1.0) or down (toward 1.0/zero)
            ramp_ms = self._ramp_up_ms if abs(self._target_speed_multiplier - self._current_speed_multiplier) > 0 else self._ramp_down_ms
            step = per_tick_rate(self._current_speed_multiplier, self._target_speed_multiplier, self._ramp_up_ms)
            delta = self._target_speed_multiplier - self._current_speed_multiplier
            if abs(delta) <= step:
                self._current_speed_multiplier = self._target_speed_multiplier
            else:
                self._current_speed_multiplier += (step if delta > 0 else -step)
        # If active and we have reached the target speed multiplier and no hold started, start hold timer
        if self._active and abs(self._current_speed_multiplier - self._target_speed_multiplier) <= self._eps:
            if self._pending_hold_ms > 0 and self._hold_start_time is None:
                # start hold when peak reached
                self._hold_start_time = now
                self._hold_end_time = now + (self._pending_hold_ms / 1000.0)
                self._log.info('speed hold started; will end at {:.3f}'.format(self._hold_end_time))
        # After hold ends, set target back to 1.0 (neutral)
        if self._hold_end_time is not None and now >= self._hold_end_time:
            # clear hold and set ramp-down target
            self._log.info('speed hold ended; ramping down to neutral')
            self._hold_start_time = None
            self._hold_end_time = None
            self._pending_hold_ms = 0
            self._target_speed_multiplier = 1.0
            # mark no longer active for speed if vx/omega also zero
            if abs(self._current_vx) <= self._eps and abs(self._current_omega) <= self._eps:
                self._active = True  # keep active until speed ramps down in this same tick

        # HANDLE LATERAL / OMEGA RAMPING
        # If currently moving toward a non-zero target
        if abs(self._current_vx - self._target_vx) > self._eps:
            step_vx = per_tick_rate(self._current_vx, self._target_vx, self._ramp_up_ms)
            delta_vx = self._target_vx - self._current_vx
            if abs(delta_vx) <= step_vx:
                self._current_vx = self._target_vx
            else:
                self._current_vx += (step_vx if delta_vx > 0 else -step_vx)
        if abs(self._current_omega - self._target_omega) > self._eps:
            step_om = per_tick_rate(self._current_omega, self._target_omega, self._ramp_up_ms)
            delta_om = self._target_omega - self._current_omega
            if abs(delta_om) <= step_om:
                self._current_omega = self._target_omega
            else:
                self._current_omega += (step_om if delta_om > 0 else -step_om)

        # If we have reached lateral/omega peak and no hold started, start hold
        if self._active and abs(self._current_vx - self._target_vx) <= self._eps and abs(self._current_omega - self._target_omega) <= self._eps:
            if self._pending_hold_ms > 0 and self._hold_start_time is None:
                self._hold_start_time = now
                self._hold_end_time = now + (self._pending_hold_ms / 1000.0)
                self._log.info('lateral/omega hold started; will end at {:.3f}'.format(self._hold_end_time))

        # After hold ends for lateral/omega, set targets back to zero
        if self._hold_end_time is not None and now >= self._hold_end_time:
            self._log.info('lateral/omega hold ended; ramping down to neutral')
            self._hold_start_time = None
            self._hold_end_time = None
            self._pending_hold_ms = 0
            self._target_vx = 0.0
            self._target_omega = 0.0

        # If both lateral/omega and speed multiplier are at neutral/1.0, and no pending hold, mark inactive and remove speed modifier if present
        lateral_zero = abs(self._current_vx) <= self._eps and abs(self._current_omega) <= self._eps
        speed_neutral = abs(self._current_speed_multiplier - 1.0) <= self._eps
        if lateral_zero and speed_neutral and self._pending_hold_ms == 0:
            # ensure variables exactly neutral
            self._current_vx = 0.0
            self._current_omega = 0.0
            self._current_speed_multiplier = 1.0
            self._target_vx = 0.0
            self._target_omega = 0.0
            self._target_speed_multiplier = 1.0
            if self._speed_modifier_registered:
                try:
                    self._motor_controller.remove_speed_modifier(self._speed_modifier_name)
                finally:
                    self._speed_modifier_registered = False
            self._active = False

        # ensure we never return NaNs
        vx = float(self._current_vx)
        vy = 0.0
        omega = float(self._current_omega)

        return (vx, vy, omega)

    def stop_loop_action(self):
        # ensure any registered speed modifier removed on loop stop
        if self._speed_modifier_registered:
            try:
                self._motor_controller.remove_speed_modifier(self._speed_modifier_name)
            finally:
                self._speed_modifier_registered = False
        # reset internals
        self._current_vx = 0.0
        self._current_omega = 0.0
        self._current_speed_multiplier = 1.0
        self._target_vx = 0.0
        self._target_omega = 0.0
        self._target_speed_multiplier = 1.0
        self._hold_start_time = None
        self._hold_end_time = None
        self._pending_hold_ms = 0
        self._active = False

    def close(self):
        # ensure cleanup
        self.cancel()
        super().close()

#EOF
