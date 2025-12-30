#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-10-29
# modified: 2025-11-29

import time
import asyncio
import numpy as np
import itertools
from datetime import datetime as dt
from threading import current_thread, Thread, Event as ThreadEvent
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.event import Event
from core.component import Component
from core.orientation import Orientation
from behave.behaviour import Behaviour

class AsyncBehaviour(Behaviour):
    NAME = 'async_behaviour'
    '''
    Provides a looping asyncio basis for a Behaviour.

    Lifecycle:
    - enable()   makes the behaviour available but does not start the loop
    - release()  starts the async loop and registers the intent vector
    - suppress() stops the async loop and unregisters the intent vector
    - disable()  permanently shuts down the behaviour
    '''
    def __init__(self, log_or_name=None, config=None, message_bus=None, message_factory=None, motor_controller=None, level=Level.INFO):
        Behaviour.__init__(self, log_or_name, config, message_bus, message_factory, suppressed=True, enabled=False, level=level)
        self._motor_controller = motor_controller
        # data logger?
        self._data_log = None
        _data_logging = config['kros'].get('application').get('data_logging')
        if _data_logging:
            self._log.data(Fore.GREEN + 'data logging is active.')
            self._data_log = Logger('{}'.format(self.NAME), log_to_file=True, data_logger=True, level=Level.INFO)
            self._data_log.data('START')
        _cfg = config['kros'].get('behaviour').get(AsyncBehaviour.NAME)
        self._poll_delay_ms   = _cfg.get('poll_delay_ms')
        self._poll_delay_sec  = self._poll_delay_ms / 1000.0
        self._log.info("poll delay: {}ms".format(self._poll_delay_ms))
        _ramp_down_duration_sec = _cfg.get('ramp_down_duration_sec', 1.0)
        self._ramp_down_step = 1.0 / (_ramp_down_duration_sec / self._poll_delay_sec) if _ramp_down_duration_sec > 0 else 1.0
        self._log.info("blind mode ramp down step: {:.4f}".format(self._ramp_down_step))
        # state flags
        self._intent_vector     = (0.0, 0.0, 0.0)
        self._use_dynamic_priority = False
        self._priority          = 0.3  # default priority
        self._intent_vector_registered = False
        self._hold_at_zero      = False
        self._intent_multiplier = 1.0
        # event loop
        self._loop_instance   = None
        self._thread          = None
        self._stop_event      = ThreadEvent()
        self._counter = itertools.count()
        self.add_event(Event.BLIND)
        if self._motor_controller:
            # get motor objects
            self._motor_pfwd = self._motor_controller.get_motor(Orientation.PFWD)
            self._motor_sfwd = self._motor_controller.get_motor(Orientation.SFWD)
            self._motor_paft = self._motor_controller.get_motor(Orientation.PAFT)
            self._motor_saft = self._motor_controller.get_motor(Orientation.SAFT)
            # geometry configuration
            _cfg = config['kros']['geometry']
            self._steps_per_rotation = _cfg.get('steps_per_rotation')
            self._wheel_diameter     = _cfg.get('wheel_diameter')
            self._wheel_track_mm     = _cfg.get('wheel_track')
            wheel_circumference_cm = np.pi * self._wheel_diameter / 10.0
            rotation_circle_cm = np.pi * self._wheel_track_mm / 10.0
            steps_per_degree_theoretical = (rotation_circle_cm / wheel_circumference_cm * self._steps_per_rotation) / 360.0
            self._steps_per_degree = _cfg.get('steps_per_degree', steps_per_degree_theoretical)
            self._log.info('set to: {:4.2f} steps/degree'.format(self._steps_per_degree))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def priority(self):
        '''
        Returns the current priority for this behaviour.
        Subclasses can override to provide dynamic priority.
        '''
        return self._priority

    @property
    def using_dynamic_priority(self):
        '''
        Returns True if this behaviour is using dynamic priority.
        '''
        return self._use_dynamic_priority

    def _is_zero(self, vector):
        return vector == (0.0, 0.0, 0.0)

    def _is_almost_zero(self, vector):
        return all(math.isclose(v, 0.0, abs_tol=0.001) for v in vector)

    def _register_intent_vector(self):
        '''
        Register this behaviour's intent vector with the motor controller.
        '''
        if not self._motor_controller:
            return
        if self._intent_vector_registered:
            self._log.warning('intent vector already registered with motor controller.')
            return
        self._motor_controller.add_intent_vector(
            self.name,
            lambda: self._intent_vector,
            lambda: self.priority
        )
        self._intent_vector_registered = True
        self._log.debug('intent vector lambda registered with motor controller.')

    def _remove_intent_vector(self):
        '''
        Unregister this Behaviour's intent vector from the motor controller.
        '''
        if not self._motor_controller:
            return
        if not self._intent_vector_registered:
            self._log.debug('intent vector not registered; nothing to remove.')
            return
        self._motor_controller.remove_intent_vector(self.name)
        self._intent_vector_registered = False
        self._log.debug('intent vector lambda removed from motor controller.')

    def set_intent_vector(self, vx, vy, omega):
        '''
        Sets the intent vector directly.
        Should only be called from _loop_main() after multiplier is applied.
        Behaviors should return values from _poll() instead.
        '''
        self._intent_vector = (vx, vy, omega)

    def clear_intent_vector(self):
        '''
        Zero the intent vector.
        '''
        self._intent_vector = (0.0, 0.0, 0.0)
        if self._data_log:
            self._data_log.data(0.0, 0.0, 0.0)

    def enable(self):
        '''
        Enable the Behaviour, making it available for use.
        Does not start the async loop - that happens on release().
        '''
        if not self.enabled:
            super().enable()
            self._log.debug('enabled.')
        else:
            self._log.warning("already enabled.")

    async def process_message(self, message):
        '''
        Overrides the method in Subscriber to handle BLIND events.
        '''
#       self._log.debug('processing message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(message.event.label))
        if message.event == Event.BLIND:
            is_blind = message.value
            if is_blind:
                if not self._hold_at_zero:
                    self._log.warning("BLIND received: engaging hold-at-zero and ramping down intent…")
                    self._hold_at_zero = True
                    # trigger the brake
                    if self._motor_controller and not self._motor_controller.braking_active:
                        self._log.info("triggering system brake…")
                        self._motor_controller.brake()
            else: # is not blind
                if self._hold_at_zero:
                    self._log.info("BLIND (exit) received: disengaging hold-at-zero…")
                    self._hold_at_zero = False
                    # first responder releases the brake
                    if self._motor_controller and self._motor_controller.is_braked:
                        self._log.info("first responder: triggering brake release.")
                        self._motor_controller.release_brake()
#           message.process(self) # let Behaviour do this
        else:
            pass #
        await super().process_message(message)

    def _start_loop(self):
        '''
        Start the async event loop and register intent vector.
        Called by release() when the Behaviour is activated.
        '''
        if self._loop_instance and self._loop_instance.is_running():
            self._log.warning('loop already running.')
            return
        self._log.debug('starting async loop…')
        self._stop_event.clear()
        self._loop_instance = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop_instance)
        self._task = self._loop_instance.create_task(self._loop_main())
        self._thread = Thread(target=self._loop_instance.run_forever, daemon=True)
        self._thread.start()
        if self._motor_controller and not self._intent_vector_registered:
            self._register_intent_vector()
        self._log.debug('async loop started.')

    def suppress(self):
        '''
        Suppresses the Behaviour. Note that this does not stop the loop but
        instead relies on the flags inside the loop to handle correct polling.
        '''
        if not self.enabled:
            self._log.warning('cannot suppress: behaviour is disabled.')
        elif not self.suppressed:
            self._log.debug("suppressing…")
            super().suppress()
#           self._remove_intent_vector()
            # stop the loop when suppressed
#           if self._loop_instance:
#               self._stop_loop()
            self._log.info('suppressed.')
        else:
            self._log.warning("already suppressed.")

    def release(self):
        '''
        Releases the Behaviour. Note that this does not stop the loop but
        instead relies on the flags inside the loop to handle correct polling.
        '''
        if not self.enabled:
            self._log.warning('cannot release: behaviour is disabled.')
        elif not self.released:
            self._log.debug("releasing…")
            super().release()
            # start the loop when released for the first time
            if not self._loop_instance:
                self._start_loop()
            self._log.info("released.")
        else:
            self._log.warning("already released.")

    def start_loop_action(self):
        '''
        Called at the beginning of the main loop. To be overridden by subclasses.
        '''
        pass

    def stop_loop_action(self):
        '''
        Called at the end of the main loop. To be overridden by subclasses.
        '''
        pass

    async def _poll(self):
        '''
        The main loop poll. To be overridden by subclasses.
        Should return (vx, vy, omega) tuple.
        '''
        return (0.0, 0.0, 0.0)

    async def _loop_main(self):
        '''
        Main async loop that polls the Behaviour at regular intervals.

        The loop runs continuously, but reacts to the suppressed/released
        state of the Behaviour. Disabling the Behaviour will exit the loop.

        Behaviors return their desired intent vector from _poll().

        The safety multiplier is always applied (normally 1.0, ramps to 0.0 during BLIND).
        Intent vector is set only once per iteration, after multiplier application.
        Data logging occurs after multiplier, before motor controller reads the value.
        '''
        self._log.debug("async loop started with {}ms delay…".format(self._poll_delay_ms))
        try:
            self.start_loop_action()
            while not self._stop_event.is_set():
                if not self.enabled:
                    self._log.debug("behaviour disabled during loop, exiting…")
                    break
                if not self._behaviour_manager.is_ballistic() and self.has_toggle_assignment():
                    # we only alter suppressed states if nobody has gone ballistic
                    if self.suppressed and self.is_released_by_toggle():
                        self._log.info('releasing…')
                        self.release()
                    elif self.released and not self.is_released_by_toggle():
                        self._log.info('suppressing…')
                        self.suppress()
                if self.suppressed:
                    self.clear_intent_vector()
                else:
                    # get desired intent vector from behavior
                    vx, vy, omega = await self._poll()
                    # update multiplier based on blind state
                    if self._hold_at_zero:
                        # in blind mode, ramp down the multiplier
                        if self._intent_multiplier > 0.0:
                            self._intent_multiplier = max(0.0, self._intent_multiplier - self._ramp_down_step)
                    else:
                        # not in blind mode, ensure multiplier is at 1.0
                        self._intent_multiplier = 1.0
                    # apply multiplier and set intent vector ONCE
                    self._intent_vector = (
                        vx * self._intent_multiplier,
                        vy * self._intent_multiplier,
                        omega * self._intent_multiplier
                    )
                    # log after multiplier applied, before motor controller reads it
                    if self._data_log:
                        self._data_log.data(
                            '{:.3f}'.format(self._intent_vector[0]),
                            '{:.3f}'.format(self._intent_vector[1]),
                            '{:.3f}'.format(self._intent_vector[2])
                        )
                await asyncio.sleep(self._poll_delay_sec)
                if not self.enabled:
                    self._log.debug("behaviour disabled during loop, exiting…")
                    break

        except asyncio.CancelledError:
            self._log.debug("async loop cancelled.")
        except Exception as e:
            self._log.error('{} encountered in async loop: {}'.format(type(e), e))
            self.disable()
        finally:
            self.stop_loop_action()
            self._log.debug("async loop stopped.")

    def _stop_loop(self):
        '''
        Stop the async event loop and clean up resources.
        '''
        if not self._loop_instance:
            self._log.debug('no loop instance to stop.')
            return
        self._stop_event.set()
        time.sleep(0.1) # give it a moment
        self._log.info(Fore.MAGENTA + 'stopping {} polling loop…'.format(self.name))
        _poll_stop_timeout_ms = 15
        _start_time = dt.now()
        try:
            # check if we're being called from the loop's own thread
            _is_loop_thread = (self._thread and self._thread == current_thread())
            if not _is_loop_thread:
                # safe to use call_soon_threadsafe and join from external thread
                self._loop_instance.call_soon_threadsafe(self._shutdown)
                # wait for thread to finish
                if self._thread and self._thread.is_alive():
                    self._thread.join(timeout=_poll_stop_timeout_ms / 1000)
                # stop the loop if it's running
                if self._loop_instance and self._loop_instance.is_running():
                    self._loop_instance.call_soon_threadsafe(self._loop_instance.stop)
                    time.sleep(0.1)
            else:
                # we're being called from within the loop thread itself
                # cannot join our own thread, just stop the loop
                self._log.warning('_stop_loop called from loop thread - cannot join self')
                if self._loop_instance and self._loop_instance.is_running():
                    self._loop_instance.stop()
            # now safe to close
            if self._loop_instance and not self._loop_instance.is_closed():
                self._loop_instance.close()
                self._log.debug('{} polling loop closed.'.format(self.name))
            else:
                self._log.warning('loop already closed')
        except Exception as e:
            self._log.error("{} raised stopping loop: {}".format(type(e), e))
        finally:
            self._loop_instance = None
            _elapsed_ms = round((dt.now() - _start_time).total_seconds() * 1000.0)
            self._log.info(Fore.MAGENTA + '{} polling loop stopped; {}ms elapsed.'.format(self.name, _elapsed_ms))

    def _shutdown(self):
        '''
        Cancel all pending tasks in the event loop.
        '''
        self._log.debug("shutting down tasks…")
        try:
            tasks = [task for task in asyncio.all_tasks(self._loop_instance) if not task.done()]
            if len(tasks) > 0:
                for task in tasks:
                    task.cancel()
                if self._loop_instance:
                    self._loop_instance.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))
                else:
                    self._log.warning('no loop instance .'.format(self.name))
        except Exception as e:
            self._log.error("{} raised during shutdown: {}".format(type(e), e))
        self._log.debug('task shutdown complete.')

    def disable(self):
        '''
        Permanently disable the Behaviour, stopping the loop.
        '''
        if self.enabled:
            self._log.debug("disabling…")
            self.clear_intent_vector()
            # stop the loop if it's running
            if self._loop_instance:
                self._stop_loop()
            super().disable()
            self._log.debug('disabled.')
        else:
            self._log.warning("already disabled.")

    def close(self):
        '''
        Permanently close the Behaviour.
        '''
        if not self.closed:
            if self._data_log:
                self._data_log.data("END")
                # note: it's not up to us to close the shared data logger
            super().close()
            self._log.debug('closed.')
        else:
            self._log.warning("already closed.")

#EOF
