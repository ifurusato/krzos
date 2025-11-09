#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-29
# modified: 2025-11-09

import time
import asyncio
import numpy as np
import itertools
from threading import Thread, Event as ThreadEvent
from colorama import init, Fore, Style
init()

from core.logger import Level
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
        _cfg = config['kros'].get('behaviour').get(AsyncBehaviour.NAME)
        self._poll_delay_ms   = _cfg.get('poll_delay_ms')
        self._poll_delay_sec  = self._poll_delay_ms / 1000.0
        self._log.info("poll delay: {}ms".format(self._poll_delay_ms))
        self._intent_vector   = (0.0, 0.0, 0.0)
        self._priority        = 0.3  # default priority
        self._intent_vector_registered = False
        self._loop_instance   = None
        self._thread          = None
        self._stop_event      = ThreadEvent()
        self._counter = itertools.count()
        if self._motor_controller:
            # get motor objects
            self._motor_pfwd = self._motor_controller.get_motor(Orientation.PFWD)
            self._motor_sfwd = self._motor_controller.get_motor(Orientation.SFWD)
            self._motor_paft = self._motor_controller.get_motor(Orientation.PAFT)
            self._motor_saft = self._motor_controller.get_motor(Orientation.SAFT)
            # geometry configuration
            velocity = self._motor_pfwd.get_velocity()
            self._steps_per_rotation = velocity.steps_per_rotation
            self._wheel_diameter_mm = velocity._wheel_diameter
            self._wheel_track_mm = config['kros']['geometry']['wheel_track']
            wheel_circumference_cm = np.pi * self._wheel_diameter_mm / 10.0
            rotation_circle_cm = np.pi * self._wheel_track_mm / 10.0
            steps_per_degree_theoretical = (rotation_circle_cm / wheel_circumference_cm * self._steps_per_rotation) / 360.0
            self._steps_per_degree = _cfg.get('steps_per_degree', steps_per_degree_theoretical)
            self._log.info('steps_per_degree set to: {}'.format(self._steps_per_degree))
        self._log.info('ready.')

    @property
    def priority(self):
        '''
        Returns the current priority for this behaviour.
        Subclasses can override to provide dynamic priority.
        '''
        return self._priority

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
        self._log.info('intent vector lambda registered with motor controller.')

    def _remove_intent_vector(self):
        '''
        Unregister this behaviour's intent vector from the motor controller.
        '''
        if not self._motor_controller:
            return
        if not self._intent_vector_registered:
            self._log.debug('intent vector not registered; nothing to remove.')
            return
        self._motor_controller.remove_intent_vector(self.name)
        self._intent_vector_registered = False
        self._log.info('intent vector lambda removed from motor controller.')

    def clear_intent_vector(self):
        '''
        Zero the intent vector.
        '''
        self._intent_vector = (0.0, 0.0, 0.0)

    def enable(self):
        '''
        Enable the behaviour, making it available for use.
        Does not start the async loop - that happens on release().
        '''
        if self.enabled:
            self._log.debug("already enabled.")
            return
        Behaviour.enable(self)
        self._log.info('enabled (suppressed, waiting for release).')

    def _start_loop(self):
        '''
        Start the async event loop and register intent vector.
        Called by release() when the behaviour is activated.
        '''
        if self._loop_instance and self._loop_instance.is_running():
            self._log.warning('loop already running.')
            return
        self._log.info('starting async loop…')
        self._stop_event.clear()
        self._loop_instance = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop_instance)
        self._task = self._loop_instance.create_task(self._loop_main())
        self._thread = Thread(target=self._loop_instance.run_forever, daemon=True)
        self._thread.start()
        if self._motor_controller and not self._intent_vector_registered:
            self._register_intent_vector()
        self._log.info('async loop started.')

    def suppress(self):
        '''
        Suppresses the behaviour, stopping the loop and removing the intent vector.
        '''
        if self.suppressed:
            self._log.debug("already suppressed.")
            return
        Behaviour.suppress(self)
        self._remove_intent_vector()
        # stop the loop when suppressed
        if self._loop_instance:
            self._stop_loop()
        self._log.info('suppressed.')

    def release(self):
        '''
        Releases suppression of the behaviour, starting the loop and registering intent vector.
        '''
        if not self.suppressed:
            self._log.debug("already released.")
            return
        if not self.enabled:
            self._log.warning('cannot release: behaviour is disabled.')
            return
        Behaviour.release(self)
        # Start the loop when released
        self._start_loop()
        self._log.info("released.")

    def disable(self):
        '''
        Permanently disable the behaviour, stopping the loop.
        '''
        if not self.enabled:
            self._log.debug("already disabled.")
            return
        self._log.info("disabling…")
        self.clear_intent_vector()
        self._stop_event.set()
        # stop the loop if it's running
        if self._loop_instance:
            self._stop_loop()
        Behaviour.disable(self)
        self._log.info('disabled.')

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
        '''
        pass

    async def _loop_main(self):
        '''
        Main async loop that polls the behaviour at regular intervals.
        This loop only runs when the behaviour is released (not suppressed).
        '''
        self._log.info("async loop started with {}ms delay…".format(self._poll_delay_ms))
        try:
            self.start_loop_action()
            while not self._stop_event.is_set():
                await self._poll()
                await asyncio.sleep(self._poll_delay_sec)
                if not self.enabled:
                    break
        except asyncio.CancelledError:
            self._log.info("async loop cancelled.")
        except Exception as e:
            self._log.error('{} encountered in async loop: {}'.format(type(e), e))
            self.disable()
        finally:
            self.stop_loop_action()
            self._log.info("async loop stopped.")

    def _stop_loop(self):
        '''
        Stop the async event loop and clean up resources.
        '''
        if not self._loop_instance:
            self._log.debug('no loop instance to stop.')
            return
            
        self._log.info("shutting down event loop…")
        try:
            self._loop_instance.stop()
            self._loop_instance.call_soon_threadsafe(self._shutdown)
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=1.0)
            self._loop_instance.run_until_complete(self._loop_instance.shutdown_asyncgens())
            if self._loop_instance.is_running():
                time.sleep(1)
            self._loop_instance.close()
        except Exception as e:
            self._log.error("{} raised stopping loop: {}".format(type(e), e))
        finally:
            self._loop_instance = None
            self._log.info('event loop shut down.')

    def _shutdown(self):
        '''
        Cancel all pending tasks in the event loop.
        '''
        self._log.info("shutting down tasks…")
        try:
            tasks = [task for task in asyncio.all_tasks(self._loop_instance) if not task.done()]
            if len(tasks) > 0:
                for task in tasks:
                    task.cancel()
                self._loop_instance.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))
        except Exception as e:
            self._log.error("{} raised during shutdown: {}".format(type(e), e))
        self._log.info('task shut down complete.')

    def close(self):
        '''
        Permanently close the behaviour.
        '''
        if not self.closed:
            self.disable()
            Behaviour.close(self)
            self._log.info('closed.')

#EOF
