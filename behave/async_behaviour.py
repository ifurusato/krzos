#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#           
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-29
# modified: 2025-10-29

import time
import asyncio
from threading import Thread, Event as ThreadEvent
from colorama import init, Fore, Style
init()

from core.logger import Level
from behave.behaviour import Behaviour

class AsyncBehaviour(Behaviour):
    NAME = 'async_behaviour'
    '''
    Provides a looping asyncio basis for a Behaviour.
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
        self._log.info('ready.')

    @property
    def priority(self):
        '''
        Returns the current priority for this behaviour.
        Subclasses can override to provide dynamic priority.
        '''
        return self._priority

    def _register_intent_vector(self):
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
        self._log.warning('removing Roam intent vector from motor controller.')
        self._motor_controller.remove_intent_vector(self.name)
        self._intent_vector_registered = False
        self._log.info('intent vector lambda removed from motor controller.')

    def clear_intent_vector(self):
        '''
        Zero the intent vector.
        '''
        self._intent_vector = (0.0, 0.0, 0.0)

    def enable(self):
        Behaviour.enable(self)
        self._loop_instance = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop_instance)
        self._task = self._loop_instance.create_task(self._loop_main())
        self._thread = Thread(target=self._loop_instance.run_forever, daemon=True)
        self._thread.start()
        if self._motor_controller and not self._intent_vector_registered:
            self._register_intent_vector()
        self._log.info('enabled.')

    def suppress(self):
        '''
        Suppresses the behaviour, removing the intent vector.
        '''
        Behaviour.suppress(self)
        self._remove_intent_vector()
        self._log.info("radiozoa suppressed.")

    def release(self):
        '''
        Releases suppression of the behaviour, re-registering intent vector.
        '''
        Behaviour.release(self)
        if not self._intent_vector_registered:
            self._register_intent_vector()
        self._log.info("radiozoa released.")

    def disable(self):
        if not self.enabled:
            self._log.info("already disabled.")
            return
        self._log.info("disabling…")
        self.clear_intent_vector()
        self._stop_event.set()
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
        self._log.info("roam loop started with {}ms delay…".format(self._poll_delay_ms))
        try:
            if not self.suppressed:
                self.start_loop_action()
            while not self._stop_event.is_set():
                if not self.suppressed:
                    await self._poll()
                else:
                    self._log.info(Fore.WHITE + "suppressed…")
                await asyncio.sleep(self._poll_delay_sec)
                if not self.enabled:
                    break
        except asyncio.CancelledError:
            self._log.info("roam loop cancelled.")
        except Exception as e:
            self._log.error('{} encountered in roam loop: {}'.format(type(e), e))
            self.disable()
        finally:
            if not self.suppressed:
                self.stop_loop_action()
            self._log.info("roam loop stopped.")

    def _stop_loop(self):
        if self._loop_instance:
            self._log.info(Fore.YELLOW + "shutting down event loop…")
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

    def _shutdown(self):
        self._log.info(Fore.YELLOW + "shutting down tasks…")
        try:
            tasks = [task for task in asyncio.all_tasks(self._loop_instance) if not task.done()]
            if len(tasks) > 0:
                for task in tasks:
                    task.cancel()
                self._loop_instance.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))
        except Exception as e:
            self._log.error("{} raised during shutdown: {}".format(type(e), e))
        self._log.info(Fore.YELLOW + 'task shut down complete.')

    def close(self):
        Behaviour.close(self)
        self._log.info(Fore.YELLOW + 'closed.')

#EOF
