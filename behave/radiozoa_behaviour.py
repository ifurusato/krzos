#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-10
# modified: 2025-10-10
#

import time
from threading import Thread, Event as ThreadEvent
import asyncio
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.event import Event
from core.orientation import Orientation
from behave.behaviour import Behaviour
from hardware.radiozoa import Radiozoa
from hardware.motor_controller import MotorController

class RadiozoaBehaviour(Behaviour):
    NAME = 'radiozoa'
    RADIOZOA_PFWD_LAMBDA_NAME = "__radiozoa_pfwd"
    RADIOZOA_SFWD_LAMBDA_NAME = "__radiozoa_sfwd"
    RADIOZOA_PAFT_LAMBDA_NAME = "__radiozoa_paft"
    RADIOZOA_SAFT_LAMBDA_NAME = "__radiozoa_saft"

    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(RadiozoaBehaviour.NAME, level)
        Behaviour.__init__(self, RadiozoaBehaviour.NAME, config, message_bus, message_factory, suppressed=False, enabled=True, level=level)
        self.add_event(Event.AVOID)
        _cfg = config['kros'].get('behaviour').get('radiozoa')
        self._loop_delay_ms = _cfg.get('loop_delay_ms', 50)
        self._default_speed = _cfg.get('default_speed', 1.0)
        self._deadband_threshold = _cfg.get('deadband_threshold', 0.05)
        # multipliers for each motor
        self._pfwd_multiplier = 1.0
        self._sfwd_multiplier = 1.0
        self._paft_multiplier = 1.0
        self._saft_multiplier = 1.0
        # lambdas for each motor
        self._roam_pfwd_lambda = lambda speed: speed * self._pfwd_multiplier
        self._roam_sfwd_lambda = lambda speed: speed * self._sfwd_multiplier
        self._roam_paft_lambda = lambda speed: speed * self._paft_multiplier
        self._roam_saft_lambda = lambda speed: speed * self._saft_multiplier
        # registry lookups
        _component_registry = Component.get_registry()
        self._radiozoa = _component_registry.get(Radiozoa.NAME)
        if self._radiozoa is None:
            self._log.info(Fore.WHITE + 'creating Radiozoa…                                     xxxxxxxxxxxxxxxxxxxxxxxxxxxx ')
            self._radiozoa = Radiozoa(config, level=Level.INFO)
        else:
            self._log.info(Fore.WHITE + 'using existing Radiozoa.                                zzzzzzzzzzzzzzzzzzzzzzzzzzzzz ')
        self._motor_controller = _component_registry.get(MotorController.NAME)
        if self._motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        self._decorate_motor_controller()
        self._log.info('ready.')

    def _decorate_motor_controller(self):
        self._motor_controller.add_lambda(Orientation.PFWD, self.RADIOZOA_PFWD_LAMBDA_NAME, self._roam_pfwd_lambda)
        self._motor_controller.add_lambda(Orientation.SFWD, self.RADIOZOA_SFWD_LAMBDA_NAME, self._roam_sfwd_lambda)
        self._motor_controller.add_lambda(Orientation.PAFT, self.RADIOZOA_PAFT_LAMBDA_NAME, self._roam_paft_lambda)
        self._motor_controller.add_lambda(Orientation.SAFT, self.RADIOZOA_SAFT_LAMBDA_NAME, self._roam_saft_lambda)
        self._log.info('lambda functions added to motors.')

    @property
    def name(self):
        return RadiozoaBehaviour.NAME

    @property
    def is_ballistic(self):
        return False

    def callback(self):
        self._log.info('radiozoa behaviour callback.')

    def get_trigger_behaviour(self, event):
        return TriggerBehaviour.EXECUTE

    @property
    def trigger_event(self):
        return Event.RADIOZOA

    def execute(self, message):
        print('🧭 execute message {}.'.format(message))
        if self.suppressed:
            self._log.info(Style.DIM + 'radiozoa suppressed; message: {}'.format(message.event.label))
        else:
            self._log.info('radiozoa execute; message: {}'.format(message.event.label))
            _payload = message.payload
            _event = _payload.event
            _timestamp = self._message_bus.last_message_timestamp
            if _timestamp is None:
                self._log.info('radiozoa loop execute; no previous messages.')
            else:
                _elapsed_ms = (time.time() - _timestamp.timestamp()) * 1000.0
                self._log.info('radiozoa loop execute; message age: {:7.2f} ms'.format(_elapsed_ms))
            if self.enabled:
                self._log.info('radiozoa enabled, execution on message {}; '.format(message.name) + Fore.YELLOW + ' event: {};'.format(_event.label))
            else:
                self._log.info('radiozoa disabled, execution on message {}; '.format(message.name) + Fore.YELLOW + ' event: {};'.format(_event.label))

    async def _loop_main(self):
        self._log.info("radiozoa loop started with {}ms delay…".format(self._loop_delay_ms))
        try:
            self._accelerate()
            while not self._stop_event.is_set():
                if not self.suppressed:
                    await self._poll()
                else:
                    self._log.info(Fore.WHITE + "suppressed…")
                await asyncio.sleep(self._loop_delay_ms / 1000)
                if not self.enabled:
                    break
        except asyncio.CancelledError:
            self._log.info("radiozoa loop cancelled.")
        except Exception as e:
            self._log.error('{} encountered in radiozoa loop: {}'.format(type(e), e))
            self.disable()
        finally:
            self._log.info("radiozoa loop stopped.")
            self._decelerate()

    async def _poll(self):
        if not self.enabled:
            self._log.warning("radiozoa has been disabled.")
            return
        try:
            self._log.debug("polling…")
            distances = self._radiozoa.get_distances()
            if not distances or all(d is None or d > Radiozoa.FAR_THRESHOLD for d in distances):
                self._log.info(Fore.GREEN + "all sensors out of range or unavailable.")
                self._set_default_multipliers()
            else:
                self._update_motor_multipliers(distances)
        except Exception as e:
            self._log.error("{} thrown while polling: {}".format(type(e), e))
            self.disable()

    def _set_default_multipliers(self):
        self._pfwd_multiplier = 1.0
        self._sfwd_multiplier = 1.0
        self._paft_multiplier = 1.0
        self._saft_multiplier = 1.0

    def _update_motor_multipliers(self, distances):
        # Opposing sensor pairs: [N,S], [NE,SW], [E,W], [SE,NW]
        # Indices: N=0, NE=1, E=2, SE=3, S=4, SW=5, W=6, NW=7
        pairs = [(0,4), (1,5), (2,6), (3,7)]
        results = []
        for i,j in pairs:
            d_i = distances[i] if distances[i] is not None else Radiozoa.FAR_THRESHOLD
            d_j = distances[j] if distances[j] is not None else Radiozoa.FAR_THRESHOLD
            # Centering: positive means more space towards i, negative means more towards j
            results.append((d_i - d_j) / Radiozoa.FAR_THRESHOLD)
        # Compose motor multipliers from these four "center offset" values
        # PFWD: prefers north/NE/east/SE; PAFT: prefers south/SW/west/NW, etc.
        # Example weighting: PFWD = 1 - max(0, -results[0]), etc.
        self._pfwd_multiplier = max(0.0, min(1.0, 1.0 - max(0, -results[0])))
        self._sfwd_multiplier = max(0.0, min(1.0, 1.0 - max(0, -results[1])))
        self._paft_multiplier = max(0.0, min(1.0, 1.0 - max(0, results[0])))
        self._saft_multiplier = max(0.0, min(1.0, 1.0 - max(0, results[1])))
        # Optionally further refine using additional pairs

    def _accelerate(self):
        self._log.info("accelerate…")
        if self._motor_controller:
            self._motor_controller.accelerate(self._default_speed, enabled=lambda: not self._stop_event.is_set())

    def _decelerate(self):
        self._log.info("decelerate…")
        if self._motor_controller:
            self._motor_controller.decelerate(0.0, enabled=lambda: not self._stop_event.is_set())

    def enable(self):
        if self._motor_controller:
            self._motor_controller.enable()
        Component.enable(self)
        self._loop_instance = asyncio.new_event_loop()
        self._stop_event = ThreadEvent()
        asyncio.set_event_loop(self._loop_instance)
        self._task = self._loop_instance.create_task(self._loop_main())
        self._thread = Thread(target=self._loop_instance.run_forever, daemon=True)
        self._thread.start()
        self._log.info("radiozoa enabled.")

    def suppress(self):
        Behaviour.suppress(self)
        self._log.info("radiozoa suppressed.")

    def disable(self):
        if not self.enabled:
            self._log.warning("already disabled.")
            return
        self._log.info("radiozoa disabling…")
        self._stop_event.set()
        time.sleep(0.1)
        if self._loop_instance:
            self._loop_instance.stop()
            self._loop_instance.call_soon_threadsafe(self._shutdown)
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=1.0)
        if self._motor_controller:
            self._motor_controller.disable()
        Component.disable(self)
        self._log.info(Fore.YELLOW + 'disabled.')

    def _shutdown(self):
        self._log.info("shutting down tasks and event loop…")
        try:
            tasks = [task for task in asyncio.all_tasks(self._loop_instance) if not task.done()]
            for task in tasks:
                task.cancel()
            self._loop_instance.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))
        except Exception as e:
            self._log.error("{} raised during shutdown: {}".format(type(e), e))
        self._loop_instance.stop()
        self._loop_instance.close()
        self._log.info(Fore.YELLOW + 'shut down complete.')

    def close(self):
        super().close()
        self._log.info(Fore.YELLOW + 'closed.')

#EOF
