#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-06
# modified: 2025-10-12
#

import sys, traceback
import time
import math
from threading import Thread
from threading import Event as ThreadEvent
from math import isclose
import asyncio
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component, MissingComponentError
from core.config_loader import ConfigLoader
from core.orientation import Orientation
from core.speed import Speed
from core.event import Event
from core.queue_publisher import QueuePublisher
from core.cardinal import Cardinal
from behave.behaviour import Behaviour
from hardware.motor_controller import MotorController
from hardware.roam_sensor import RoamSensor

class Roam(Behaviour):
    NAME = 'roam'
    STOPPED = 'stopped'
    ROAM_VECTOR_NAME = "__roam_vector"
    '''
    Implements a roaming behaviour. The end result of this Behaviour is to
    provide a forward speed limit based on a fused distance value provided
    by RoamSensor (PWM + VL53L5CX). If no obstacle is perceived, the velocity
    limit is removed.

    Roam now uses a vector-based motor control, allowing for heading-based
    driving (default North, but arbitrary heading supported).
    '''
    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Roam.NAME, level)
        Behaviour.__init__(self, Roam.NAME, config, message_bus, message_factory, suppressed=True, enabled=False, level=level)
        self.add_event(Event.AVOID)
        # configuration
        _cfg = config['kros'].get('behaviour').get('roam')
        self._loop_delay_ms  = _cfg.get('loop_delay_ms', 50) # 50ms
        _cruising_speed = Speed.from_string(_cfg.get('cruising_speed'))
        self._log.info('cruising speed: {} ({:5.2f}cm/sec)'.format(_cruising_speed.label, _cruising_speed.velocity))
        self._default_speed  = _cruising_speed.proportional
        self._log.info('default speed:  {}'.format(self._default_speed))
        self._zero_tolerance = 0.2
        self._post_delay     = 500
        self._task           = None
        self._stop_delay     = _cfg.get('stop_delay_s', 3)  # seconds, configurable
        self._deadband_threshold = _cfg.get('deadband_threshold', 0.05) # for multiplier
        self._roam_distance  = -1
        # heading (in degrees, default to North)
        self._heading_degrees = 0.0

        # get necessary components from registry
        _component_registry = Component.get_registry()
        self._queue_publisher = _component_registry.get(QueuePublisher.NAME)
        if self._queue_publisher is None:
            raise MissingComponentError('queue publisher not available.')
        self._roam_sensor = None
        # MotorController instantiation (from registry only)
        self._motor_controller = _component_registry.get(MotorController.NAME)
        if self._motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        self._log.info('ready.')

    # ┈───────────── VECTOR CONTROL ──────────────┈

    def set_heading_degrees(self, degrees):
        """Set heading in degrees (0° = North, 90° = East, etc.)"""
        self._heading_degrees = float(degrees)

    def set_heading_radians(self, radians):
        """Set heading in radians (0 = North, pi/2 = East, etc.)"""
        self._heading_degrees = math.degrees(float(radians))

    def set_heading_cardinal(self, cardinal):
        """Set heading from Cardinal enum"""
        if not isinstance(cardinal, Cardinal):
            raise TypeError("cardinal must be a Cardinal enum.")
        self._heading_degrees = cardinal.degrees

    @property
    def heading_degrees(self):
        return self._heading_degrees

    @property
    def heading_radians(self):
        return math.radians(self._heading_degrees)

    @property
    def heading_vector(self):
        """Returns the unit vector for the current heading as (x, y)."""
        rad = self.heading_radians
        return (math.cos(rad), math.sin(rad))

    def _roam_vector_function(self):
        '''
        Returns a function that computes the forward vector amplitude (speed)
        based on current heading and obstacle distance.
        '''
        def vector():
            amplitude = self._compute_amplitude()
            x, y = self.heading_vector
            return (amplitude * x, amplitude * y)
        return vector

    def _compute_amplitude(self):
        '''
        Linearly scales amplitude from 1.0 (far) to 0.0 (close),
        with a deadband near zero to prevent jitter.
        '''
        if self._roam_sensor is None:
            return 0.0
        min_d = self._roam_sensor.min_distance
        max_d = self._roam_sensor.max_distance
        roam_distance = self._roam_distance

        if roam_distance is None or roam_distance >= max_d:
            multiplier = 1.0
        elif roam_distance <= min_d:
            multiplier = 0.0
        else:
            multiplier = (roam_distance - min_d) / float(max_d - min_d)
            multiplier = max(0.0, min(multiplier, 1.0))
            # apply deadband near zero
            if isclose(multiplier, 0.0, abs_tol=self._deadband_threshold):
                multiplier = 0.0
        return self._default_speed * multiplier

    def _register_roam_vector(self):
        '''
        Register the Roam vector function for each motor.
        '''
        vf = self._roam_vector_function()
        for orientation in [Orientation.PFWD, Orientation.SFWD, Orientation.PAFT, Orientation.SAFT]:
            self._motor_controller.add_vector(orientation, Roam.ROAM_VECTOR_NAME, vf, exclusive=True)
        self._log.info('vector function registered for Roam.')

    def _remove_roam_vector(self):
        '''
        Remove Roam vector functions from all motors.
        '''
        for orientation in [Orientation.PFWD, Orientation.SFWD, Orientation.PAFT, Orientation.SAFT]:
            self._motor_controller.remove_vector(orientation, Roam.ROAM_VECTOR_NAME)
        self._log.info('vector functions removed from motors.')

    # ┈───────────── BEHAVIOUR INTERFACE ──────────────┈

    @property
    def name(self):
        return Roam.NAME

    @property
    def is_ballistic(self):
        return False

    def callback(self):
        self._log.info('roam callback.')

    async def process_message(self, message):
        if message.gcd:
            raise GarbageCollectedError('cannot process message: message has been garbage collected.')
        _event = message.event
        if _event is Event.AVOID:
            if _event.value == 'suppress':
                self._log.info(Fore.WHITE + "processing AVOID message with event: '{}'; value: {}".format(_event.name, _event.value))
                self.suppress()
            else:
                self._log.info(Fore.BLUE + "ignored AVOID message with event: '{}'; value: {}".format(_event.name, _event.value))
        else:
            self._log.info(Fore.WHITE + "processing {} message with event: '{}'; value: {}".format(message.name, _event.name, _event.value))
        await Behaviour.process_message(self, message)

    def execute(self, message):
        if self.suppressed:
            self._log.info(Style.DIM + 'roam suppressed; message: {}'.format(message.event.label))
        else:
            self._log.info('roam execute; message: {}'.format(message.event.label))
            _payload = message.payload
            _event   = _payload.event
            _timestamp = self._message_bus.last_message_timestamp
            if _timestamp is None:
                self._log.info('roam loop execute; no previous messages.')
            else:
                _elapsed_ms = (time.time() - _timestamp.timestamp()) * 1000.0
                self._log.info('roam loop execute; message age: {:7.2f} ms'.format(_elapsed_ms))
            if self.enabled:
                self._log.info('roam enabled, execution on message {}; '.format(message.name) + Fore.YELLOW + ' event: {};'.format(_event.label))
            else:
                self._log.info('roam disabled, execution on message {}; '.format(message.name) + Fore.YELLOW + ' event: {};'.format(_event.label))

    async def _loop_main(self):
        self._log.info("roam loop started with {}ms delay…".format(self._loop_delay_ms))
        try:
            if not self.suppressed:
                self._accelerate()
            while not self._stop_event.is_set():
                if not self.suppressed:
                    await self._poll()
                else:
                    self._log.debug("suppressed…")
                    pass
                await asyncio.sleep(self._loop_delay_ms / 1000)
                if not self.enabled:
                    break
        except asyncio.CancelledError:
            self._log.info("roam loop cancelled.")
        except Exception as e:
            self._log.error('{} encountered in roam loop: {}\n{}'.format(type(e), e, traceback.format_exc()))
            self.disable()
        finally:
            if not self.suppressed:
                self._decelerate()
            self._log.info("roam loop stopped.")

    async def _poll(self):
        if not self.enabled:
            self._log.warning("roam has been disabled.")
            return
        try:
            # get fused distance from RoamSensor
            roam_distance = self.get_roam_distance()
            if roam_distance is None:
                self._log.warning(Fore.WHITE + "no distance available: ignored.")
            elif roam_distance > 0.0:
                self._log.debug(Fore.WHITE + "roam distance: {:4.2f}".format(roam_distance))
            else:
                self._log.warning(Fore.WHITE + "roam distance not set.")
        except Exception as e:
            self._log.error("{} thrown while polling: {}\n{}".format(type(e), e, traceback.format_exc()))
            self.disable()

    @property
    def roam_distance(self):
        '''
        Return the last-measured roam distance, -1 if not set. This does not poll the sensor.
        '''
        return self._roam_distance

    def get_roam_distance(self):
        '''
        Returns the fused front distance from RoamSensor.
        '''
        if not self.enabled:
            self._log.warning('get_roam_distance called while Roam is not enabled.')
            return None
        distance = self._roam_sensor.get_distance()
        self._roam_distance = distance if distance is not None else -1
        return self._roam_distance

    def _accelerate(self):
        self._log.info("accelerate…")
        if self._motor_controller:
            self._motor_controller.accelerate(self._default_speed, enabled=lambda: not self._stop_event.is_set())

    def _decelerate(self):
        self._log.info("decelerate…")
        if self._motor_controller:
            self._motor_controller.decelerate(0.0, enabled=lambda: not self._stop_event.is_set())

    def _stop(self):
        self._log.info("stop…")
        if self._motor_controller:
            self._motor_controller.stop()

    def _handle_stoppage(self):
        _message = self.message_factory.create_message(Event.ROAM, Roam.STOPPED)
        self._queue_publisher.put(_message)
        self._log.info("published ROAM message: {}".format(_message))

    def enable(self):
        if self.enabled:
            self._log.debug("already enabled.")
            return
        # RoamSensor instantiation
        _component_registry = Component.get_registry()
        self._roam_sensor = _component_registry.get(RoamSensor.NAME)
        if self._roam_sensor is None:
            self._roam_sensor = RoamSensor(self._config, level=Level.INFO)
        if self._motor_controller:
            self._motor_controller.enable()
            self._register_roam_vector()
        Component.enable(self)
        self._loop_instance = asyncio.new_event_loop()
        self._stop_event = ThreadEvent()
        asyncio.set_event_loop(self._loop_instance)
        self._task = self._loop_instance.create_task(self._loop_main())
        self._thread = Thread(target=self._loop_instance.run_forever, daemon=True)
        self._thread.start()
        self._log.info("roam enabled.")

    def suppress(self):
        Behaviour.suppress(self)
        self._remove_roam_vector()
        self._log.info("roam suppressed.")

    def disable(self):
        if not self.enabled:
            self._log.debug("already disabled.")
            return
        self._log.info("disabling roam…")
        # 1. Signal coroutine to exit.
        self._stop_event.set()
        time.sleep(0.1)
        if self._loop_instance:
            self._loop_instance.call_soon_threadsafe(self._shutdown)
            self._loop_instance.call_soon_threadsafe(self._loop_instance.stop)
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=2.0)
            self._loop_instance.close()
        if self._motor_controller:
            self._remove_roam_vector()
            self._motor_controller.disable()
        Component.disable(self)
        self._log.info(Fore.YELLOW + 'disabled.')

    def _shutdown(self):
        self._log.info("shutting down tasks and event loop…")
        try:
            tasks = [task for task in asyncio.all_tasks(self._loop_instance) if not task.done()]
            for task in tasks:
                task.cancel()
            if tasks:
                self._loop_instance.run_until_complete(
                    asyncio.gather(*tasks, return_exceptions=True)
                )
        except Exception as e:
            self._log.error("{} raised during shutdown: {}".format(type(e), e))
        self._log.info(Fore.YELLOW + 'shut down complete.')

    def close(self):
        super().close()
        self._log.info(Fore.YELLOW + 'closed.')

#EOF
