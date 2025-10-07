#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-06
# modified: 2025-05-06
#

import sys, traceback
import time
import numpy as np
from threading import Thread
from threading import Event as ThreadEvent # differentiate from our Event
from math import isclose as isclose
import asyncio
from datetime import datetime, timedelta
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.config_loader import ConfigLoader
from core.event import Event, Group
from core.queue_publisher import QueuePublisher
from core.logger import Logger, Level
from core.orientation import Orientation
from core.speed import Speed
from core.subscriber import Subscriber
from behave.behaviour import Behaviour
from hardware.motor_controller import MotorController
from hardware.vl53l5cx_sensor import Vl53l5cxSensor

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Roam(Behaviour):
    STOPPED = 'stopped'
    '''
    Implements a roaming behaviour. The end result of this Behaviour is to
    provide a forward speed limit for both motors based on a distance value
    provided by the center infrared sensor, i.e., the distance to an obstacle
    in cm. If no obstacle is perceived within the range of the sensor, the
    velocity limit is removed.

    Because we only know how far the obstacle is based on incoming events,
    if we haven't seen an event in awhile we may assume there is nothing
    there and start moving again, at "cruising" speed. But we need to wait
    a bit after reacting to an obstacle before attempting to start moving
    again.

    The Roam behaviour is by default suppressed.
    '''
    def __init__(self, config=None, message_bus=None, message_factory=None, motor_controller=None, level=Level.INFO):
        '''
        :param config:              the application configuration
        :param message_bus:         the asynchronous message bus
        :param message_factory:     the factory for creating messages
        :param motor_controller:    the motor controller
        :param level:               the log level
        '''
        self._log = Logger('roam', level)
        if motor_controller is None:
            raise ValueError('no motor controller provided.')
        Behaviour.__init__(self, 'roam', config, message_bus, message_factory, suppressed=False, enabled=True, level=level)
        self.add_event(Event.AVOID)
        # configuration
        _cfg = config['kros'].get('behaviour').get('roam')
        self._loop_delay_ms  = _cfg.get('loop_delay_ms', 50) # 50ms
        _cruising_speed = Speed.from_string(_cfg.get('cruising_speed'))
        _vl53_cfg = config['kros'].get('hardware').get('vl53l5cx')
        self._cols = _vl53_cfg.get('cols', 8)
        self._rows = _vl53_cfg.get('rows', 8)
        # variables
        self._log.info(Style.BRIGHT + 'cruising speed: \t{} ({:5.2f}cm/sec)'.format(_cruising_speed.label, _cruising_speed.velocity))
        self._default_speed  = _cruising_speed.proportional
        self._zero_tolerance = 0.2
        self._log.info(Style.BRIGHT + 'default speed: \t{}'.format(self._default_speed))
        self._post_delay     = 500
        self._task           = None 
        _component_registry  = Component.get_registry()
        self._queue_publisher = _component_registry.get(QueuePublisher.NAME)
        if self._queue_publisher is None:
            raise MissingComponentError('queue publisher not available.')
        # see if it's in the ComponentRegistry
        _component_registry = Component.get_registry()
        self._vl53l5cx = _component_registry.get(Vl53l5cxSensor.NAME)
        if self._vl53l5cx is None:
            # otherwise we make one
            self._vl53l5cx = Vl53l5cxSensor(config, level=Level.INFO)
        self._motor_controller = _component_registry.get(MotorController.NAME)
        if self._motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return Roam.CLASS_NAME

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_ballistic(self):
        return False

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def callback(self):
        self._log.info('roam callback.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return 'roam'

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_trigger_behaviour(self, event):
        '''
        Indicates what the Behaviour should do when triggered, which is to
        execute the behaviour.
        '''
        return TriggerBehaviour.EXECUTE

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def trigger_event(self):
        '''
        This returns the event used to enable/disable the behaviour manually.

        Note: the priority of this event determines the priority of this
        Behaviour when compared to other Behaviours.
        '''
        return Event.ROAM

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def process_message(self, message):
        '''
        Process the message. If it's not an IDLE message this indicates activity.

        A Subscriber method.

        :param message:  the message to process.
        '''
        if message.gcd:
            raise GarbageCollectedError('cannot process message: message has been garbage collected.')
        _event = message.event
        if _event is Event.AVOID:
            if _event.value == 'suppress':
                self._log.info(Fore.WHITE + "🍀 processing AVOID message with event: '{}'; value: {}".format(_event.name, _event.value))
                self.suppress()
                # TODO what to do?
            else:
                self._log.info(Fore.BLUE + "ignored AVOID message with event: '{}'; value: {}".format(_event.name, _event.value))
        else:
            self._log.info(Fore.WHITE + "🍀 processing {} message with event: '{}'; value: {}".format(_message.name, _event.name, _event.value))
        await Subscriber.process_message(self, message)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def execute(self, message):
        '''
        The method called upon each loop iteration.

        :param message:  an optional Message passed along by the message bus
        '''
        print('🍀 execute message {}.'.format(message))
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
                _elapsed_ms = (dt.now() - _timestamp).total_seconds() * 1000.0
                self._log.info('roam loop execute; {}'.format(Util.get_formatted_time('message age:', _elapsed_ms)))
            if self.enabled:
                self._log.info('roam enabled, execution on message {}; '.format(message.name) + Fore.YELLOW + ' event: {};'.format(_event.label))
            else:
                self._log.info('roam disabled, execution on message {}; '.format(message.name) + Fore.YELLOW + ' event: {};'.format(_event.label))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _loop_main(self):
        self._log.info("roam loop started with {}ms delay…".format(self._loop_delay_ms))
        try:
            self._accelerate()
            if self._motor_controller:
                self._motor_controller.set_differential_speeds(self._default_speed, self._default_speed)
            while not self._stop_event.is_set():
                if not self.suppressed:
                    await self._poll()
                else:
                    self._log.info(Fore.WHITE + "suppressed…")
                await asyncio.sleep(self._loop_delay_ms / 1000)
                # add a safe exit condition for testing
                if not self.enabled:
                    break
        except asyncio.CancelledError:
            self._log.info("roam loop cancelled.")
        except Exception as e:
            self._log.error('{} encountered in roam loop: {}\n{}'.format(type(e), e, traceback.format_exc()))
            self.disable()
        finally:
            self._log.info("roam loop stopped.")
            self._decelerate()
            self._stop()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _poll(self):
        if not self.enabled:
            self._log.warning("roam has been disabled.")
            return
        try:
            self._log.debug("polling…")
            # get weighted averages for each motor
            roam_distance = self.get_roam_distance()
            if roam_distance is None:
                self._log.warning(Fore.WHITE + "🐭 nada.")
                pass
            elif roam_distance > 0.0:
                self._log.info(Fore.WHITE + "🐭 roam distance: {:4.2f}".format(roam_distance))
                # TODO
                pass
            else:
                self._log.warning(Fore.WHITE + "🐭 roam distance not set.")
                pass

#           if isclose(port_speed, 0.0, abs_tol=self._zero_tolerance) or isclose(stbd_speed, 0.0, abs_tol=self._zero_tolerance):
#               # there's no point in setting a speed the motors cannot fulfill
#               self._motor_controller.set_differential_speeds(0.0, 0.0, save=False)
#               self._handle_stoppage()
#           else:
#               self._motor_controller.set_differential_speeds(port_speed, stbd_speed, save=False)

        except Exception as e:
            self._log.error("{} thrown while polling: {}\n{}".format(type(e), e, traceback.format_exc()))
            self.disable()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def roam_distance(self):
        '''
        Return the last-measured roam distance, -1.0 if it has not been set. This
        does not poll the sensor.
        '''
        return self._roam_distance

    def get_distance_mm(self):
        if not self.enabled:
            self._log.warning('get_distance_mm called while Roam is not enabled.')
            return None
        if not self._vl53l5cx.enabled:
            self._log.warning('get_distance_mm called while VL53L5CX is not enabled.')
            return None
        return self._vl53l5cx.get_distance_mm()

    def get_roam_distance(self):
        '''
        Returns the average distance in front of the robot, derived from the first two non-floor rows
        and the center two columns (columns 3 and 4). The result is a single float value.
        '''
        data = self.get_distance_mm()
        if data is None:
            self._log.warning('no data.')
            return None
        distance_grid = np.array(data).reshape((self._rows, self._cols))
        # use columns 3 and 4 (center two columns for 8-column sensor)
        center_cols = [3, 4]
        values = [] 
        for row in self._vl53l5cx.non_floor_rows:
            for col in center_cols:
                values.append(distance_grid[row, col])
        if values:
#           front_distance = float(np.mean(values))   # use mean
            front_distance = float(np.median(values)) # use median
        else:
            front_distance = None
        self._log.info('roam distance: ' + Fore.BLUE + '{}mm'.format(front_distance))
        return int(front_distance)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _accelerate(self):
        self._log.info("accelerate…")
        if self._motor_controller:
            self._motor_controller.accelerate(self._default_speed, enabled=lambda: not self._stop_event.is_set())

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _decelerate(self):
        self._log.info("decelerate…")
        if self._motor_controller:
            self._motor_controller.decelerate(0.0, enabled=lambda: not self._stop_event.is_set())

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _stop(self):
        self._log.info("stop…")
        if self._motor_controller:
            self._motor_controller.stop()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_stoppage(self):
        '''
        Called when it's clear that the robot has stopped.
        '''
        self._log.info(Fore.WHITE + "🌼 stoppage")
#       self._motor_controller.play('boink')
        # notify Roam that the robot has stopped
        _message = self.message_factory.create_message(Event.ROAM, Roam.STOPPED)
        self._queue_publisher.put(_message)
        self._log.info(Fore.WHITE + "🌼 published ROAM message: {}".format(_message))
#       self.suppress() # TODO this should be done by a takeover Behaviour

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enable the roaming behaviour.
        '''
        if self._motor_controller:
            self._motor_controller.enable()
        Component.enable(self)
        self._loop_instance = asyncio.new_event_loop()
        self._stop_event = ThreadEvent()
        asyncio.set_event_loop(self._loop_instance)
        self._task = self._loop_instance.create_task(self._loop_main())
        # start the loop in a background thread
        self._thread = Thread(target=self._loop_instance.run_forever, daemon=True)
        self._thread.start()
        self._log.info("roam enabled.")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def suppress(self):
        '''
        Suppresses this Component.
        '''
        Behaviour.suppress(self)
        self._log.info("roam suppressed.")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable the roaming behaviour.
        '''
        if not self.enabled:
            self._log.warning("already disabled.")
            return
        self._log.info(Fore.YELLOW + "roam disabling…")
        # set stop flag
        self._stop_event.set()
        time.sleep(0.1)
        if self._loop_instance:
            self._loop_instance.stop()
            self._loop_instance.call_soon_threadsafe(self._shutdown)
        if self._motor_controller:
            self._motor_controller.disable()
        Component.disable(self)
        self._log.info("disabled.")

    def _shutdown(self):
        if not self._task.done():
            self._task.cancel()
        self._loop_instance.stop()

#EOF
