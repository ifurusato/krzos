#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-01
# modified: 2025-11-01

import time
import numpy as np
import itertools
from math import isclose
from threading import Thread, Event as ThreadEvent
import asyncio
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.orientation import Orientation
from core.event import Event
from behave.async_behaviour import AsyncBehaviour
from hardware.motor_controller import MotorController
from hardware.aft_sensor import AftSensor
from hardware.side_sensor import SideSensor

class Avoid(AsyncBehaviour):
    NAME = 'avoid'
    '''
    An avoidance behaviour that uses the side and aft sensors to force the
    robot away from obstacles.
    '''
    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Avoid.NAME, level)
        _component_registry = Component.get_registry()
        _motor_controller = _component_registry.get(MotorController.NAME)
        if _motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        AsyncBehaviour.__init__(self, self._log, config, message_bus, message_factory, _motor_controller, level=level)
        self.add_event(Event.AVOID)
        # configuration
        _cfg = config['kros'].get('behaviour').get('avoid')
        self._default_speed  = _cfg.get('avoid_speed', 1.0)
        self._side_threshold_mm = _cfg.get('side_threshold_mm', 280)
        self._aft_threshold_mm  = _cfg.get('aft_threshold_mm', 500)
        self._aft_sensor  = AftSensor(config, level=Level.INFO)
        self._port_sensor = SideSensor(config, Orientation.PORT)
        self._stbd_sensor = SideSensor(config, Orientation.STBD)


        self._log.info('ready.')

    @property
    def name(self):
        return Avoid.NAME

    @property
    def is_ballistic(self):
        return False

    def execute(self, message):
        self._log.info('execute message {}.'.format(message))
        pass

    def start_loop_action(self):
        self._log.info('start loop action.')
        pass

    def stop_loop_action(self):
        self._log.info('stop loop action.')
        pass

    async def _poll(self):
        try:
            pass
            # poll sensors and set intent vector accordingly
            _port_distance = self._port_sensor.get_distance()
            _stbd_distance = self._stbd_sensor.get_distance()
            _aft_distance = self._aft_sensor.get_distance() * 10 # returned in cm
            # then set intent vector accordingly
            self._update_intent_vector(_port_distance, _stbd_distance, _aft_distance)
        except Exception as e:
            self._log.error("{} thrown while polling: {}".format(type(e), e))
            self.disable()

    def _update_intent_vector(self, port_distance, stbd_distance, aft_distance):
        '''
        Compute the robot's movement intent as a single (vx, vy, omega) vector
        based on the three distance sensors.
        '''
        # TODO update intent vector

        if port_distance and port_distance < self._side_threshold_mm:
            _port_color = Fore.RED + Style.NORMAL
        else:
            _port_color = Fore.RED + Style.DIM
        if stbd_distance and stbd_distance < self._side_threshold_mm:
            _stbd_color = Fore.GREEN + Style.NORMAL
        else:
            _stbd_color = Fore.GREEN + Style.DIM
        if aft_distance and aft_distance < self._aft_threshold_mm:
            _aft_color = Fore.YELLOW + Style.NORMAL
        else:
            _aft_color = Fore.YELLOW + Style.DIM
        self._log.info("update intent vector;{} port: {};{} stbd: {};{} aft: {}".format(
                _port_color,
                '{}mm'.format(port_distance) if port_distance else 'NA',
                _stbd_color,
                '{}mm'.format(stbd_distance) if stbd_distance else 'NA',
                _aft_color,
                '{}mm'.format(aft_distance)  if aft_distance else 'NA'))


    def enable(self):
        if not self.enabled:
            self._aft_sensor.enable()
            self._port_sensor.enable()
            self._stbd_sensor.enable()
            AsyncBehaviour.enable(self)
            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def disable(self):
        if self.enabled:
            self._aft_sensor.disable()
            self._port_sensor.disable()
            self._stbd_sensor.disable()
            AsyncBehaviour.disable(self)
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

#EOF
