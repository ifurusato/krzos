#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-03-12
# modified: 2026-03-12

import traceback
import math
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.event import Event
from behave.async_behaviour import AsyncBehaviour
from hardware.motor_controller import MotorController
from hardware.movement_controller import MovementController
from hardware.rotation_controller import RotationController

class Navigator(AsyncBehaviour):
    NAME = 'navigator'
    '''
    Description TBD.
    '''
    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Navigator.NAME, level)
        _component_registry = Component.get_registry()
        _motor_controller = _component_registry.get(MotorController.NAME)
        if _motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        self._movement_controller = _component_registry.get(MovementController.NAME)
        # create movement controller if it doesnt' already exist
        if self._movement_controller is None:
            self._log.info('creating rotation controller…')
            _rotation_controller = None # RotationController(_config, _motor_controller, level=Level.INFO)
            self._log.info('creating movement controller…')
            self._movement_controller = MovementController(config, _motor_controller, _rotation_controller, level=level)
#           raise MissingComponentError('movement controller not available.')
        AsyncBehaviour.__init__(self, self._log, config, message_bus, message_factory, _motor_controller, level=level)
#       self.add_event(Event.AVOID) # TBD
        # configuration
        _cfg = config['kros'].get('behaviour').get('navigator')
        self._verbose        = _cfg.get('verbose', False)
        self._priority       = _cfg.get('default_priority', 0.3)
        self._default_speed  = _cfg.get('default_speed', 0.5)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def set_route(self, waypoints, is_first_route = False):
        '''
        Set the route to navigate. If is_first_route is True, sets the odometer
        pose from waypoint 0 and begins from waypoint 1. Otherwise compares
        current pose to waypoint 0 and inserts a pre-navigation leg if needed.
        '''
        if not waypoints:
            self._log.warning('set_route: empty waypoint list.')
            return
        self._log.info('🐮 set route to waypoints: {}'.format(waypoints))
        self._route       = waypoints
        self._route_index = 0
        _wp0      = self._route[0]
        _odometer = self._motor_controller.odometer
        if is_first_route:
            _heading_deg = _wp0.heading if _wp0.heading is not None else 0.0
            _odometer.set_pose(_wp0.position.x, _wp0.position.y,
                    math.radians(_heading_deg), use_radians=True)
            self._log.info('odometer pose set to waypoint 0: {}'.format(_wp0))
            self._route_index = 1
        else:
            _x, _y, _theta = _odometer.pose
            if _wp0.reached(_x, _y):
                self._route_index = 1
                self._log.info('already at waypoint 0, starting from waypoint 1.')
            else:
                self._log.info('not at waypoint 0, will navigate there first.')
        self._log.info('route set: {} waypoints, starting from index {}.'.format(
                len(self._route), self._route_index))

    @property
    def is_ballistic(self):
        return False

    def callback(self):
        raise NotImplementedError('callback unsupported in Navigator.')

    def execute(self, message):
        raise NotImplementedError('execute unsupported in Navigator.')

    def start_loop_action(self):
        pass

    def stop_loop_action(self):
        pass

    def _dynamic_set_default_speed(self):
        '''
        Updates default speed from digital potentiometer if available.
        '''
        # TBD
        pass

    async def _poll(self):
        '''
        The asynchronous poll, returns the intent vector.
        '''
        if self.suppressed or self.disabled:
            return (0.0, 0.0, 0.0)
        try:
#           if next(self._counter) % 5 == 0:
#               self._dynamic_set_default_speed()
            return self._update_intent_vector()
        except Exception as e:
            self._log.error("{} thrown while polling: {}\n{}".format(type(e), e, traceback.format_exc()))
            self.disable()
            return (0.0, 0.0, 0.0)

    def _update_intent_vector(self):
        '''
        Description TBD.
        Returns (vx, vy, omega) tuple.
        '''
        # TBD
        return (0.0, 0.0, 0.0)

    def enable(self):
        if not self.enabled:
            self._log.debug("enabling navigator…")
            super().enable()
            self._log.info("navigator enabled.")
        else:
            self._log.warning("already enabled.")

    def disable(self):
        if self.enabled:
            self._log.debug("disabling navigator…")
            super().disable()
            self._log.info('disabled.')
        else:
            self._log.warning("already disabled.")

#EOF
