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
        self._verbose   = _cfg.get('verbose', False)
        self._priority  = _cfg.get('default_priority', 0.3)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

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
