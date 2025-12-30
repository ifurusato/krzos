#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-12-01
# modified: 2025-12-01

from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component, MissingComponentError
from behave.behaviour import Behaviour
from hardware.odometer import Odometer
from hardware.motor_controller import MotorController

class Smoothing(Behaviour):
    NAME = 'smoothing'
    _LISTENER_LOOP_NAME = '__thoughts_listener_loop'
    '''
    A experimental Behaviour that adds a smoothing post-processor to the
    motor controller.

    Q: Is this a behaviour or simply a filter?

    :param config:          the application configuration
    :param level:           the optional log level
    '''
    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        if message_bus is None:
            raise ValueError('no message bus argument.')
        Behaviour.__init__(self, Smoothing.NAME, config, message_bus, message_factory,
                          suppressed=True, enabled=False, level=level)
        # idle configuration
        _idle_cfg = config['kros']['behaviour']['smoothing']
        # thoughts configuration
        _cfg = config['kros']['behaviour']['smoothing']
        self._verbose  = _cfg.get('verbose', False)
        self._priority = _cfg.get('priority', 0.4)
        # components
        _component_registry = Component.get_registry()
        self._odometer = _component_registry.get(Odometer.NAME)
        if self._odometer is None:
            self._log.info('odometer not available; relying on message activity alone.')
        else:
            self._log.info('odometer available; will monitor for movement.')
            self._odometer.add_callback(self._odometer_callback)
        self._motor_controller = _component_registry.get(MotorController.NAME)
        if self._motor_controller is None:
            raise MissingComponentError('missing motor controller.')
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def using_dynamic_priority(self):
        return False

    @property
    def priority(self):
        '''
        Returns the current priority for this behaviour.
        Subclasses can override to provide dynamic priority.
        '''
        return self._priority

    def process_intent_vector(self, intent):
        if self.enabled and self.released:
            return self._smoothing_filter(intent)
        else:
            return intent

    def _smoothing_filter(self, intent):
        # TODO process for smoothing here

        vx, vy, omega = intent
        self._log.info(Fore.YELLOW + 'intent: vx={:.3f}, vy={:.3f}, omega={:.3f}'.format(vx, vy, omega))
        return intent

    def execute(self, message):
        '''
        Noop in this class.
        '''
        pass

    def enable(self):
        if not self.enabled:
            self._log.info('enabling smoothing…')
            self._motor_controller.set_post_processor(self)
            self._log.info('added smoothing callback to motor controller.')
            super().enable()
            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def _odometer_callback(self):
        self._log.info(Style.DIM + 'odometer reports movement.')
        self._idle_count  = 0

    def disable(self):
        if self.enabled:
            self._log.debug('disabling…')
            self._motor_controller.set_post_processor(None)
            self._log.info('removed smoothing callback from motor controller.')
            super().disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

    def close(self):
        if not self.closed:
            self._log.debug('closing…')
            super().close()
            self._log.info('closed.')
        else:
            self._log.warning('already closed.')

#EOF
