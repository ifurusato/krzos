#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2019-12-23
# modified: 2025-10-12

import asyncio
import random
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.fsm import FiniteStateMachine

class Publisher(Component, FiniteStateMachine):
    '''
    Extends Component and FiniteStateMachine as a message/event publisher to
    the message bus.

    :param log_or_name:      the Logger or publisher name (for logging)
    :param config:           the application configuration
    :param message_bus:      the asynchronous message bus
    :param message_factory:  the factory for messages
    :param suppressed:       the supprsssed flag (optional, default False)
    :param level:            the logging level
    '''
    def __init__(self, log_or_name, config, message_bus, message_factory, suppressed=False, enabled=False, level=Level.INFO):
        if isinstance(log_or_name, Logger):
            self._log = log_or_name
            self._name = self._log.name
        elif isinstance(log_or_name, str):
            self._log = Logger('pub:{}'.format(log_or_name), level)
            self._name = log_or_name
        else:
            raise ValueError('wrong type for log_or_name argument: {}'.format(type(log_or_name)))
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._level = level
        if not isinstance(config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(name)))
        self._config = config
        if message_bus is None:
            raise ValueError('no message bus argument provided.')
        self._message_bus = message_bus
        if message_factory is None:
            raise ValueError('no message factory argument provided.')
        self._message_factory = message_factory
        Component.__init__(self, self._log, suppressed=suppressed, enabled=enabled)
        FiniteStateMachine.__init__(self, self._log, self._name)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def logger(self):
        return self._log

    def set_log_level(self, level):
        self._log.level = level

    @property
    def name(self):
        return self._name

    @property
    def message_bus(self):
        return self._message_bus

    @property
    def message_factory(self):
        return self._message_factory

    async def publish(self, message):
        '''
        Asynchronously publishes the message to the message bus.
        This is preferred to calling the message bus directly, and
        as a rule should not be overridden by subclasses.
        '''
        await self._message_bus.publish_message(message)
        await asyncio.sleep(0.05)

    def start(self):
        '''
        The necessary state machine call to start the publisher, which performs
        any initialisations of active sub-components, etc.
        '''
        FiniteStateMachine.start(self)

    def __eq__(self, obj):
        return isinstance(obj, Publisher) and obj.name == self.name

#EOF
