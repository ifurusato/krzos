#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-05-19
# modified: 2025-11-29

from abc import ABC, abstractmethod
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.event import Event
from core.message import Message
from core.message_bus import MessageBus
from core.message_factory import MessageFactory
from core.subscriber import Subscriber

class Behaviour(ABC, Subscriber):
    NAME = 'behaviour'
    '''
    An abstract class providing the basis for a behaviour.
    The loop callback is registered during class construction.

    :param log_or_name:      the name of this behaviour, or an existing logger
    :param config:           the application configuration
    :param message_bus:      the message bus
    :param message_factory:  the message factory
    :param suppressed:       suppressed state, default False
    :param enabled:          enabled state, default True
    :param level:            the optional log level
    '''
    def __init__(self, log_or_name, config, message_bus, message_factory, suppressed=True, enabled=False, level=Level.INFO):
        Subscriber.__init__(self, log_or_name, config, message_bus, suppressed=suppressed, enabled=enabled, level=Level.INFO)
        if not isinstance(message_factory, MessageFactory):
            raise ValueError('expected MessageFactory, not {}.'.format(type(message_factory)))
        self._message_factory = message_factory
        # get instance of BehaviourManager
        self._behaviour_manager = Component.get_registry().get('behave-mgr') # hard-coded to avoid circular ref
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def name(self):
        return type(self).NAME

    @property
    def message_factory(self):
        return self._message_factory

    def is_released_by_toggle(self):
        '''
        Returns True if this Behaviour is currently released by a toggle switch.
        '''
        return self._behaviour_manager.is_released_by_toggle(self.name)

    def has_toggle_assignment(self):
        '''
        Returns True if this Behaviour has a toggle assignment.
        '''
        return self._behaviour_manager.has_toggle_assignment(self.name)

    async def process_message(self, message):
        '''
        Overrides the method in Subscriber.
        '''
        if message.gcd:
            raise GarbageCollectedError('cannot process message: message has been garbage collected. [3]')
        _event = message.payload.event
        self._log.info('processing message {}; with event '.format(message.name) + Fore.YELLOW + ' {}'.format(_event.name))
        # indicate that this subscriber has processed the message
        message.process(self)
        # now process message...
        if not self.suppressed:
            self.execute(message)
        self._log.debug('processed message {}'.format(message.name))

    @abstractmethod
    def execute(self, message):
        '''
        The method called by process_message(). This does nothing in this
        abstract class and is meant to be extended by subclasses. It is not
        called when the behaviour is suppressed.

        :param message:  the Message passed along by the message bus
        '''
        raise NotImplementedError('execute() must be implemented in subclasses.')

    def enable(self):
        '''
        The necessary state machine call to enable the behaviour.
        '''
        if not self.closed:
            if not self.enabled:
                super().enable()
                self._log.debug('enabled {} behaviour.'.format(self.name))

    def disable(self):
        '''
        The state machine call to disable the behaviour.
        '''
        if self.enabled:
            super().disable()
            self._log.debug('disabled {} behaviour.'.format(self.name))

#EOF
