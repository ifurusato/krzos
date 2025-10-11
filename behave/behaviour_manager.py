#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-02-16
# modified: 2025-10-11


import traceback
import os, inspect, importlib.util # to locate Behaviours
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.event import Event, Group
from core.orientation import Orientation
from core.message import Message
from core.message_factory import MessageFactory
from core.subscriber import Subscriber
from core.util import Util
from behave.behaviour import Behaviour

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class BehaviourManager(Subscriber):
    CLASS_NAME='behave-mgr'
    '''
    Extends Subscriber as a manager of high-level, low-priority behaviours.
    This subscribes to all events grouped as an Event.BEHAVIOUR.

    See the note under suppress_all_behaviours() regarding the stored state
    of Behaviours when this manager is itself suppressed.

    :param name:         the subscriber name (for logging)
    :param config:       the application configuration
    :param message_bus:  the message bus
    :param level:        the logging level
    '''
    def __init__(self, config, message_bus=None, message_factory=None, level=Level.INFO):
        Subscriber.__init__(self, BehaviourManager.CLASS_NAME, config, message_bus=message_bus, suppressed=False, enabled=True, level=Level.INFO)
        self._config = config
        if not isinstance(message_factory, MessageFactory):
            raise ValueError('wrong type for message factory argument: {}'.format(type(message_factory)))
        self._message_factory  = message_factory
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._level            = level
        self._active_behaviour = None
        self._was_suppressed   = None
        self._clip_event_list  = True #_cfg.get('clip_event_list') # used for printing only
        self._clip_length      = 42   #_cfg.get('clip_length')
        self._find_behaviours()
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_behaviours(self):
        '''
        Returns a list of all Behaviour instances registered in the ComponentRegistry.
        '''
        return Component.get_registry().filter_by_type(Behaviour)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _find_behaviours(self):
        '''
        Instantiates all Behaviour subclasses found in the ./behave/ directory if they
        are not already present in the ComponentRegistry.
        '''
        registry = Component.get_registry()
        current_dir = os.path.dirname(os.path.abspath(__file__))
        for filename in os.listdir(current_dir):
            if filename.endswith(".py") and filename != os.path.basename(__file__):
                module_path = os.path.join(current_dir, filename)
                module_name = filename[:-3]
                spec = importlib.util.spec_from_file_location(module_name, module_path)
                if spec and spec.loader:
                    module = importlib.util.module_from_spec(spec)
                    try:
                        spec.loader.exec_module(module)
                        for name, obj in inspect.getmembers(module, inspect.isclass):
                            if issubclass(obj, Behaviour) and obj is not Behaviour:
                                # Only instantiate if not in ComponentRegistry
                                already_registered = False
                                for behaviour in registry.filter_by_type(obj):
                                    if behaviour.classname == name:
                                        already_registered = True
                                        break
                                if not already_registered:
                                    _behaviour = obj(
                                        self._config,
                                        self._message_bus,
                                        self._message_factory,
                                        self._level
                                    )
                                    self._log.info("instantiated behaviour name '{}' as '{}'".format(name.lower(), _behaviour.name))
                                else:
                                    self._log.info("behaviour '{}' already registered; skipping instantiation.".format(name))
                    except Exception as e:
                        self._log.error("{} thrown loading filename '{}' for behaviour: {}".format(type(e), filename, e))
        # list registered behaviours
        behaviours = registry.filter_by_type(Behaviour)
        if len(behaviours) > 0:
            self._log.info('registered {} behaviours:'.format(len(behaviours)))
            for _behaviour in behaviours:
                self._log.info('   behaviour: ' + Fore.GREEN + "'{}'".format(_behaviour.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def start(self):
        '''
        The necessary state machine call to start the publisher, which performs
        any initialisations of active sub-components, etc.
        '''
        self._log.info('starting behaviour manager…')
        for _behaviour in self.get_behaviours():
            _behaviour.start()
            self._log.info('started behaviour {}'.format(_behaviour.name))
        Subscriber.start(self)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable_all_behaviours(self):
        '''
        Enable all registered behaviours. Some may already be enabled if they
        are also Publisher subclasses, when the MessageBus started up.
        '''
        if not self.closed:
            self._log.info('enable all behaviours…')
            for _behaviour in self.get_behaviours():
                enable_flag  = self._config['kros']['behaviour'].get('enable_{}_behaviour'.format(_behaviour.name.lower()), False)
                if enable_flag:
                    _behaviour.enable()
                    self._log.info(Fore.MAGENTA + "{} behaviour enabled.".format(_behaviour.name))
                else:
                    _behaviour.disable()
                    self._log.info(Fore.MAGENTA + "{} behaviour disabled.".format(_behaviour.name))
                release_flag = self._config['kros']['behaviour'].get('release_{}_behaviour'.format(_behaviour.name.lower()), False)
                if release_flag:
                    _behaviour.release()
                    self._log.info(Fore.MAGENTA + "{} behaviour released.".format(_behaviour.name))
                else:
                    _behaviour.suppress()
                    self._log.info(Fore.MAGENTA + "{} behaviour suppressed.".format(_behaviour.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable_all_behaviours(self):
        '''
        Disable all registered behaviours.
        '''
        self._log.info('disable all behaviours…')
        for _behaviour in self.get_behaviours():
            _behaviour.disable()
            self._log.info('{} behaviour disabled.'.format(_behaviour.name))
        self._was_suppressed = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def suppress(self):
        '''
        Suppresses the Behaviour Manager as well as any registered Behaviours.
        '''
        Component.suppress(self)
        self.suppress_all_behaviours()
        self._log.info('suppressed.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def suppress_all_behaviours(self, store_states=True):
        '''
        Suppresses all registered behaviours. This stores the suppressed state
        of each Behaviour.

        STORAGE POLICY

        Note that suppressing the BehaviourManager (or alternately calling
        this method, which calls this method) will suppress all existing
        Behaviours and store their respective suppression states. If there
        is a set of stored states upon releasing the BehaviourManager, those
        states will be restored.
        '''
        self._log.info('suppress all behaviours…')
        self._was_suppressed = {}
        for _behaviour in self.get_behaviours():
            self._was_suppressed[_behaviour] = _behaviour.suppressed
            if not _behaviour.suppressed:
                _behaviour.suppress()
                self._log.info('{} behaviour suppressed.'.format(_behaviour.name))
            else:
                self._log.info('{} behaviour not suppressed (was not active).'.format(_behaviour.name))
        if not store_states:
            self._was_suppressed = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def release(self):
        '''
        Releases (un-suppresses) the BehaviourManager and releases all
        Behaviours according to the storage policy.
        '''
        Component.release(self)
        self.release_all_behaviours()
        self._log.info('released.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def release_all_behaviours(self):
        '''
        Release (un-suppress) all registered behaviours.

        If the suppressed state of each Behaviour has been stored this will
        determine whether a given Behaviour is released.
        '''
        if not self.closed:
            self._log.info('release all behaviours…')
            for _behaviour in self.get_behaviours():
                if self._was_suppressed:
                    if not self._was_suppressed[_behaviour]:
                        _behaviour.release()
                else:
                    _behaviour.release()
                self._log.info('{} behaviour released.'.format(_behaviour.name))
            self._was_suppressed = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close_all_behaviours(self):
        '''
        Permanently close all registered behaviours. They cannot be reopened
        or otherwise enabled after this.
        '''
        for _behaviour in self.get_behaviours():
            _behaviour.close()
            self._log.info('{} behaviour closed.'.format(_behaviour.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def process_message(self, message):
        '''
        Process the message.

        :param message:  the message to process.
        '''
        _event = message.event
        if message.gcd:
            raise GarbageCollectedError('cannot process message: message has been garbage collected. [3]')
        self._log.info(Fore.MAGENTA + 'pre-processing message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))
        self._log.debug('awaiting subscriber process_message {}.'.format(_event.name))
        await Subscriber.process_message(self, message)
        self._log.debug('complete: awaited subscriber process_message {}.'.format(_event.name))
        self._log.debug('post-processing message {}'.format(message.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def print_info(self):
        '''
        Print information about the currently registered Behaviours.
        '''
        self._log.info('behaviour manager:')
        self._log.info('  suppressed:\t' + Fore.YELLOW + '{}'.format(self.suppressed))
        self._log.info('  behaviours:')
        for _behaviour in self.get_behaviours():
            if self._clip_event_list:
                _event_list = Util.ellipsis(_behaviour.print_events(), self._clip_length)
            else:
                _event_list = _behaviour.print_events()
            self._log.info(Fore.YELLOW + '\t{}'.format(_behaviour.name)
                    + Fore.CYAN + ' {}enabled: '.format((' ' * max(0, (10 - len(_behaviour.name)))))
                    + Fore.YELLOW + '{}\t'.format(_behaviour.enabled)
                    + Fore.CYAN + 'suppressed: '
                    + Fore.YELLOW + '{}\t'.format(_behaviour.suppressed)
                    + Fore.CYAN + 'listening for: '
                    + Fore.YELLOW + '{}'.format(_event_list))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enable the behaviour manager and all behaviours.
        '''
        self._log.info('enabling behaviour manager and all behaviours…')
#       self.release_all_behaviours()
        self.enable_all_behaviours()
        Subscriber.enable(self)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable the behaviour manager and all behaviours.
        '''
        self._log.info('disabling behaviour manager and all behaviours…')
#       self.suppress_all_behaviours()
        self.disable_all_behaviours()
        Subscriber.disable(self)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Permanently close and disable the behaviour manager and all behaviours.
        '''
        if not self.closed:
            Subscriber.close(self) # will call disable
            self.close_all_behaviours()

#EOF
