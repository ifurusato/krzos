#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-02-16
# modified: 2025-11-28

import sys
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
from behave.async_behaviour import AsyncBehaviour
from behave.idle import Idle
from behave.behaviour import Behaviour
from hardware.toggle_config import ToggleConfig

class BehaviourManager(Subscriber):
    NAME='behave-mgr'
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
        Subscriber.__init__(self, BehaviourManager.NAME, config, message_bus=message_bus, suppressed=False, enabled=True, level=Level.INFO)
        self._config = config
        if not isinstance(message_factory, MessageFactory):
            raise ValueError('wrong type for message factory argument: {}'.format(type(message_factory)))
        self._message_factory  = message_factory
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._level            = level
        self._was_suppressed   = None
        # data logger?
        self._data_log = None
        _data_logging = config['kros'].get('application').get('data_logging')
        if _data_logging:
            self._log.info(Fore.GREEN + 'data logging is active.')
            self._data_log = Logger('{}'.format(self.NAME), log_to_file=True, data_logger=True, level=Level.INFO)
            self._data_log.data('START')
        # configuration
        _cfg = config['kros'].get('behaviour_manager')
        self._clip_event_list  = True #_cfg.get('clip_event_list') # used for printing only
        self._clip_length      = 42   #_cfg.get('clip_length')
        self._release_on_startup = _cfg.get('release_on_startup')
        self.add_event(Event.IDLE)
        self._log.info('subscribed to IDLE events.')
        self._configured_behaviour_names = list(self._config['kros']['behaviour'].keys())
        # toggle configuration
        self._released_by_toggle = {}
        _component_registry = Component.get_registry()
        self._toggle_config = _component_registry.get(ToggleConfig.NAME)
        self._find_behaviours(_component_registry)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def is_released_by_toggle(self, name):
        '''
        Returns True if the named Behaviour has been configured to be released
        via a toggle switch, and the toggle switch is set on.
        '''
        _released = self._released_by_toggle[name]
        _enabled  = self._toggle_config.is_enabled(name)
        return _released and _enabled

    def has_toggle_assignment(self, name):
        '''
        Returns True if the name has a toggle assignment.
        '''
        return self._toggle_config.has_assignment(name)

    def get_configured_behaviour_names(self):
        '''
        Returns the list of behaviour names configured at startup.
        '''
        return self._configured_behaviour_names

    def get_behaviours(self):
        '''
        Returns a list of all Behaviour instances registered in the ComponentRegistry.
        '''
        return Component.get_registry().filter_by_type(Behaviour)

    def _find_behaviours(self, component_registry):
        '''
        Instantiates all Behaviour subclasses defined in the configuration if they
        are not already present in the ComponentRegistry. Uses convention-based
        dynamic import: each behaviour key should correspond to a module named
        '{key}_behaviour' containing a class '{Key}Behaviour'.
        '''
        for behaviour_key in self._configured_behaviour_names:
            if behaviour_key == AsyncBehaviour.NAME: # skip abstract class
                continue
            enable = self._config['kros']['behaviour'][behaviour_key.lower()]['enable']
            if not enable:
                self._log.info('skipping disabled behaviour: {}'.format(behaviour_key))
                self._released_by_toggle[behaviour_key] = False
                continue
            if component_registry.has(behaviour_key):
                self._log.info("behaviour '{}' already registered; skipping instantiation.".format(behaviour_key))
                continue
            try:
                self._log.info('instantiating behaviour {}…'.format(behaviour_key))
                module_name = 'behave.{}'.format(behaviour_key.lower())
                module = importlib.import_module(module_name)
                class_name = '{}'.format(behaviour_key.capitalize())
                behaviour_class = None
                for name, obj in inspect.getmembers(module, inspect.isclass):
                    if name == class_name:
                        behaviour_class = obj
                        break
                if behaviour_class is None:
                    self._log.warning("could not find class '{}' in module '{}'.".format(class_name, module_name))
                    continue
                _behaviour = behaviour_class(
                    self._config,
                    self._message_bus,
                    self._message_factory,
                    self._level
                )
                # set released-by-toggle lambda
                self._released_by_toggle[behaviour_key] = lambda _behaviour: self._toggle_config.is_enabled(_behaviour.name)
                self._log.info("instantiated behaviour '{}' as '{}'.".format(behaviour_key, _behaviour.name))
            except ImportError as e:
                self._log.warning("Could not import module '{}' for behaviour '{}': {}".format(module_name, behaviour_key, e))
            except Exception as e:
                self._log.error("{} thrown instantiating behaviour '{}': {}\n{}".format(type(e), behaviour_key, e, traceback.format_exc()))
        # list registered behaviours
        behaviours = self.get_behaviours()
        if len(behaviours) > 0:
            self._log.info("registered {} behaviour{}:".format(len(behaviours), '' if len(behaviours) == 1 else 's'))
            for _behaviour in behaviours:
                self._log.info('   behaviour: ' + Fore.GREEN + '{}'.format(_behaviour.name))

    def start(self):
        '''
        The necessary state machine call to start the publisher, which performs
        any initialisations of active sub-components, etc.
        '''
        self._log.info('starting behaviour manager…')
        for _behaviour in self.get_behaviours():
            if _behaviour.enabled and not _behaviour.suppressed:
                _behaviour.start()
                self._log.info(Fore.GREEN + 'started behaviour {}'.format(_behaviour.name))
            else:
                self._log.info(Style.DIM + 'behaviour {} not started (enabled={}, suppressed={})'.format(_behaviour.name, _behaviour.enabled, _behaviour.suppressed))
        Subscriber.start(self)

    def enable_all_behaviours(self):
        '''
        Enable all registered behaviours using the new YAML config structure.
        Each behaviour is enabled/released according to its own section under kros.behaviour.
        '''
        if not self.closed:
            self._log.info(Fore.MAGENTA + 'enable/release all behaviours…')
            _behaviour_cfg = self._config['kros']['behaviour']
            for _behaviour in self.get_behaviours():
                _settings = _behaviour_cfg.get(_behaviour.name, {})
                if _settings.get('enable', False):
                    if not _behaviour.enabled:
                        _behaviour.enable()
                        self._log.info(Fore.GREEN + "{} behaviour enabled.".format(_behaviour.name))
                else:
                    if not _behaviour.disabled:
                        _behaviour.disable()
                        self._log.info(Style.DIM + "{} behaviour disabled.".format(_behaviour.name))
                if _settings.get('release', False):
#                   if not _behaviour.released:
                    _behaviour.release()
                    self._log.info(Fore.GREEN + "{} behaviour released.".format(_behaviour.name))
                else:
                    if not _behaviour.suppressed:
                        _behaviour.suppress()
                        self._log.info(Style.DIM + "{} behaviour suppressed.".format(_behaviour.name))

    def disable_all_behaviours(self):
        '''
        Disable all registered behaviours.
        '''
        self._log.info('disable all behaviours…')
        for _behaviour in self.get_behaviours():
            if not _behaviour.disabled:
                _behaviour.disable()
                self._log.info('{} behaviour disabled.'.format(_behaviour.name))
        self._was_suppressed = None

    def suppress(self):
        '''
        Suppresses the Behaviour Manager as well as any registered Behaviours.
        '''
        super().suppress()
        self.suppress_all_behaviours()
        self._log.info('suppressed.')

    def suppress_all_behaviours(self, store_states=True):
        '''
        Suppresses all registered behaviours. This stores the suppressed state
        of each Behaviour.

        STORAGE POLICY

        Note that suppressing the BehaviourManager (or alternately calling
        this method) will suppress all existing Behaviours and store their
        respective suppression states. If there is a set of stored states
        upon releasing the BehaviourManager, those states will be restored.
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

    def release(self):
        '''
        Releases (un-suppresses) the BehaviourManager and releases all
        Behaviours according to the storage policy.
        '''
        Component.release(self)
        self.release_all_behaviours()
        self._log.info('released.')

    def release_all_behaviours(self):
        '''
        Release (un-suppress) all registered behaviours.

        If the suppressed state of each Behaviour has been stored this will
        determine whether a given Behaviour is released.
        '''
        self._log.info(Fore.WHITE + Style.BRIGHT + 'A. release all behaviours…  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx ')
        if self.closed:
            self._log.warning('cannot release behaviours: behaviour manager is closed.')
            return
        elif not self.enabled:
            self._log.warning('cannot release behaviours: behaviour manager is disabled.')
            return
        self._log.info(Fore.WHITE + Style.BRIGHT + 'B. release all behaviours…  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx ')
        _idle_behaviour = None
        for _behaviour in self.get_behaviours():
            # special handling for Idle - reset its timer instead of releasing
            if _behaviour.name == Idle.NAME:
                _idle_behaviour = _behaviour
                self._log.debug('found idle behaviour, will reset timer after releasing others.')
                continue
            # only process enabled behaviours
            if not _behaviour.enabled:
                self._log.debug('skipping disabled behaviour: {}'.format(_behaviour.name))
                continue
            elif not self.is_released_by_toggle(_behaviour.name):
                self._log.info('skipping toggle-disabled behaviour: {}'.format(_behaviour.name))
                continue
            # if we have stored suppression states, restore them
            if self._was_suppressed is not None:
                if not self._was_suppressed.get(_behaviour, True):  # was not suppressed before
                    _behaviour.release()
                    self._log.info(Fore.GREEN + '{} behaviour released (restored state).'.format(_behaviour.name))
                else:
                    self._log.debug('{} behaviour remains suppressed (restored state).'.format(_behaviour.name))
            else:
                # no stored state - release if currently suppressed
                if _behaviour.suppressed:
                    _behaviour.release()
                    self._log.info(Fore.GREEN + '{} behaviour released.'.format(_behaviour.name))
                else:
                    self._log.debug('{} behaviour already released.'.format(_behaviour.name))
        # reset Idle's timer after releasing other behaviors
        if _idle_behaviour and _idle_behaviour.enabled and not _idle_behaviour.suppressed:
            _idle_behaviour.reset_activity_timer()
            self._log.info(Fore.GREEN + 'idle activity timer reset.')
        # clear stored states
        self._was_suppressed = None

    def close_all_behaviours(self):
        '''
        Permanently close all registered behaviours. They cannot be reopened
        or otherwise enabled after this.
        '''
        for _behaviour in self.get_behaviours():
            _behaviour.close()
            self._log.info('{} behaviour closed.'.format(_behaviour.name))

    async def process_message(self, message):
        '''
        Process the message.

        :param message:  the message to process.
        '''
        self._log.info('🔔 a. message received: {}'.format(message))
        _event = message.event
        self._log.info('🔔 b. event: {}'.format(_event))
        if message.gcd:
            raise GarbageCollectedError('cannot process message: message has been garbage collected. [3]')
        # handle IDLE events
        if _event is Event.IDLE:
            elapsed = message.value
            self._log.info('🔔 d. IDLE threshold exceeded ({:.1f}s) - releasing suppressed behaviors'.format(elapsed))
            self.release_all_behaviours()
        else:
            self._log.info(Fore.MAGENTA + '🔔 e. pre-processing message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))
        self._log.info('awaiting subscriber process_message {}.'.format(_event.name))
        await Subscriber.process_message(self, message)
        self._log.info('🔔 f. complete: awaited subscriber process_message {}.'.format(_event.name))

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
            enabled_text    = Style.NORMAL + "enabled" \
                    if _behaviour.enabled \
                    else Style.DIM + "disabled"
            suppressed_text = Style.DIM + "suppressed" \
                    if _behaviour.suppressed \
                    else Style.NORMAL   + "released  "
            priority_text   = "dynamic ({:3.1f})".format(_behaviour.priority) \
                    if _behaviour.using_dynamic_priority \
                    else 'fixed ({:3.1f})'.format(_behaviour.priority)
            self._log.info("{:<12}".format(_behaviour.name)
                + Style.DIM   + " | " + Style.NORMAL
                + Fore.YELLOW + "{:<9}".format(enabled_text)
                + Fore.CYAN + Style.DIM  + " | " + Style.NORMAL
                + Fore.YELLOW + "{:<11}".format(suppressed_text)
                + Fore.CYAN  + Style.DIM + " | "
                + Style.NORMAL + " priority: "
                + Fore.YELLOW + "{:<9}".format(priority_text)
                + Fore.CYAN  + Style.DIM + " | "
                + Style.NORMAL + "listening for: "
                + Fore.YELLOW + "{}".format(_event_list)
            )

    def enable(self):
        '''
        Enable the behaviour manager. If the release on startup flag is
        True, enables and/or releases all behaviours depending upon the
        configuration.
        '''
        self._log.info('enabling behaviour manager…')
        if self._release_on_startup:
            self.enable_all_behaviours()
        self.print_info()
#       super().enable()
        Subscriber.enable(self)

    def disable(self):
        '''
        Disable the behaviour manager and all behaviours.
        '''
        self._log.info('disabling behaviour manager…')
        self.disable_all_behaviours()
#       super().disable()
        Subscriber.disable(self)

    def close(self):
        '''
        Permanently close and disable the behaviour manager and all behaviours.
        '''
        if not self.closed:
            if self._data_log:
                self._data_log.data('END')
                # note: it's not up to us to close the shared data logger
            self.close_all_behaviours()
            super().close()

#EOF
