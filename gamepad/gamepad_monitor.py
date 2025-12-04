#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-09-06
# modified: 2025-11-27
#
# Monitors the device path for the Gamepad, and if it goes null, puts a
# DISAPPEARED message into the queue publisher to signal a system shutdown.
#
# This also has a close method that is responsible for cleanly closing all
# gamepad-related classes.

import itertools
from colorama import init, Fore, Style
init()

from core.component import Component
from core.event import Event
from core.logger import Logger, Level
from hardware.irq_clock import IrqClock

class GamepadMonitor(Component):
    NAME = 'gamepad-mon'
    '''
    A simple callback on the IRQ clock that monitors the device path used by
    the Gamepad, executing a callback if the device disappears. You must call
    enable to establish the callback on the clock.

    Args:
        config:    the application configuration
        gamepad:   the gamepad to monitor
        callback:  the callback to execute if the path goes cold
        level:     the logging Level
    '''
    def __init__(self, config=None, gamepad=None, callback=None, level=Level.INFO):
        self._log = Logger(GamepadMonitor.NAME, level)
        self._config = config
        Component.__init__(self, self._log, False, False)
        self._component_registry = Component.get_registry()
        self._clock = self._component_registry.get(IrqClock.NAME)
        if self._clock is None:
            raise ValueError('no IRQ clock available.')
        self._queue_publisher = self._component_registry.get('pub:queue')
        if self._queue_publisher is None:
            raise ValueError('no queue publisher available.')
        self._message_factory = self._component_registry.get('msgfactory')
        if self._message_factory is None:
            raise ValueError('no message factory available.')
        self._counter  = itertools.count()
        self._gamepad  = gamepad
        self._disconnected_callback = callback
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def name(self):
        return GamepadMonitor.NAME

    def no_connection(self):
        '''
        Publishes a NO_CONNECTION error message to the message bus.
        '''
        if self._queue_publisher:
            _message = self._message_factory.create_message(Event.NO_CONNECTION, 'no gamepad available.')
            self._queue_publisher.put(_message)
        else:
            self._log.warning('no queue publisher avaiable: no gamepad available! Time to shut down manually.')

    def _poll(self):
        if self._gamepad.has_connection():
            self._log.debug('gamepad connected.')
        else:
            self._log.warning('gamepad disconnected!')
            self._disconnected_callback()
            if self._queue_publisher:
                _message = self._message_factory.create_message(Event.DISCONNECTED, 'gamepad disconnected.')
                self._queue_publisher.put(_message)
            else:
                self._log.warning('no queue publisher available: gamepad disconnected! Time to shut down manually.')

    def enable(self):
        if not self.enabled:
            super().enable()
            self._clock.add_callback(self._poll)
            self._clock.enable()
            self._log.info('enabled gamepad monitor.')
        else:
            self._log.warning('already enabled.')

    def disable(self):
        if self.enabled:
            self._clock.remove_callback(self._poll)
            self._clock.disable()
            super().disable()
            self._log.debug('disabled.')
        else:
            self._log.debug('already disabled.')

    def close(self):
        if not self.closed:
            try:
                from gamepad.gamepad import Gamepad
                from gamepad.gamepad_controller import GamepadController
                from gamepad.gamepad_publisher import GamepadPublisher

                self._log.info('closing all gamepad components…')
                _components = []
                _gamepad_controller = self._component_registry.get(GamepadController.NAME)
                if _gamepad_controller:
                    _components.append(_gamepad_controller)
                _gamepad_publisher  = self._component_registry.get(GamepadPublisher.NAME)
                if _gamepad_publisher:
                    _components.append(_gamepad_publisher)
                _gamepad = self._component_registry.get(Gamepad.NAME)
                if _gamepad:
                    _components.append(_gamepad)
                for _component in _components:
                    self._log.debug("closing & disabling: '{}'…".format(_component.name))
                    _component.close()
                super().close()
                self._log.info('all gamepad components closed.')
            except Exception as e:
                self._log.error('{} raised closing gamepad components: {}'.format(type(e), e))

            self._log.info('closed.')
        else:
            self._log.warning('already closed.')

#EOF
