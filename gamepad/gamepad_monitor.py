#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-09-06
# modified: 2025-09-15
#
# Monitors the device path for the Gamepad, and if it goes null, puts a
# DISAPPEARED message into the queue publisher to signal a system shutdown.
#

import itertools
from colorama import init, Fore, Style
init()

from core.component import Component
from core.event import Event
from core.logger import Logger, Level

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class GamepadMonitor(Component):
    NAME = 'gamepad-mon'
    '''
    A simple callback on the IRQ clock that monitors the device path used by
    the Gamepad, executing a callback if the device disappears. You must call
    enable to establish the callback on the clock.

    :param config:        the application configuration
    :param gamepad:       the gamepad to monitor
    :param callback:      the callback to execute if the path goes cold 
    :param level:         the logging Level
    '''
    def __init__(self, config=None, gamepad=None, callback=None, level=Level.INFO):
        self._log = Logger(GamepadMonitor.NAME, level)
        self._config = config
        Component.__init__(self, self._log, False, False)
        _component_registry = Component.get_registry()
        self._clock = _component_registry.get('irq-clock')
        if self._clock is None:
#           self._log.notice('no IRQ clock available; creating instance…')
#           from hardware.irq_clock import IrqClock
#           self._clock = IrqClock(config=self._config, level=Level.INFO)
            raise ValueError('no IRQ clock available.')
        self._queue_publisher = _component_registry.get('pub:queue')
        if self._queue_publisher is None:
            raise ValueError('no queue publisher available.')
        self._message_factory = _component_registry.get('msgfactory')
        if self._message_factory is None:
            raise ValueError('no message factory available.')
        self._counter  = itertools.count()
        self._gamepad  = gamepad
        self._disconnected_callback = callback
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def name(self):
        return 'gamepad-monitor'

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def no_connection(self):
        '''
        Publishes a NO_CONNECTION error message to the message bus.
        '''
        if self._queue_publisher:
            _message = self._message_factory.create_message(Event.NO_CONNECTION, 'no gamepad available.')
            self._queue_publisher.put(_message)
        else:
            self._log.warning('no queue publisher avaiable: no gamepad available! Time to shut down manually.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _poll(self):
#       if next(self._counter) % 5 == 0: # every second
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

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        if not self.enabled:
            Component.enable(self)
            self._clock.add_callback(self._poll)
            self._clock.enable()
            self._log.info('enabled gamepad monitor.')
        else:
            self._log.warning('already enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        if self.enabled:
            self._clock.remove_callback(self._poll)
            self._clock.disable()
            Component.disable(self)
            self._log.debug('successfully disabled.')
        else:
            self._log.debug('already disabled.')

#EOF
