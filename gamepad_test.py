#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-09-11
# modified: 2025-09-11
#

import asyncio
import sys, traceback
import time
import traceback
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.subscriber import Subscriber
from core.event import Group
from core.config_loader import ConfigLoader
from core.message_bus import MessageBus
from core.message_factory import MessageFactory
from core.queue_publisher import QueuePublisher

from gamepad.gamepad_publisher import GamepadPublisher
#from gamepad.gamepad_controller import GamepadController
from hardware.irq_clock import IrqClock

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class GamepadSubscriber(Subscriber):
    CLASS_NAME = 'gamepad'
    '''
    A subscriber to Gamepad events.

    :param config:       the application configuration
    :param message_bus:  the message bus
    :param level:        the logging level
    '''
    def __init__(self, config, message_bus, level=Level.INFO):
        Subscriber.__init__(self, GamepadSubscriber.CLASS_NAME, config, message_bus=message_bus, suppressed=False, enabled=False, level=level)
        self.add_events([ Group.GAMEPAD ])
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _arbitrate_message(self, message):
        '''
        Pass the message on to the Arbitrator and acknowledge that it has been
        sent (by setting a flag in the message).
        '''
        try:
            await self._message_bus.arbitrate(message.payload)
            message.acknowledge_sent()
            _value = message.payload.value
            self._log.info('arbitrated message ' + Fore.WHITE + '{} '.format(message.name)
                    + Fore.CYAN + 'for event \'{}\' with value type: '.format(message.event.name)
                    + Fore.YELLOW + '{}'.format(type(_value)))
        except Exception as e:
            self._log.error('{} raised arbitrating message: {}\n{}'.format(type(e), e, traceback.format_exc()))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def process_message(self, message):
        '''
        Process the message.

        :param message:  the message to process.
        '''
        try:
            if message.gcd:
                raise GarbageCollectedError('cannot process message: message has been garbage collected.')
            _event = message.event
            self._log.info('pre-processing message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))
            await Subscriber.process_message(self, message)
            self._log.info('post-processing message {}'.format(message.name))
        except Exception as e:
            self._log.error('{} raised processing message: {}\n{}'.format(type(e), e, traceback.format_exc()))

#EOF

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    _level = Level.INFO
    _message_bus       = None
    _queue_publisher   = None
    _clock = None
    _gamepad_publisher = None
    _gamepad_subscriber = None

    _log = Logger("dist-test", _level)

    try:

        _config = ConfigLoader(Level.INFO).configure()

        _log.info('creating message bus…')
        _message_bus = MessageBus(_config, _level)
        _log.info('creating message factory…')
        _message_factory = MessageFactory(_message_bus, _level)

        _queue_publisher = QueuePublisher(_config, _message_bus, _message_factory, _level)
        _clock = IrqClock(config=_config, level=Level.INFO)

#       _gamepad_subscriber = GamepadSubscriber(config=_config, message_bus=_message_bus, level=Level.INFO)

        _gamepad_publisher = GamepadPublisher(_config, _message_bus, _message_factory, exit_on_complete=True, level=_level)
#       _gamepad_controller = GamepadController(_message_bus, _level)

        _log.info("starting message bus…")
        _message_bus.enable()

        _log.info("waiting for gamepad distances…")
        while _message_factory.enabled:
            time.sleep(1)

    except KeyboardInterrupt:
        _log.info("\nCtrl-C caught, exiting…")
    except Exception as e:
        _log.error('error in motor test: {}'.format(e))
        traceback.print_exc(file=sys.stdout)
    finally:
        if _clock:
            _clock.close()
        if _queue_publisher:
            _queue_publisher.close()
        if _gamepad_publisher:
            _gamepad_publisher.close()
        if _gamepad_subscriber:
            _gamepad_subscriber.close()
            
if __name__ == "__main__":
    main()

#EOF
