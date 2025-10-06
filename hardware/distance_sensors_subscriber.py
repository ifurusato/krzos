#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-23
# modified: 2025-05-08
#

from colorama import init, Fore, Style
init(autoreset=True)

from core.logger import Logger, Level
from core.event import Event, Group
from core.subscriber import Subscriber

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DistanceSensorsSubscriber(Subscriber):
    CLASS_NAME = 'distance'
    '''
    A subscriber to distance sensor events.

    :param config:       the application configuration
    :param message_bus:  the message bus
    :param level:        the logging level
    '''
    def __init__(self, config, message_bus, level=Level.INFO):
        Subscriber.__init__(self, DistanceSensorsSubscriber.CLASS_NAME, config, message_bus=message_bus, suppressed=False, enabled=False, level=level)
        self.add_events(Event.by_groups([Group.BUMPER, Group.INFRARED]))
        _cfg = config['krzos'].get('subscriber').get('distance_sensors')
        self._verbose = _cfg.get('verbose')
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_fore(self, event):
        match event:
            case Event.INFRARED_PORT:
                return Fore.RED
            case Event.BUMPER_PORT:
                return Fore.RED
            case Event.BUMPER_CNTR:
                return Fore.BLUE
            case Event.INFRARED_CNTR:
                return Fore.BLUE
            case Event.BUMPER_STBD:
                return Fore.GREEN
            case Event.INFRARED_STBD:
                return Fore.GREEN

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def is_bumper_event(event):
        return event.group is Group.BUMPER

    @staticmethod
    def is_infrared_event(event):
        return event.group is Group.INFRARED

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def process_message(self, message):
        '''
        Process the message.

        :param message:  the message to process.
        '''
        if message is None:
            raise ValueError('null message.')
        elif message.gcd:
            raise GarbageCollectedError('cannot process message: message has been garbage collected.')
        elif self._verbose:
            _event = message.event
            _value = message.payload.value
            if _event is None:
                raise TypeError('event is None.')
            elif _value is None:
                raise TypeError('event value is None.')
            elif DistanceSensorsSubscriber.is_bumper_event(_event):
                self._log.info(Style.BRIGHT + 'processing message:' + Fore.WHITE + ' {}; value: {}'.format(_event.name, self.format_value(_value)))
            elif DistanceSensorsSubscriber.is_infrared_event(_event):
                self._log.info('processing message:' + Fore.WHITE + ' {}; value: {}'.format(_event.name, self.format_value(_value)))
            else:
                self._log.info(Style.DIM + 'processing message:' + Fore.WHITE + ' {}; value: {}'.format(_event.name, self.format_value(_value)))
        await Subscriber.process_message(self, message)

    @staticmethod
    def format_value(value):
        try:
            return "{:.2f}mm".format(float(value))
        except (ValueError, TypeError):
            return str(value)

#EOF
