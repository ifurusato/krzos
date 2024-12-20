#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-11-24
# modified: 2024-11-25
#

import asyncio
from colorama import init, Fore, Style
init(autoreset=True)

import core.globals as globals # TEMP
globals.init()

from core.logger import Logger, Level
from core.event import Event, Group
from core.subscriber import Subscriber
from hardware.sound import Sound
from hardware.player import Player

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SoundSubscriber(Subscriber):
    CLASS_NAME = 'sound'
    '''
    A subscriber to many events, with some triggering sounds.

    :param config:       the application configuration
    :param message_bus:  the message bus
    :param level:        the logging level
    '''
    def __init__(self, config, message_bus, level=Level.INFO):
        Subscriber.__init__(self, SoundSubscriber.CLASS_NAME, config, message_bus=message_bus, suppressed=False, enabled=False, level=level)
        self.add_events([ Group.SYSTEM,
                Group.GAMEPAD,
                Group.STOP,
                Group.IMU,
                Group.BUMPER,
                Group.INFRARED,
                Group.BEHAVIOUR,
                Group.REMOTE
            ])
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _arbitrate_message(self, message):
        '''
        Pass the message on to the Arbitrator and acknowledge that it has been
        sent (by setting a flag in the message).
        '''
        await self._message_bus.arbitrate(message.payload)
        message.acknowledge_sent()
        _value = message.payload.value
#       self._log.info('arbitrated message ' + Fore.WHITE + '{} '.format(message.name)
#               + Fore.CYAN + 'for event \'{}\' with value type: '.format(message.event.name)
#               + Fore.YELLOW + '{}'.format(type(_value)))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _shutdown(self): # TEMP
        self._log.info(Fore.MAGENTA + 'shutdown from bumper.')
        _component_registry = globals.get('component-registry')
        _krzos = _component_registry.get('krzos')
        _krzos.shutdown()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def process_message(self, message):
        '''
        Process the message.

        :param message:  the message to process.
        '''
        if message.gcd:
            raise GarbageCollectedError('cannot process message: message has been garbage collected.')
        _event = message.event

#       SILENCE ARMING_TONE BEEP BEEP_HI BLIP BOINK BUZZ CHATTER CHIRP CHIRP_4 CHIRP_7 
#       DWERP EARPIT GLINCE GWOLP HONK HZAH IPPURT ITIZ IZIT PEW_PEW_PEW PIZZLE SKID_FZZT 
#       SONIC_BAT TELEMETRY TIKA_TIKA TSK_TSK_TSK TICK TWEAK TWIDDLE_POP TWIT WOW ZZT 

        match _event.group:
            case Group.SYSTEM:      #  1, "system" 
                self._log.info(Style.DIM + 'SYSTEM: message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))
            case Group.GAMEPAD:     #  3, "gamepad" 
                self._log.info(Style.DIM + 'GAMEPAD: message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))
            case Group.STOP:        #  4, "stop" 
                self._log.info(Style.DIM + 'STOP: message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))
            case Group.BUMPER:      #  5, "bumper" 
                Player.instance().play(Sound.HONK)
                self._shutdown()
                self._log.info(Style.BRIGHT + 'BUMPER: message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))
            case Group.INFRARED:    #  6, "infrared" 
                if _event is Event.INFRARED_PORT:
                    self._log.info(Fore.RED   + 'port infrared: {:d}mm'.format(int(message.payload.value)))
                    Player.instance().play(Sound.DIT_A)
                elif _event is Event.INFRARED_CNTR:
                    self._log.info(Fore.BLUE  + 'center infrared: {:d}mm'.format(int(message.payload.value)))
                    Player.instance().play(Sound.DIT_B)
                elif _event is Event.INFRARED_STBD:
                    self._log.info(Fore.GREEN + 'starboard infrared: {:d}mm'.format(int(message.payload.value)))
                    Player.instance().play(Sound.DIT_C)
            case Group.IMU:         #  7, "imu" 
                self._log.info(Style.DIM + 'IMU: message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))
            case Group.BEHAVIOUR:   # 11, "behaviour" 
                self._log.info(Style.DIM + 'BEHAVIOUR: message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))
            case Group.REMOTE:      # 14, "remote" 
                self._log.info(Style.DIM + 'REMOTE: message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))

        self._log.debug('pre-processing message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))
        await Subscriber.process_message(self, message)
        self._log.debug('post-processing message {}'.format(message.name))

#EOF
