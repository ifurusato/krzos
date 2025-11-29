#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-24
# modified: 2025-11-24
#
# A singleton class to make playing sounds simpler to call. It includes a
# hard-coded rate limiting of 3000ms. The Player is by default enabled but
# can be controlled thusly:
#
#     Player.instance.disable()
#
# Usage:
#
#     from player import Player
#         Player.play('name')
#

from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level
from core.rate_limited import rate_limited
from hardware.tinyfx_controller import TinyFxController

class Player(Component):
    NAME = 'player'
    _instance = None
    _initialized = False

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if not self._initialized:
            self._log = Logger(Player.NAME, level=Level.INFO)
            Component.__init__(self, self._log, suppressed=False, enabled=False)
            self._tinyfx_controller = TinyFxController(level=Level.INFO)
            self._verbose     = False
            self._initialized = True
            self.enable() # by default
            self._log.info('ready.')

    @classmethod
    def play(cls, name):
        '''
        Public entry point: Player.play('sound')
        '''
        inst = cls.get_instance()
        return inst._maybe_play_sound(name)

    @classmethod
    @property
    def instance(cls):
        return cls.get_instance()

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls()
        return cls._instance

    def _maybe_play_sound(self, name):
        '''
        Calls _play_sound(), which is rate-limited so this logs that the initial call was made.
        '''
        if self.enabled:
            if self._verbose:
                self._log.info(Style.DIM + 'calling play: ' + Fore.WHITE + Style.BRIGHT + "'{}'".format(name))
            self._play_sound(name)
        else:
            self._log.warning('not enabled.')

    @rate_limited(3000)
    def _play_sound(self, name):
        '''
        Play the specified sound via TinyFX controller.
        Rate-limited to prevent sounds from overlapping.

        :param name: the sound name to play
        '''
        self._log.info('play: ' + Fore.WHITE + Style.BRIGHT + "'{}'".format(name))
        response = self._tinyfx_controller.send_request('play {}'.format(name))
        if self._verbose:
            if response == 'OK':
                self._log.info(Style.DIM + 'response: {}'.format(response))
            else:
                self._log.info('response: {}'.format(response))

    def enable(self):
        '''
        Enable the sensor, setting up the GPIO pin.
        '''
        if not self.enabled:
            if self._tinyfx_controller:
                self._tinyfx_controller.enable()
            Component.enable(self)
            self._log.info('enabled.')
        else:
            self._log.debug('already enabled.')

    def disable(self):
        '''
        Disable the sensor and clean up resources.
        '''
        if self.enabled:
            if self._tinyfx_controller:
                self._tinyfx_controller.disable()
            Component.disable(self)

    def close(self):
        '''
        Stop the loop if running, then close the sensor.
        '''
        if not self.closed:
            if self._tinyfx_controller:
                self._tinyfx_controller.close()
            Component.close(self)

#EOF
