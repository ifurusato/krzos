#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-25
# modified: 2025-09-12
#

import datetime as dt
from colorama import init, Fore, Style
init()

from core.component import Component
from core.orientation import Orientation
from core.logger import Logger, Level
from hardware.controller import Controller
from hardware.response import*

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class TinyFxController(Component):
    NAME = 'tinyfx'
    '''
    Connects with a Tiny FX over I2C.
    '''
    def __init__(self, config=None, level=Level.INFO):
        self._log = Logger(TinyFxController.NAME, level)
        self._log.info('instantiating TinyFxController…')
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        _cfg = config['kros'].get('hardware').get('tinyfx-controller')
        self._i2c_address        = _cfg.get('i2c_address')
        self._bus_number         = _cfg.get('bus_number')
        self._i2cbus             = None
        self._config_register    = 1
        self._max_payload_length = 32
        self._controller = Controller('tinyfx', i2c_bus=1, i2c_address=0x44)
        self._last_send_time = None  # timestamp of last send
        self._min_send_interval = dt.timedelta(milliseconds=100)  # 100ms minimum send interval
        self._log.info('ready at 0x{:02X} on I2C bus {}.'.format(self._i2c_address, self._bus_number))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def help(self):
        '''
        Print help.
        '''
        self.send_data('help')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def play(self, key):
        '''
        A convenience method to play the sound corresponding to the key,
        returning the Response.
        '''
        if key.startswith('play '):
            self._log.info("> '{}'".format(key))
            return self.send_data(key)
        else:
            self._log.debug("play: '{}'".format(key))
            return self.send_data('play {}'.format(key))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def channel_on(self, orientation):
        '''
        Accepts an Orientation argument. Modify the channel designated by:

            Orientation.NONE :  turn off all lights
            Orientation.ALL :   turn on port, starboard and mast lights
            Orientation.PORT :  turn on port light
            Orientation.STBD :  turn on starboard light
            Orientation.MAST :  turn on mast flashing light
        '''
        match orientation:
            case Orientation.NONE:
                return self.send_data('off')
            case Orientation.ALL:
                return self.send_data('on')
            case Orientation.PORT:
                return self.send_data('port')
            case Orientation.STBD:
                return self.send_data('stbd')
            case Orientation.MAST:
                return self.send_data('mast')
            case Orientation.PIR:
                return self.send_data('pir get')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def on(self):
        '''
        A shortcut that turns on all channels, returning the Response.
        '''
        self._log.info('lights on…')
        return self.channel_on(Orientation.ALL)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def off(self):
        '''
        A shortcut that turns off all channels, returning the Response.
        '''
        self._log.info('lights off…')
        return self.channel_on(Orientation.NONE)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def pir(self, enabled):
        '''
        Enables or disables the PIR sensor, returning the Response.
        '''
        if enabled:
            return self.send_data('pir on')
        else:
            return self.send_data('pir off')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def ram(self):
        '''
        Displays free RAM on the console, returning the Response.
        '''
        return self.send_data('ram')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def sounds(self):
        '''
        Displays the list of sounds, returning the Response.
        '''
        return self.send_data('sounds')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def flash(self):
        '''
        Displays flash memory info on the console, returning the Response.
        '''
        return self.send_data('flash')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def exit(self):
        '''
        Exits the main loop on the TinyFX, returning the Response.
        This will disable the I2C bus so don't do it.
        '''
        return self.send_data('exit')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def send_data(self, data):
        '''
        Sends a string to the TinyFX.
        '''
        start_time = dt.datetime.now()
        if self._last_send_time:
            elapsed = start_time - self._last_send_time
            if elapsed < self._min_send_interval:
                self._log.warning(
                    "write_payload skipped: only {:.1f}ms since last send (minimum is {:.1f}ms)".format(
                        elapsed.total_seconds() * 1000,
                        self._min_send_interval.total_seconds() * 1000
                    )
                )
                return RESPONSE_SKIPPED
        self._log.info("sending data: '{}'…".format(data))
        _response = self._controller.send_payload(data)
        elapsed_ms = (dt.datetime.now() - start_time).total_seconds() * 1000.0
        if _response is None:
            raise ValueError('null response.')
        elif isinstance(_response, Response):
            if _response == RESPONSE_OKAY:
                self._log.info("response: "
                        + Fore.GREEN + "'{}'".format(_response.description)
                        + Fore.CYAN + "; {:5.2f}ms elapsed.".format(elapsed_ms))
            else:
                self._log.warning("response: "
                        + Fore.RED + "'{}'".format(_response.description)
                        + Fore.WHITE + "; {:5.2f}ms elapsed.".format(elapsed_ms))
        elif not isinstance(_response, Response):
            raise ValueError('expected Response, not {}.'.format(type(_response)))
        else:
            self._log.error("error response: {}; {:5.2f}ms elapsed.".format(_response.description, elapsed_ms))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        Component.close(self)
#       self.send_data('off')
        self._log.info('closed.')

#EOF
