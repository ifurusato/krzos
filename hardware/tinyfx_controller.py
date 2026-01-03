#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-01-03

import time
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.orientation import Orientation
from hardware.i2c_master import I2CMaster

class TinyFxController(I2CMaster):
    NAME = 'tinyfx-ctrl'
    I2C_BUS_ID  = 1
    I2C_ADDRESS = 0x43
    '''
    Extends I2CMaster to control a Pimoroni TinyFX.
    '''
    def __init__(self, config=None, i2c_address=None, timeset=True, level=Level.INFO):
        if config:
            _cfg = config.get('kros').get('hardware').get('tinyfx-controller')
            _i2c_bus_id  = _cfg.get('i2c_bus_id')
            _i2c_address = _cfg.get('i2c_address')
        else:
            _i2c_bus_id  = TinyFxController.I2C_BUS_ID
            _i2c_address = TinyFxController.I2C_ADDRESS if i2c_address is None else i2c_address
        I2CMaster.__init__(self, log_or_name=TinyFxController.NAME, i2c_bus_id=_i2c_bus_id, i2c_address=_i2c_address, timeset=timeset, level=level)
        # ready

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def play(self, name):
        '''
        Play the sound, returning the response ('ACK').
        '''
        return self.send_request('play {}'.format(name))

    def pir(self):
        '''
        Return the number of seconds since the PIR sensor was triggered.
        This returns -1 if it has never been triggered, -2 if an error
        occurs and the value is therefore unavailable.
        '''
        try:
            response = self.send_data_request('pir')
            self._log.info('pir response: {}'.format(response))
            elapsed_sec = int(response)
            self._log.info('pir response as int: {}sec'.format(elapsed_sec))
            return elapsed_sec
        except Exception as e:
            self._log.error('{} raised getting pir response: {}'.format(type(e), e))
            return -2

    def off(self):
        '''
        Turn off all running lights.
        '''
        response = self.send_request('all off')
        self._log.debug('all off response: {}'.format(response))

    def running_lights(self, enable):
        '''
        Turn the running lights (PORT, STBD and MAST) on or off.
        '''
        response = self.send_request('run {}'.format('on' if enable else 'off'))
        self._log.info(Style.DIM + 'response: {}'.format(response))

    def light(self, orientation, enable):
        '''
        Turn the port running lights on or off.
        '''
        self._log.debug('light {}: enable? {}'.format(orientation.label, enable))
        match orientation:
            case Orientation.AFT: # flashing backup light
                response = self.send_request('ch1 {}'.format('on' if enable else 'off'))
            case Orientation.FWD: # headlight
                response = self.send_request('ch2 {}'.format('on' if enable else 'off'))
            case Orientation.INT: # internal light
                response = self.send_request('ch3 {}'.format('on' if enable else 'off'))
            case Orientation.MAST:
                response = self.send_request('ch4 {}'.format('on' if enable else 'off'))
            case Orientation.PORT:
                response = self.send_request('ch5 {}'.format('on' if enable else 'off'))
            case Orientation.STBD:
                response = self.send_request('ch6 {}'.format('on' if enable else 'off'))
            case _: # ignore
                response = 'ERR'
        if response != 'ACK' and response != 'ERR':
            self._log.info(Style.DIM + '{} response: {}'.format(orientation.label, response))

    def enable(self):
        '''
        Enable the TinyFxController.
        '''
        if not self.enabled:
            super().enable()
            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def disable(self):
        '''
        Turn off the running lights and disable the TinyFxController.
        '''
        if self.enabled:
            self.off()
            super().disable()
        else:
            self._log.debug('already disabled.')

    def close(self):
        '''
        Disable and close the TinyFxController.
        '''
        if not self.closed:
            super().close()
            self._bus.close()
        else:
            self._log.warning('already closed.')

#EOF
