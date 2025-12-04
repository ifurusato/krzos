#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2025-11-29

import time
from datetime import datetime as dt
import smbus2
import traceback
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.orientation import Orientation
from tinyfx.message_util import pack_message, unpack_message

class TinyFxController(Component):
    NAME = 'tinyfx-ctrl'
    I2C_BUS  = 1      # the I2C bus number; on a Raspberry Pi the default is 1
    I2C_ADDR = 0x43   # the I2C address used to connect to the TinyFX

    def __init__(self, config=None, i2c_address=None, timeset=True, level=Level.INFO):
        self._log = Logger(TinyFxController.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        if config:
            _cfg = config.get('kros').get('hardware').get('tinyfx-controller')
            self._i2c_bus     = _cfg.get('i2c_bus')
            self._i2c_address = _cfg.get('i2c_address')
        else:
            # use defaults
            self._i2c_bus     = TinyFxController.I2C_BUS
            self._i2c_address = TinyFxController.I2C_ADDR
        if i2c_address:
            # arg overrides config
            self._i2c_address = i2c_address
        if self._i2c_bus is None:
            raise ValueError('no I2C bus number provided.')
        if self._i2c_address is None:
            raise ValueError('no I2C address provided.')
        self._timeset = timeset
        try:
            self._bus = smbus2.SMBus(self._i2c_bus)
            self._log.info('opening I2C bus {} at address {:#04x}'.format(self._i2c_bus, self._i2c_address))
            self._log.info('ready.')
        except Exception as e:
            self._log.error('{} raised opening smbus: {}'.format(type(e), e))
            raise

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _i2c_write_and_read(self, out_msg):
        if out_msg is None:
            raise ValueError('null message.')
        elif len(out_msg) == 0:
            self._log.warning('did not send empty message.')
            return
        self._bus.write_i2c_block_data(self._i2c_address, 0, list(out_msg))
        time.sleep(0.002)
        for _ in range(2):
            resp_buf = self._bus.read_i2c_block_data(self._i2c_address, 0, 32)
            # auto-detect and extract the real message
            if resp_buf and resp_buf[0] == 0 and len(resp_buf) > 2:
                # skip first byte, interpret the second as length
                msg_len = resp_buf[1]
                if 1 <= msg_len < 32:
                    resp_bytes = bytes(resp_buf[1:1+msg_len+2])
                    return resp_bytes
            else:
                msg_len = resp_buf[0]
                if 1 <= msg_len < 32:
                    resp_bytes = bytes(resp_buf[:msg_len+2])
                    return resp_bytes
            time.sleep(0.003)
        raise RuntimeError("bad message length or slave not ready.")

    def send_request(self, message):
        '''
        Send a message and return the response.
        '''
        if self.enabled:
            if message.startswith('time set'):
                now = dt.now()
                self._log.info(Fore.GREEN + 'setting time on TinyFX to: {}'.format(now.isoformat()))
                ts = now.strftime("%Y%m%d-%H%M%S")
                message = message.replace("now", ts)
            out_msg = pack_message(message)
            try:
                resp_bytes = self._i2c_write_and_read(out_msg)
                return unpack_message(resp_bytes)
            except Exception as e:
                self._log.error('I2C message error: {}\n{}'.format(e, traceback.format_exc()))
                return None
            finally:
                # don't repeat too quickly
                time.sleep(0.05)
        else:
            self._log.warning('cannot send request: disabled.')

    def send_data_request(self, message):
        '''
        Send a message and return the response as data.

        This is treated as a data request, causing three transactions to occur:
        1. The first sends the message but its response is thrown away (will be an ACK)
        2. The second retrieves the data
        3. The third clears the buffer

        This is due to a requirement on slave IRQ processing that limits the ability
        to return the status of the current transaction (meaning the response returned
        is always from the previous transaction).
        '''
        if self.enabled:
            try:
                # first transaction: send request (response discarded)
                data_request_bytes = self._i2c_write_and_read(pack_message(message))
                first_response = unpack_message(data_request_bytes)
                self._log.info(Style.DIM + "data request sent, ACK: '{}'".format(first_response))
                time.sleep(0.005)
                # second transaction: retrieve data
                response_bytes = self._i2c_write_and_read(pack_message('get'))
                second_response = unpack_message(response_bytes)
                self._log.info(Style.DIM + "data response: '{}'".format(second_response))
                time.sleep(0.005)
                # third transaction: clear buffer
                clear_bytes = self._i2c_write_and_read(pack_message('clear'))
                third_response = unpack_message(clear_bytes)
                self._log.info(Style.DIM + "buffer cleared: '{}'".format(third_response))
                # hopefully locate the correct response
                for candidate in (second_response, third_response):
                    if candidate not in ('ACK', 'ERR'):
                        return candidate
                return second_response

            except Exception as e:
                self._log.error('I2C data request error: {}\n{}'.format(e, traceback.format_exc()))
                return None
            finally:
                # don't repeat too quickly
                time.sleep(0.05)
        else:
            self._log.warning('cannot send data request: disabled.')
            return None

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
        self._log.info(Style.DIM + '{} response: {}'.format(orientation.label, response))

    def enable(self):
        '''
        Enable the TinyFxController.
        '''
        if not self.enabled:
            super().enable()
            time.sleep(0.3)
            if self._timeset:
                self._log.info(Fore.YELLOW + 'setting RTC time on TinyFX…')
                self.send_request('time set now')
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
        print('TinyFxController.close()             xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx ')
        if not self.closed:
            super().close()
            self._bus.close()
        else:
            self._log.warning('already closed.')

#EOF
