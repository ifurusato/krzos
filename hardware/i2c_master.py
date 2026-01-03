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
from datetime import datetime as dt, timezone
import smbus2
import traceback
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from upy.message_util import pack_message, unpack_message

class I2CMaster(Component):
    I2C_BUS_ID  = 1     # the I2C bus number; on a Raspberry Pi the default is 1
    I2C_ADDRESS = 0x43  # the default I2C address
    '''
    Abstract base class for an I2C master controller.
    '''
    def __init__(self, log_or_name=None, i2c_bus_id=None, i2c_address=None, timeset=True, level=Level.INFO):
        if log_or_name is None:
            raise ValueError('required logger or class name.')
        elif isinstance(log_or_name, str):
            self._log = Logger(log_or_name, level=level)
        elif isinstance(log_or_name, Logger):
            self._log = logger
        else:
            raise ValueError('expected logger or class name, not {}'.format(type(log_or_name)))
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._i2c_bus_id  = i2c_bus_id if i2c_bus_id else I2CMaster.I2C_BUS_ID
        self._i2c_address = i2c_address if i2c_address else I2CMaster.I2C_ADDRESS
        self._timeset = timeset
        try:
            self._bus = smbus2.SMBus(self._i2c_bus_id)
            self._log.info('opening I2C bus {} at address {:#04x}'.format(self._i2c_bus_id, self._i2c_address))
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
#           print('read {} bytes: {}'.format(len(resp_buf), ' '.join('{:02x}'.format(b) for b in resp_buf[: 16])))
            if resp_buf and len(resp_buf) >= 2:
                msg_len = resp_buf[0]
                if 1 <= msg_len <= 30:
                    resp_bytes = bytes(resp_buf[: msg_len+2])
                    return resp_bytes
            time.sleep(0.003)
        raise RuntimeError("bad message length or slave not ready.")

    def send_request(self, message):
        '''
        Send a message and return the response.
        '''
        if self.enabled:
            if message.startswith('time set'):
#               now = dt.now() # as local time
                now = dt.now(timezone.utc) # as UTC time
                self._log.info(Fore.GREEN + 'setting time to: {}'.format(now.isoformat()))
                ts = now.strftime("%Y%m%d-%H%M%S")
                message = message.replace("now", ts)
            out_msg = pack_message(message)
            try:
                resp_bytes = self._i2c_write_and_read(out_msg)
                return unpack_message(resp_bytes)
            except Exception as e:
#               self._log.error('{} raised by send request: {}\n{}'.format(type(e), e, traceback.format_exc()))
                self._log.error('{} raised by send request: {}'.format(type(e), e))
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

    def enable(self):
        '''
        Enable the I2CMaster.
        '''
        if not self.enabled:
            super().enable()
            time.sleep(0.3)
            if self._timeset:
                self._log.info(Fore.YELLOW + 'setting RTC time…')
                self.send_request('time set now')
        else:
            self._log.warning('already enabled.')

    def disable(self):
        '''
        Disable the I2CMaster.
        '''
        if self.enabled:
            super().disable()
        else:
            self._log.debug('already disabled.')

    def close(self):
        '''
        Disable and close the I2CMaster.
        '''
        if not self.closed:
            super().close()
            self._bus.close()
        else:
            self._log.warning('already closed.')

#EOF
