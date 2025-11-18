#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2025-11-18

import time
import smbus2
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.orientation import Orientation
from tinyfx.message_util import pack_message, unpack_message

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class TinyFxController(Component):
    NAME = 'tinyfx-ctrl'
    I2C_BUS  = 1      # the I2C bus number; on a Raspberry Pi the default is 1
    I2C_ADDR = 0x43   # the I2C address used to connect to the TinyFX

    def __init__(self, config=None, level=Level.INFO):
        self._log = Logger(TinyFxController.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._i2c_bus     = TinyFxController.I2C_BUS
        self._i2c_address = TinyFxController.I2C_ADDR
        self._bus = smbus2.SMBus(self._i2c_bus)
        self._log.info('opening I2C bus {} at address {:#04x}'.format(self._i2c_bus, self._i2c_address))
        self._log.info('ready.')

    def _i2c_write_and_read(self, out_msg):
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

    def send(self, message):
        if self.enabled:
            out_msg = pack_message(message)
            try:
                resp_bytes = self._i2c_write_and_read(out_msg)
                return unpack_message(resp_bytes)
            except Exception as e:
                self._log.error('I2C message error: {}'.format(e))
                return None
            finally:
                # don't repeat too quickly
                time.sleep(0.05)
        else:
            self._log.warning('disabled.')

    def off(self):
        '''
        Turn off all running lights.
        '''
        response = self.send('all off')
        self._log.info(Style.DIM + 'all off response: {}'.format(response))

    def light(self, orientation, enable):
        '''
        Turn the port running lights on or off.
        '''
        match orientation:
            case Orientation.PORT:
                response = self.send('ch5 {}'.format('on' if enable else 'off'))
            case Orientation.STBD:
                response = self.send('ch6 {}'.format('on' if enable else 'off'))
            case Orientation.MAST:
                response = self.send('ch4 {}'.format('on' if enable else 'off'))
            case _: # ignore
                response = 'ERR'
        self._log.info(Style.DIM + '{} response: {}'.format(orientation.label, response))

    def enable(self):
        '''
        Enable the TinyFxController.
        '''
        if not self.enabled:
            Component.enable(self)
            self._log.info('enabled.')
        else:
            self._log.debug('already enabled.')

    def disable(self):
        '''
        Turn off the running lights and disable the TinyFxController.
        '''
        if self.enabled:
            self.off()
            Component.disable(self)
        else:
            self._log.debug('already disabled.')

    def close(self):
        '''
        Disable and close the TinyFxController.
        '''
        if not self.closed:
            self.disable()
            self._bus.close()
            Component.close(self)
        else:
            self._log.debug('already closed.')

#EOF
