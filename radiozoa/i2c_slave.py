#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-02-07
#
# I2C slave using single memory buffer for ESP32-S3.

import sys
import time
from machine import Pin, I2CTarget

from message_util import pack_message, unpack_message

class I2CSlave:
    MEM_LENGTH = 64
    # pre-packed constant responses
    PACKED_ACK  = pack_message('ACK')
    PACKED_ERR  = pack_message('ERR')
    '''
    Memory-based I2C slave with proper separation of RX and TX data.
    '''
    def __init__(self, i2c_id, scl, sda, i2c_address):
        self._i2c_id      = i2c_id
        self._scl_pin     = Pin(scl) if scl else None
        self._sda_pin     = Pin(sda) if sda else None
        self._i2c_address = i2c_address
        if self._scl_pin is not None:
            print('I2C slave configured with SDA on pin {}, SCL on pin {}'.format(sda, scl))
        else:
            print('I2C slave configured using configured values.')
        self._i2c = None
        self._mem_buf = bytearray(I2CSlave.MEM_LENGTH)
        self._rx_copy = bytearray(I2CSlave.MEM_LENGTH)
        self._callback = None
        self._new_cmd = False
        self._processing = False
        # initialize with ACK
        init_msg = I2CSlave.PACKED_ACK
        for i in range(len(init_msg)):
            self._mem_buf[i] = init_msg[i]
        print('I2C slave ready.')

    def enable(self):
        i2c_id = self._i2c_id
        if self._scl_pin is not None:
            self._i2c = I2CTarget(i2c_id, self._i2c_address, mem=self._mem_buf, scl=self._scl_pin, sda=self._sda_pin)
            print('I2C slave enabled on I2C{} address {:#04x}'.format(i2c_id, self._i2c_address))
        else:
            self._i2c = I2CTarget(i2c_id, self._i2c_address, mem=self._mem_buf)
            print('I2C slave enabled on I2C{} address {:#04x}'.format(i2c_id, self._i2c_address))
        self._i2c.irq(self._irq_handler, trigger=I2CTarget.IRQ_END_WRITE, hard=False)

    def disable(self):
        if self._i2c:
            self._i2c.deinit()
            self._i2c = None

    def add_callback(self, callback):
        self._callback = callback

    def _irq_handler(self, i2c):
        '''
        The IRQ handler used on the ESP32 and RP2.
        '''
        flags = i2c.irq().flags()
        if flags & I2CTarget.IRQ_END_WRITE:
            msg_len = self._mem_buf[0]
            if msg_len > 0 and msg_len < 60:
                for i in range(msg_len + 2):
                    self._rx_copy[i] = self._mem_buf[i]
                self._new_cmd = True

    def check_and_process(self):
        if self._new_cmd and not self._processing:
            self._new_cmd = False
            self._processing = True
            msg_len = self._rx_copy[0]
            try:
                rx_bytes = bytes(self._rx_copy[:msg_len + 2])
                cmd = unpack_message(rx_bytes)
                if self._callback:
                    resp_bytes = self._callback(cmd)
                    if not resp_bytes:
                        resp_bytes = I2CSlave.PACKED_ACK
                else:
                    resp_bytes = I2CSlave.PACKED_ACK
            except Exception as e:
                print("ERROR: {} raised: {} [1]".format(type(e), e))
                resp_bytes = I2CSlave.PACKED_ERR
            try: 
                for i in range(len(resp_bytes)):
                    self._mem_buf[i] = resp_bytes[i]
                for i in range(len(resp_bytes), I2CSlave.MEM_LENGTH):
                    self._mem_buf[i] = 0
            except Exception as e:
                print("ERROR: {} raised: {} [2]".format(type(e), e))
            finally:
                self._processing = False

#EOF
