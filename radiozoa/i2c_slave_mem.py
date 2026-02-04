#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-02-04
#
# I2C slave using single memory buffer for ESP32-S3

import sys
import time
from machine import Pin, I2CTarget

try:
    from upy.message_util import pack_message, unpack_message
except ImportError:
    from message_util import pack_message, unpack_message

from colorama import Fore, Style

__I2C_ID      = 1
__I2C_ADDRESS = 0x45
_I2C_PINS     = {
    0: (7,  6),        # ESP32
    1: ('B6',  'B7'),  # STM32F405
    2: ('B10', 'B11'), # STM32F405
}
_MEM_LENGTH = 64

class I2CSlave:
    '''
    Memory-based I2C slave with proper separation of RX and TX data.
    '''
    def __init__(self, i2c_id=None, i2c_address=None):
        self._i2c_id      = i2c_id if i2c_id is not None else __I2C_ID
        self._i2c_address = i2c_address if i2c_address else __I2C_ADDRESS
        _i2c_scl_pin, _i2c_sda_pin = _I2C_PINS[self._i2c_id]
        self._scl = Pin(_i2c_scl_pin)
        self._sda = Pin(_i2c_sda_pin)
        print('I2C slave configured for SDA on pin {}, SCL on pin {}'.format(_i2c_sda_pin, _i2c_scl_pin))
        self._i2c = None
        self._mem_buf = bytearray(_MEM_LENGTH)
        self._rx_copy = bytearray(_MEM_LENGTH)
        self._callback = None
        self._new_cmd = False
        self._processing = False
        # initialize with ACK
        init_msg = pack_message("ACK")
        for i in range(len(init_msg)):
            self._mem_buf[i] = init_msg[i]

    def enable(self):
        i2c_id = self._i2c_id
        self._i2c = I2CTarget(i2c_id, self._i2c_address, mem=self._mem_buf, scl=self._scl, sda=self._sda)
        self._i2c.irq(self._irq_handler, trigger=I2CTarget.IRQ_END_WRITE, hard=False)
        print(Fore.GREEN + 'I2C slave enabled on I2C{} address {:#04x}'.format(i2c_id, self._i2c_address) + Style.RESET_ALL)

    def disable(self):
        if self._i2c:
            self._i2c.deinit()
            self._i2c = None

    def add_callback(self, callback):
        self._callback = callback

    def _irq_handler(self, i2c):
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
                    response = self._callback(cmd)
                    if not response:
                        response = "ACK"
                else:
                    response = "ACK"
            except Exception as e:
                print("ERROR: {} during processing: {}".format(type(e), e))
                response = "ERR"
            try:
                resp_bytes = pack_message(str(response))
                for i in range(len(resp_bytes)):
                    self._mem_buf[i] = resp_bytes[i]
                for i in range(len(resp_bytes), _MEM_LENGTH):
                    self._mem_buf[i] = 0
            except Exception as e:
                print("ERROR: {} during response: {}".format(type(e), e))
            finally:
                self._processing = False

#EOF
