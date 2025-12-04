#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-23
# modified: 2025-08-04

import serial
import time
from colorama import init, Fore, Style
init()

from upy.payload import Payload
from uart.too_many_errors import TooManyErrors
from core.logger import Logger, Level

class SyncUARTManager:
    def __init__(self, port='/dev/serial0', baudrate=115200, tx_timeout_ms=10, rx_timeout_ms=27):
        self._log = Logger('sync-uart-mgr', Level.INFO)
        self._port         = port
        self._baudrate     = baudrate
        self._tx_timeout_s = tx_timeout_ms / 1000
        self._rx_timeout_s = rx_timeout_ms / 1000
        self._log.info('tx timeout: {}ms; rx timeout: {}ms'.format(tx_timeout_ms, rx_timeout_ms))
        self._serial       = None
        self._errors       = 0
        self._error_limit  = 500
        self._rx_buffer    = None # buffer for sync-header-based framing
        self._log.info('UART synchronous master ready on port {} at baud rate: {:,}.'.format(self._port, self._baudrate))

    def open(self):
        if self._serial is None or not self._serial.is_open:
            self._rx_buffer = bytearray()
            self._serial = serial.Serial(self._port, self._baudrate, timeout=self._tx_timeout_s)
            self._log.info("serial port {} opened at {} baud.".format(self._port, self._baudrate))

    def close(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._log.info("serial port closed.")
            self._serial = None

    def send_packet(self, payload):
        packet_bytes = payload.to_bytes()
        # check that sync header is present for robust protocol
        if not packet_bytes.startswith(Payload.SYNC_HEADER):
            packet_bytes = Payload.SYNC_HEADER + packet_bytes[len(Payload.SYNC_HEADER):]
        self._serial.write(packet_bytes)
        self._serial.flush()
        self._log.debug("sent: {}".format(repr(payload)))

    def receive_packet(self):
        '''
        Reads bytes, synchronizes on sync header, and returns the first valid Payload found.
        '''
        self._log.debug("receiving packet with timeout of {}ms…".format(1000 * self._rx_timeout_s))
        start_time = time.time()
        while True:
            if self._errors > self._error_limit:
                raise TooManyErrors('exceeded error limit in async uart manager!')
            if self._serial.in_waiting:
                data = self._serial.read(self._serial.in_waiting)
                self._rx_buffer += data
                start_time = time.time()
            idx = self._rx_buffer.find(Payload.SYNC_HEADER)
            if idx == -1:
                # not found: trim buffer if too large
                if len(self._rx_buffer) > len(Payload.SYNC_HEADER):
                    self._rx_buffer = self._rx_buffer[-(len(Payload.SYNC_HEADER)-1):]
                # tight loop, no sleep
                _elapsed_s = time.time() - start_time
#               if time.time() - start_time > self._rx_timeout_s:
                if _elapsed_s > self._rx_timeout_s:
                    self._errors += 1
                    self._log.error('UART RX timeout after {}ms with {} errors; sync header not found, clearing buffer…'.format(round(1000 * _elapsed_s), self._errors))
                    self._rx_buffer = bytearray()
                    start_time = time.time()
                continue
            self._log.debug(Style.DIM + "sync header: idx={}, {} bytes.".format(idx, len(self._rx_buffer)))
            # found sync header: do we have a full packet?
            if len(self._rx_buffer) - idx >= Payload.PACKET_SIZE:
                packet = self._rx_buffer[idx: idx + Payload.PACKET_SIZE]
                self._rx_buffer = self._rx_buffer[idx + Payload.PACKET_SIZE:]
                try:
                    payload = Payload.from_bytes(packet)
                    self._errors = 0 # success, we clear error count
                    return payload
                except ValueError as e:
                    self._errors += 1
                    self._log.error('receive error {}: {}. Resyncing…'.format(self._errors, e))
                    # remove just the first header byte to attempt resync
                    self._rx_buffer = self._rx_buffer[idx+1:]
                    self._errors = 0
                    continue
            else:
                # not enough bytes yet for a full packet
                # tight loop, no sleep
                if time.time() - start_time > self._rx_timeout_s:
                    self._errors += 1
                    self._log.error('UART RX timeout error {}; incomplete packet, clearing buffer…'.format(self._errors))
                    self._rx_buffer = bytearray()
                    start_time = time.time()
                continue

    def receive_values(self):
        '''
        Convenience method to receive a Payload and return the tuple (cmd, pfwd, sfwd, paft, saft).
        '''
        payload = self.receive_packet()
        if payload:
            return (payload.cmd, payload.pfwd, payload.sfwd, payload.paft, payload.saft)
        return None

#EOF
