#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-24
# modified: 2025-07-02

import uasyncio as asyncio
import time
from pyb import LED, Pin, UART
from pyb import LED
from colorama import Fore, Style

from ucomponent import Component
from logger import Logger, Level
from fsm import IllegalStateError
from payload import Payload

class UartSlaveBase(Component):
    def __init__(self, name, uart_id=1, baudrate=115200, status=None):
        self._log = Logger(name, Level.INFO)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self.baudrate    = baudrate
        self._status      = status
        self._buffer     = bytearray()
        self._last_rx    = time.ticks_ms()
        self._timeout_ms = 250
        self._verbose    = False
        self._led        = LED(1)
        try:
            self._uart = UART(uart_id)
            self._uart.init(baudrate=baudrate, bits=8, parity=None, stop=1)
            if self._status:
                self._status.ready()
            self._log.info(Fore.GREEN + 'UART{} slave ready at {:,} baud.'.format(uart_id, baudrate))
        except Exception as e:
            self._signal_error()
            self._log.info('{} raised setting up UART{} ready at {:,} baud: {}'.format(type(e), uart_id, baudrate, e))

    def set_verbose(self, verbose: bool):
        self._verbose = verbose

    def _signal_error(self):
        self._log.error('error.')
        if self._status:
            self._status.error()

    async def receive_packet(self):
        if not self.enabled:
            raise IllegalStateError('not enabled.')
        while True:
            if self._uart.any():
                # read all available bytes at once
                data = self._uart.read(self._uart.any())
                self._buffer += data
                self._last_rx = time.ticks_ms()
                if self._verbose:
                    self._log.debug("read {} bytes, buffer size now {}".format(len(data), len(self._buffer)))
            else:
                # timeout: clear buffer to avoid garbage growth
                if self._buffer and time.ticks_diff(time.ticks_ms(), self._last_rx) > self._timeout_ms:
                    self._log.error("UART RX timeout; clearing buffer…")
                    self._buffer = bytearray()
                await asyncio.sleep(0) # was 0.005
                continue
            # check if buffer starts with SYNC_HEADER (avoid .find if possible)
            if self._buffer.startswith(Payload.SYNC_HEADER):
                if len(self._buffer) >= Payload.PACKET_SIZE:
                    packet = self._buffer[:Payload.PACKET_SIZE]
                    self._buffer = self._buffer[Payload.PACKET_SIZE:]
                    try:
                        _payload = Payload.from_bytes(packet)
                        if self._verbose:
                            self._log.info('rx: ' + Fore.GREEN + '{}'.format(_payload))
                        self._led.on()
                        return _payload
                    except Exception:
                        # corrupt packet: remove SYNC_HEADER and resync
                        self._signal_error()
                        self._log.error("packet decode error: {}. resyncing…".format(e))
                        self._buffer = self._buffer[1:]
                        continue
                else:
                    # not enough data yet for a full packet
                    await asyncio.sleep(0)
                    continue
            else:
                # slow-path: search for SYNC_HEADER
                idx = self._buffer.find(Payload.SYNC_HEADER)
                if idx == -1:
                    # keep only enough bytes to possibly contain the next header
                    if len(self._buffer) > len(Payload.SYNC_HEADER):
                        self._buffer = self._buffer[-(len(Payload.SYNC_HEADER) - 1):]
                    await asyncio.sleep(0)
                    continue
                else:
                    # discard bytes up to found SYNC_HEADER
                    self._buffer = self._buffer[idx:]
                    await asyncio.sleep(0)
                    continue

    async def send_packet(self, payload: Payload):
        if not self.enabled:
            raise IllegalStateError('not enabled.')
        try:
            packet = payload.to_bytes()
            if not packet.startswith(Payload.SYNC_HEADER):
                packet = Payload.SYNC_HEADER + packet[len(Payload.SYNC_HEADER):]
            self._uart.write(packet)
            if self._verbose:
                self._log.info(Style.DIM + "tx: " + Fore.GREEN + 'AK')
            self._led.off()
        except Exception as e:
            self._signal_error()
            self._log.error("failed to send packet: {}".format(e))

    def close(self):
        Component.close(self)
        self._log.info("closed.")

#EOF
