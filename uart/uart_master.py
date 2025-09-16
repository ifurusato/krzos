#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-12
# modified: 2025-09-16

import time
import traceback
import itertools
from typing import Callable, Optional
from datetime import datetime as dt, timedelta
from colorama import init, Fore, Style
init()

from uart.async_uart_manager import AsyncUARTManager
from uart.sync_uart_manager import SyncUARTManager
from uart.too_many_errors import TooManyErrors
from upy.mode import Mode
from upy.payload import Payload
from hardware.value_provider import DigitalPotSpeedProvider, RotaryEncoderCommandProvider
from hardware.ip_util import get_ip_address
from core.logger import Logger, Level

class UARTMaster:
    ERROR_PAYLOAD = Payload("ER", -1.0, -1.0, -1.0, -1.0) # singleton error payload
    PING_PAYLOAD  = Payload("PN", 0.0, 0.0, 0.0, 0.0)
    '''
    Uses UART 4 on port /dev/ttyAMA0 as the default.
    '''
    def __init__(self, config, port='/dev/ttyAMA0'):
        self._log = Logger('uart-master', Level.INFO)
        _cfg = config['kros']['uart_master']
        _port          = _cfg.get('port')
        _baudrate      = _cfg.get('baud_rate')
        _tx_timeout_ms = _cfg.get('tx_timeout_ms')
        _rx_timeout_ms = _cfg.get('rx_timeout_ms')
        use_async_mgr = _cfg.get('use_async_mgr')
        if use_async_mgr:
            self.uart = AsyncUARTManager(port=_port, baudrate=_baudrate, tx_timeout_ms=_tx_timeout_ms, rx_timeout_ms=_rx_timeout_ms)
        else:
            self.uart = SyncUARTManager(port=_port, baudrate=_baudrate, tx_timeout_ms=_tx_timeout_ms, rx_timeout_ms=_rx_timeout_ms)
        self.uart.open()
        self._hindered = False
        self._log.info(Style.BRIGHT + 'UART master is {}.'.format('hindered' if self._hindered else 'unhindered'))
        self._last_tx  = None
        self._last_rx  = None
        self._last_payload = None
        self._verbose  = False
        self._ip_address = get_ip_address(True)
        self._log.info('UART master ready on port {} at baud rate: {:,}.'.format(_port, _baudrate))

    def send_payload(self, payload):
        '''
        Send a Payload object after converting it to bytes.
        '''
        packet_bytes = payload.to_bytes()
        self.uart.send_packet(payload)
        if self._verbose:
            if payload.code != self._last_tx:
                self._log.info(Fore.MAGENTA + "tx: {}".format(payload))
        self._last_tx = payload.code

    def receive_payload(self):
        '''
        Receive a Payload object.
        If the Mode is PING, returns the converted ping count as an int.
        '''
        payload = self.uart.receive_packet()
        if payload:
            if payload.code == Mode.PING.code: # display ping result
                return Payload.decode_to_int(*payload.values)
            elif self._verbose:
                if payload.code != self._last_rx:
                    self._log.info(Fore.MAGENTA + "rx: {}".format(payload))
            self._last_rx = payload.code
            return payload
        else:
            raise ValueError("no valid response received.")

    def run(self,
            command_source: Optional[Callable[[], int]] = None,
            speed_source: Optional[Callable[[], int]] = None,
            delay_sec=0,
            ping_interval_ms=1000):
        '''
        Main loop for communication with elapsed time measurement.
        If no non-identical payload is sent for ping_interval_ms, send a PING payload.
        '''
        try:
            if speed_source is None:
                self._log.info("speed source not provided, using counter.")
            else:
                self._log.info("using speed source for data.")
            red = green = blue = 0.0
            counter = itertools.count()
            div     = 1
            code    = 'CO'
            speed   = 0.0
            enabled = True

            last_payload_sent_time = dt.now()
            ping_mode_code = Mode.PING.code
            ping_interval = timedelta(milliseconds=ping_interval_ms)

            while enabled:
                start_time = dt.now()
                # keepalive PING payload if nothing sent 
                if start_time - last_payload_sent_time > ping_interval:
                    self.send_payload(self.PING_PAYLOAD)
                    last_payload_sent_time = start_time
                    elapsed_time = (dt.now() - start_time).total_seconds() * 1000  # ms
#                   if self._verbose:
                    self._log.info(Style.DIM + "PN / ---; tx: {:.2f} ms elapsed.".format(elapsed_time))
                else:
                    payload = None
                    if command_source is not None:
                        code = command_source()
                    _mode = Mode.from_code(code)
                    if speed_source is not None:
                        speed, red, green, blue = speed_source()
                    else:
                        speed = 0.0
                    match _mode:
                        case Mode.COLOR:
                            payload = Payload(code, red, green, blue, 0.0)
                        case Mode.IP_ADDRESS:
                            payload = Payload(code, *self._ip_address)
                        case Mode.STOP:
                            payload = Payload(code, 0.0, 0.0, 0.0, 0.0)
                        case Mode.GO:
                            payload = Payload(code, speed, speed, speed, speed)
                        case Mode.REQUEST:
                            payload = Payload(code, 0.0, 0.0, 0.0, 0.0)
                        case Mode.ACK:
                            payload = Payload(code, 0.0, 0.0, 0.0, 0.0)
                        case Mode.ENABLE:
                            payload = Payload(code, 0.0, 0.0, 0.0, 0.0)
                        case Mode.DISABLE:
                            payload = Payload(code, 0.0, 0.0, 0.0, 0.0)
                        case Mode.ERROR:
                            payload = Payload(code, -1.0, -1.0, -1.0, -1.0)
                        case Mode.PING:
                            payload = None
                        case _:
                            payload = Payload(code, speed, speed, -speed, -speed)
                    # only send the payload if it's changed since last time
                    if payload and payload != self._last_payload:
                        self._last_payload = payload
                        self.send_payload(payload)
                        last_payload_sent_time = dt.now()
                        try:
                            value = self.receive_payload()
                            if isinstance(value, int):
                                speed = value
                        except TooManyErrors:
                            self._log.error("cannot continue: too many errors.")
                            enabled = False
                        except ValueError as e:
                            self._log.error("error receiving payload: {}:".format(e))
                            continue
                        elapsed_time = (dt.now() - start_time).total_seconds() * 1000  # ms
                        if next(counter) % div == 0:
                            self._log.info("{} / {:>3}; tx: {:.2f} ms elapsed.".format(code, speed, elapsed_time))
                    # else: nothing to send, wait for next iteration
                if self._hindered:
                    print('hinder by {}s…'.format(delay_sec))
                    time.sleep(delay_sec)

        except KeyboardInterrupt:
            print('\n')
            self._log.info(Fore.YELLOW + "ctrl-c caught, exiting…")
        except Exception as e:
            self._log.error("{} raised in run loop: {}".format(type(e), e))
            traceback.print_exception(type(e), e, e.__traceback__)
        finally:
            self._log.info("closing…")
            self.uart.close()
            if command_source:
                if isinstance(command_source, RotaryEncoderCommandProvider):
                    command_source.close()
                else:
                    self._log.info("command source: {}".format(type(command_source)))
            if speed_source:
                if isinstance(speed_source, DigitalPotSpeedProvider):
                    speed_source.close()
                else:
                    self._log.info("speed source: {}".format(type(speed_source)))
            self._log.info("closed.")

#EOF
