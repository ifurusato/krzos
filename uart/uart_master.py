#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-12
# modified: 2025-06-23

import time
import traceback
import itertools
from typing import Callable, Optional
from datetime import datetime as dt
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
    '''
    Uses UART 4 on port /dev/ttyAMA0 as the default.
    '''
    def __init__(self, config, port='/dev/ttyAMA0'):
        self._log = Logger('uart-master', Level.INFO)
        _cfg = config['kros']['uart_master']
        _port          = _cfg.get('port')
        _baudrate      = _cfg.get('baud_rate')     #, 1_000_000) # 115200 460800 921600
        _tx_timeout_ms = _cfg.get('tx_timeout_ms') # 10ms
        _rx_timeout_ms = _cfg.get('rx_timeout_ms') # 27ms
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
#       self._log.info(f"MASTER TX BYTES: {packet_bytes.hex(' ')}") # TEMP
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

#   def send_receive_payload(self, payload):
#       '''
#       Accept a Payload, send it, then wait for the response and return the Payload result.
#       This method can be used without needing to run the full loop. If an error occurs
#       this returns the ERROR_PAYLOAD.
#       '''
#       self.send_payload(payload)
#       try:
#           response_payload = self.receive_payload()
#           return response_payload
#       except ValueError as e:
#           self._log.error("error during communication: {}".format(e))
#           return self.ERROR_PAYLOAD

    def run(self,
                command_source: Optional[Callable[[], int]] = None,
                speed_source: Optional[Callable[[], int]] = None,
                delay_sec=0):
        '''
        Main loop for communication with elapsed time measurement. This is currently
        used for testing but could easily be modified for continuous use.
        '''
        try:
            if speed_source is None:
                self._log.info("speed source not provided, using counter.")
            else:
                self._log.info("using speed source for data.")
            # initial values
            red = green = blue = 0.0
            counter = itertools.count()
            div     = 1
            code    = 'CO'
            speed   = 0.0
            enabled = True

            while enabled:
                if command_source is not None:
                    code = command_source()
                # create Payload with code (2 letters) and floats for pfwd, sfwd, paft, saft
                _color = Fore.CYAN
                _mode = Mode.from_code(code)
#               self._log.info(Fore.YELLOW + "mode: {}".format(_mode))
                if _mode == Mode.PING:
                    payload = Payload(code, 0.0, 0.0, 0.0, 0.0)
                else:
                    if speed_source is not None:
                        speed, red, green, blue = speed_source()
                    else:
                        speed = 0.0
                    match _mode:
                        case Mode.COLOR:
                            _color = Fore.BLUE
                            payload = Payload(code, red, green, blue, 0.0)
                        case Mode.IP_ADDRESS:
                            _color = Fore.YELLOW
                            payload = Payload(code, *self._ip_address)
                        case Mode.STOP:
                            _color = Fore.RED
                            payload = Payload(code, 0.0, 0.0, 0.0, 0.0)
                        case Mode.GO:
                            _color = Fore.GREEN
                            payload = Payload(code, speed, speed, speed, speed)

                        case Mode.REQUEST:
                            # TODO
                            _color = Fore.WHITE
                            payload = Payload(code, 0.0, 0.0, 0.0, 0.0)
                        case Mode.ACK:
                            # TODO
                            payload = Payload(code, 0.0, 0.0, 0.0, 0.0)
                        case Mode.ENABLE:
                            payload = Payload(code, 0.0, 0.0, 0.0, 0.0)
                        case Mode.DISABLE:
                            _color = Fore.BLACK
                            payload = Payload(code, 0.0, 0.0, 0.0, 0.0)
                        case Mode.ERROR:
                            _color = Fore.MAGENTA
                            payload = Payload(code, -1.0, -1.0, -1.0, -1.0)
#                       case Mode.ROT_CW:
#                       case Mode.ROT_CCW:
#                       case Mode.CRAB_PORT:
#                       case Mode.CRAB_STBD:
#                       case Mode.DIA_PFWD:
#                       case Mode.DIA_SFWD:
#                       case Mode.DIA_PREV:
#                       case Mode.DIA_SREV:
                        case _:
                            payload = Payload(code, speed, speed, -speed, -speed)
                    if payload == self._last_payload: # then don't bother
                        continue
                self._last_payload = payload
                start_time = dt.now()
                self.send_payload(payload)
                try:
                    value = self.receive_payload()
                    if isinstance(value, int):
                        speed = value
                except TooManyErrors:
                    self._log.error("cannot continue: too many errors.")
                    enabled = False
                except ValueError as e:
                    self._log.error("error receiving payload: {}:".format(e))
                    continue  # optionally, continue the loop without stopping
                # calculate elapsed time
                elapsed_time = (dt.now() - start_time).total_seconds() * 1000  # Convert to milliseconds

                if next(counter) % div == 0: # every 10th time
                    self._log.info(_color + "{} / {:>3}".format(code, speed) + Fore.CYAN + "; tx: {:.2f} ms elapsed.".format(elapsed_time))
                # with no sleep here, would be running as fast as the system allows
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
