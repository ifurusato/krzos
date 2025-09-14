#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-24
# modified: 2025-07-15

from datetime import datetime as dt, timedelta
import uasyncio as asyncio
from colorama import Fore, Style

from logger import Logger, Level
from payload import Payload
from mode import Mode
from pixel import Pixel
from display import Display
from colors import *

class PayloadRouter:
    ACK = Payload("AK", 0.0, 0.0, 0.0, 0.0)
    ERR = Payload("ER", 0.0, 0.0, 0.0, 0.0)
    NO_ERROR = 0.0

    def __init__(self, status, display, motor_controller):
        self._log = Logger('router', Level.INFO)
        self._motor_controller = motor_controller
        self._status         = status
        self._display        = display
        self._errors         = 0
        self._last_payload   = PayloadRouter.ACK # placeholder
        self._console        = True # use Display as console
        self._verbose        = False
        self._last_error     = PayloadRouter.NO_ERROR
        self._error_time     = None
        self._error_timeout  = timedelta(seconds=5)  # configurable timeout
        self._GENERATOR_TYPE = type((lambda: (yield))()) # the type indicating a coroutine
        # mode colors
        self._default_console       = Display.to_tft_color((0, 255, 255))   # CYAN
        self._mode_color_color      = Display.to_tft_color((67, 86, 255))   # BLUE
        self._mode_color_ip_address = Display.to_tft_color((255, 203, 0))   # YELLOW
        self._mode_color_stop       = Display.to_tft_color((255, 0, 0))     # RED
        self._mode_color_go         = Display.to_tft_color((0, 255, 0))     # GREEN
        self._mode_color_request    = Display.to_tft_color((255, 255, 255)) # WHITE
        self._mode_color_disable    = Display.to_tft_color((111, 55, 148))  # BLACK
        self._mode_color_error      = Display.to_tft_color((255, 0, 255))   # MAGENTA
        # define command handlers
        self._handlers = {
            # non-motion handlers
            Mode.COLOR:       lambda payload: self._handle_color(payload.rgb),           # color
            Mode.ENABLE:      lambda payload: self._motor_controller.enable(),           # enable
            Mode.PING:        lambda payload: self._handle_ping(),                       # ping (shouldn't be here)
            Mode.IP_ADDRESS:  lambda payload: self._display_ip_address(payload.speeds),  # display ip address
            # stopped   
            Mode.STOP:        lambda payload: self._motor_controller.go(payload),        # stop
            # all wheels forward/backward
            Mode.GO:          lambda payload: self._motor_controller.go(payload),        # go forward or reverse
            # rotation (spin in place)
            Mode.ROT_CW:      lambda payload: self._motor_controller.go(payload),        # rotate clockwise
            Mode.ROT_CCW:     lambda payload: self._motor_controller.go(payload),        # rotate counter-clockwise
            # crab movement (lateral strafe)  
            Mode.CRAB_PORT:   lambda payload: self._motor_controller.go(payload),        # crab to port
            Mode.CRAB_STBD:   lambda payload: self._motor_controller.go(payload),        # crab to starboard
            # crab movement (lateral strafe)  
            Mode.DIA_PFWD:    lambda payload: self._motor_controller.go(payload),        # diagonal forward to port
            Mode.DIA_SFWD:    lambda payload: self._motor_controller.go(payload),        # diagonal forward to starboard
            Mode.DIA_PREV:    lambda payload: self._motor_controller.go(payload),        # diagonal reverse to port
            Mode.DIA_SREV:    lambda payload: self._motor_controller.go(payload),        # diagonal reverse to starboard
            Mode.ACK:         lambda payload: self._handle_ack(),                        # acknowledge (for completeness)
            Mode.REQUEST:     lambda payload: self._handle_request(payload),             # request status
            Mode.DISABLE:     lambda payload: self._handle_disable(),                    # mock disable
            Mode.ERROR:       lambda payload: self._handle_error(payload),               # error state
        }
        self._check_event_loop()
        self._log.info('ready.')
            
    def _check_event_loop(self):
        try:
            loop = asyncio.get_event_loop()
            # try scheduling a no-op coroutine to confirm the loop is "usable"
            loop.create_task(self._noop())
            self._log.info('asyncio loop is running.')
        except Exception:
            self._log.warning('no asyncio event loop running: payloads will not be routed.')

    async def _noop(self):
        self._log.info(Style.DIM + 'noop: asyncio loop is running.')
        pass

    def get_error_info(self):
        '''
        Returns two floats: a timestamp (as a float) followed by an error number.
        If there has been no error the timestamp will be None, the error number 0.0.
        '''
        _error_time = self._error_time.timestamp() if self._error_time is not None else None
        return ( _error_time, self._last_error )

    def _record_error(self, error_msg):
        self._last_error = error_msg
        self._error_time = dt.now()

    def _has_recent_error(self):
        if self._last_error and self._error_time:
            return (dt.now() - self._error_time) < self._error_timeout
        return False

    def clear_error(self):
        self._last_error = PayloadRouter.NO_ERROR
        self._error_time = None

    def off(self):
        if self._status:
            self._status.off()

    async def route(self, payload):
        '''
        Routes the Payload to an appropriate recipient. If the Payload is
        identical to the previous one it is ignored.
        '''
        if payload is None:
            raise ValueError('null Payload.')
        elif not isinstance(payload, Payload):
            raise TypeError('expected a Payload, not a {}'.format(type(payload)))
        if payload != self._last_payload:
            asyncio.create_task(self._handle_payload(payload))
            self._last_payload = payload

    async def _handle_payload(self, payload):
        code = payload.code
        if self._verbose:
            if code != self._last_payload.code:
                self._log.info(Fore.MAGENTA + "route code: '{}' from payload: {}".format(code, payload))
        mode = Mode.from_code(code)
        if self._console:
            if code != 'IP': # fixed display, so don't overwrite with console
                color = self.get_color_for_code(code)
                self._motor_controller.pause()
                await self._display.console(payload.as_log(), color=color)
                self._motor_controller.resume()

        handler = self._handlers.get(mode)
        if handler:
            result = handler(payload)
            if isinstance(result, self._GENERATOR_TYPE):
                await result 
        else:
            raise ValueError('unhandled payload: {}'.format(payload))

    def get_color_for_code(self, code):
        mode = Mode.from_code(code)
        if mode is Mode.COLOR:
            return self._mode_color_color
        elif mode is Mode.GO:
            return self._mode_color_go
        elif mode is Mode.STOP:
            return self._mode_color_stop
        elif mode is Mode.IP_ADDRESS:
            return self._mode_color_ip_address
        elif mode is Mode.REQUEST:
            return self._mode_color_request
        elif mode is Mode.DISABLE:
            return self._mode_color_disable
        elif mode is Mode.ERROR:
            return self._mode_color_error
        else:
            return self._default_console

    def _handle_color(self, rgb):
        if self._verbose:
            self._log.info(Fore.MAGENTA + 'color: {}'.format(rgb))
        if self._status:
            self._status.rgb(color=rgb)

    def _handle_ack(self):
        self._log.info(Fore.GREEN + 'ACK')
        pass

    def _handle_disable(self):
        '''
        Mocked disable (we don't actually disable remotely).
        '''
        self._log.info(Style.DIM + 'disable (mock)')
        pass

    def _handle_request(self, payload):
        self._log.info('REQUEST. payload: {}'.format(payload))
        pass

    def _handle_ping():
        self._log.warning('ping at payload router.') # should be caught by app

    def _display_ip_address(self, octets):
        ip_string = '.'.join(str(int(x)) for x in octets)
        self._display.ip_address(ip_string)

    def _handle_error(self, payload):
        self._errors += 1
        self._log.error('ERROR. count: {}'.format(self._errors))

    def close(self):
        self.off()
        self._log.info('closed.')

#EOF
