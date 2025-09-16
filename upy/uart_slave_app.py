#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-23
# modified: 2025-09-16
#
# This is the entry point to the UART slave application.

import sys
import uasyncio as asyncio
from pyb import Pin, Timer
from colorama import Fore, Style

from logger import Logger, Level
from config_loader import ConfigLoader
from datetime import datetime as dt, timedelta
from payload import Payload
from pixel import Pixel
from status import Status
from payload_router import PayloadRouter
from motor_controller import MotorController
from mode import Mode
from colors import *
from stm32_uart_slave import Stm32UartSlave

import cwd
import free

class UartSlaveApp:

    ACK_PAYLOAD   = Payload(Mode.ACK.code, 0.0, 0.0, 0.0, 0.0)
    EMPTY_PAYLOAD = Payload("MT", 0.0, 0.0, 0.0, 0.0)

    def __init__(self):
        _config = ConfigLoader.configure('config.yaml')
        if _config is None:
            raise ValueError('failed to import configuration.')
        _cfg = _config['kros']['uart_slave_app']
        self._uart_id    = _cfg['uart_id'] # on RP2040 this should be 1
        self._verbose    = _cfg['verbose']
        self._log = Logger('uart{}_slave'.format(self._uart_id), Level.INFO)
        self._irq_freq_hz      = _cfg['irq_freq_hz']      # 100Hz
        self._irq_timer_number = _cfg['irq_timer_number'] # 15
        self._irq_pin_number   = _cfg['irq_pin_number']   # 'E4'
        _display_enabled = _config['kros']['display']['enable']
        if _display_enabled:
            from display import Display
            _rotation = _config['kros']['display']['rotation']
            self._display  = Display(_rotation)
        else:
            from mock_display import Display
            self._display  = Display()
        _pixel_enabled = _config['kros']['pixel']['enable']
        if _pixel_enabled:
            self._pixel = Pixel(_config, pixel_count=8, brightness=0.1)
            self._status   = Status(self._pixel)
        else:
            self._pixel = None
            self._status = None
        self._paused     = False
        self._last_command_time = dt.now()
        self._timeout_threshold = timedelta(seconds=1)
        self._slave      = None
        self._irq_timer  = None
        self._tx_count   = 0
        self._baudrate   = 1_000_000 # default
        self._motor_controller = MotorController(config=_config, status=self._status, level=Level.INFO)
        self._router     = PayloadRouter(self._status, self._display, self._motor_controller)
        self._display.hello()
        self._log.info('ready.')

    def rgb(self, color=None):
        if self._pixel:
            self._pixel.set_color(color)

    async def _pyb_wait_a_bit(self):
        from pyb import LED
        _led = LED(1)
        if self._status:
            self._status.off()
        for _ in range(3):
            self.rgb()
            _led.on()
            if self._status:
                self._status.ready()
            await asyncio.sleep_ms(100)
            if self._status:
                self._status.off()
            _led.off()
            await asyncio.sleep_ms(900)
        if self._status:
            self._status.off()
        _led.off()

    async def _setup_uart_slave(self):
        self._log.info(Fore.WHITE + 'waiting a bit…')
        await self._pyb_wait_a_bit()
        self._log.info(Fore.WHITE + 'done waiting.')
        self._log.info('configuring UART{} slave for STM32 Pyboard…'.format(self._uart_id))
        self._slave = Stm32UartSlave(uart_id=self._uart_id, baudrate=self._baudrate, status=self._status)
        self._slave.set_verbose(False)
        self._motor_controller.enable()
#       self._display.ready()
        self._log.info(Fore.GREEN + 'waiting for payload…')

    def _start_irq_timer(self):
        '''
        Starts a hardware timer on pin E4, used by the Raspberry Pi.
        '''
        self._log.info('starting {}Hz hardware timer {} on pin {}…'.format(self._irq_freq_hz, self._irq_timer_number, self._irq_pin_number))
        _pin_e4 = Pin(self._irq_pin_number, Pin.OUT_PP)
        self._irq_timer = Timer(self._irq_timer_number, freq=self._irq_freq_hz)
        self._irq_timer.channel(1, Timer.PWM, pin=_pin_e4, pulse_width_percent=50)

    async def run(self):
        self._start_irq_timer()
        await self._setup_uart_slave()
        self._slave.enable()
        try:
            while True:
                now = dt.now()
                print("🍏 now type:", type(now), "now value:", now)
                # check for timeout and pause motors if needed
                if not self._paused and now - self._last_command_time > self._timeout_threshold:
                    self._log.info("no packet received for {}s, stopping motors and entering paused state…".format(
                            self._timeout_threshold.total_seconds()))
                    self._router.stop()
                    self._paused = True
                    # do not update self._last_command_time here; will update when resuming

                _payload = await self._slave.receive_packet()
                if _payload is None:
                    if self._verbose:
                        self._log.warning("empty packet: {} (type: {})".format(_payload, type(_payload)))
                    await self._slave.send_packet(UartSlaveApp.EMPTY_PAYLOAD)
                elif isinstance(_payload, Payload):
                    self._tx_count += 1
                    if self._verbose:
                        self._log.info("payload: {}".format(_payload))

                    # any successfully processed packet indicates the UART is working, therefore:
                    # after a pause, any valid packet resumes the system and resets timeout
                    if self._paused:
                        self._log.info(Style.BRIGHT + "received packet after pause, resuming.")
                        self._paused = False

                    self._last_command_time = now  # Reset timeout for ANY valid packet
                    # process packet by Mode code:
                    if _payload.code == Mode.PING.code:
                        await self._slave.send_packet(UartSlaveApp.ACK_PAYLOAD)
                    elif _payload.code == Mode.REQUEST.code:
                        self._log.info(Fore.MAGENTA + "request status…")
                        timestamp, code = self._router.get_error_info()
                        if timestamp is not None:
                            status_payload = Payload(Mode.ERROR.code, timestamp, code, 0.0, 0.0)
                        else:
                            status_payload = UartSlaveApp.ACK_PAYLOAD
                        await self._slave.send_packet(status_payload)
                        self._router.clear_error()
                    elif _payload.code == Mode.COLOR.code:
                        # example non-movement mode, still processed
                        asyncio.create_task(self._router.route(_payload))
                        await self._slave.send_packet(UartSlaveApp.ACK_PAYLOAD)
                    elif _payload.code == Mode.IP_ADDRESS.code:
                        # IP address info, if implemented
                        asyncio.create_task(self._router.route(_payload))
                        await self._slave.send_packet(UartSlaveApp.ACK_PAYLOAD)
                    elif _payload.code == Mode.ENABLE.code or _payload.code == Mode.DISABLE.code:
                        # system enable/disable
                        asyncio.create_task(self._router.route(_payload))
                        await self._slave.send_packet(UartSlaveApp.ACK_PAYLOAD)
                    else:
                        # all other movement and control modes
                        asyncio.create_task(self._router.route(_payload))
                        await self._slave.send_packet(UartSlaveApp.ACK_PAYLOAD)
                else:
                    self._log.warning("no valid payload received: type: {}; value: {}".format(type(_payload), _payload))
        except KeyboardInterrupt:
            self._log.info("Ctrl-C caught, exiting application…")
        except Exception as e:
            self._log.error("{} raised in run loop: {}".format(type(e), e))
            sys.print_exception(e)
        finally:
            self.close()

    def close(self):
        self._log.info('closing…')
        if self._status:
            self._status.off()
        if self._motor_controller:
            self._motor_controller.close()
        if self._slave:
            self._slave.disable()
        if self._display:
            self._display.close()
        if self._irq_timer:
            self._irq_timer.deinit()
        self._log.info('closed.')

# for REPL usage or testing
def exec():
    app = None
    try:
        app = UartSlaveApp()
        asyncio.run(app.run())
    except KeyboardInterrupt:
        pass
    finally:
        if app:
            app.close()

if __name__ == "__main__":
    app = None
    try:
        app = UartSlaveApp()
        asyncio.run(app.run())
    except KeyboardInterrupt:
        pass
    finally:
        if app:
            app.close()

#EOF
