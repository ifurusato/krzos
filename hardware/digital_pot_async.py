#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-09-19
# modified: 2025-06-09
#
# A simplified, asynchronous version of the DigitalPotentiometer class, used
# for testing.
#

import time
import asyncio
import traceback
import threading
import colorsys
from math import isclose
from colorama import init, Fore, Style
init()

import ioexpander as io
from core.logger import Logger, Level

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DigitalPotentiometer:
    I2C_ADDR   = 0x0C
    PIN_RED    = 1
    PIN_GREEN  = 7
    PIN_BLUE   = 2
    POT_ENC_A  = 12
    POT_ENC_B  = 3
    POT_ENC_C  = 11
    BRIGHTNESS = 0.5
    PERIOD     = int(255 / BRIGHTNESS)

    def __init__(self, fps=30, multiplier=100.0, level=Level.INFO):
        self._log = Logger('pot', level)
        self._max = 3.3
        self._multiplier = multiplier
        self._red   = 0
        self._green = 0
        self._blue  = 0
        self._fps   = fps
        self._task  = None
        self._loop  = None
        self._use_deadzone = True # if near 50% disable LED
        self._loop_thread = None
        self._stop_event = threading.Event()
        try:
            self._ioe = io.IOE(i2c_addr=DigitalPotentiometer.I2C_ADDR)
            self._ioe.set_mode(DigitalPotentiometer.POT_ENC_A, io.PIN_MODE_PP)
            self._ioe.set_mode(DigitalPotentiometer.POT_ENC_B, io.PIN_MODE_PP)
            self._ioe.set_mode(DigitalPotentiometer.POT_ENC_C, io.ADC)
            self._ioe.output(DigitalPotentiometer.POT_ENC_A, 1)
            self._ioe.output(DigitalPotentiometer.POT_ENC_B, 0)
            self._ioe.set_pwm_period(DigitalPotentiometer.PERIOD)
            self._ioe.set_pwm_control(divider=2)
            self._ioe.set_mode(DigitalPotentiometer.PIN_RED, io.PWM, invert=True)
            self._ioe.set_mode(DigitalPotentiometer.PIN_GREEN, io.PWM, invert=True)
            self._ioe.set_mode(DigitalPotentiometer.PIN_BLUE, io.PWM, invert=True)
            self._log.info("running LED with {} brightness steps.".format(int(DigitalPotentiometer.PERIOD * DigitalPotentiometer.BRIGHTNESS)))
            self._log.info('ready.')
        except Exception as e:
            self._log.info('{} raised: {}'.format(type(e), e))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def analog(self):
        return self._ioe.input(self.POT_ENC_C)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def value(self):
        '''
        Return the analog value (voltage) from the ADC pin.
        '''
        _value = max(0.0, min(self.analog, self._max))
        return 1.0 - (_value / self._max)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def normalised_value(self):
        '''
        Return a normalised int value between -100 and 100, or the 
        multiplier if changed in the constructor.
        '''
        raw_value = (self.value * 2) - 1 # scale 0..1 to -1..+1
        if self._use_deadzone and isclose(raw_value, 0.0, abs_tol=0.02):
            return 0
        else:
            return int(self._multiplier * raw_value)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def rgb(self):
        return self._red, self._green, self._blue

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def data(self):
        return self.normalised_value, self._red, self._green, self._blue

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _update(self):
        hue = max(0.0, min(self.analog / self._max, 1.0))
        if self._use_deadzone and isclose(hue, 0.5, abs_tol=0.01):
            self._red = self._green = self._blue = 0.0
            self._ioe.output(self.PIN_RED, 0)
            self._ioe.output(self.PIN_GREEN, 0)
            self._ioe.output(self.PIN_BLUE, 0)
        else:
            self._red, self._green, self._blue = [int(c * self.PERIOD * self.BRIGHTNESS)
                       for c in colorsys.hsv_to_rgb(hue, 1.0, 1.0)]
            self._ioe.output(self.PIN_RED, self._red)
            self._ioe.output(self.PIN_GREEN, self._green)
            self._ioe.output(self.PIN_BLUE, self._blue)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def start(self):
        if self._task is not None:
            self._log.warning('already running.')
            return # already running
        def run_loop():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            async def runner():
                self._task = asyncio.create_task(self._run())
                await self._task # wait for _run() to complete
            try:
                self._loop.run_until_complete(runner())
            except Exception as e:
                self._log.error('{} raised in pot run loop: {}\n{}'.format(type(e), e, traceback.format_exc()))
            finally:
                self._loop.close()
        self._stop_event.clear()
        self._loop_thread = threading.Thread(target=run_loop, daemon=True)
        self._log.info('starting loop…')
        self._loop_thread.start()

    async def _run(self):
        delay = 1.0 / self._fps
        while not self._stop_event.is_set():
            self._update()
            await asyncio.sleep(delay)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def off(self):
        self._log.info('off.')
        self._ioe.output(self.PIN_RED, 0)
        self._ioe.output(self.PIN_GREEN, 0)
        self._ioe.output(self.PIN_BLUE, 0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        if self._task is None:
            return # not running
        self._stop_event.set()
        # wait for the loop thread to finish gracefully
        if self._loop_thread:
            self._loop_thread.join()
        self._task = None
        self._loop = None
        self._loop_thread = None

#EOF
