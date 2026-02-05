#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-26
# modified: 2024-11-17
#

import asyncio
import time
from datetime import datetime as dt
from pmw3901 import BG_CS_FRONT_BCM, PAA5100, PMW3901
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class OpticalFlowSensor(Component):
    '''
    A wrapper for the PAA5100JE or PMW3901 optical flow sensor, that uses a
    low-resolution camera and some clever algorithms to detect motion across
    surfaces. The PAA5100JE sensor has a range of 15-35mm, a frame rate of 
    242 FPS, FoV of 42°, and a maximum speed of 1.14 metres per second (with 
    sensor 25mm away from surface). By contrast, the PMW3901 has a range from
    80mm to infinity, and is more suited for tall robots or flying drones.

    In configuration there are X and Y trim values as multipliers on the values
    read by the sensor.

    Connections:

    * 3-5V to any 5V or 3V pin
    * CS to BCM 7
    * SCK to BCM 11
    * MOSI to BCM 10
    * MISO to BCM 9
    * INT to BCM 19
    * GND to any ground pin

    Values are kept internally as floats, returned as ints.

    Note: This was calibrated on a bamboo cutting board, and on other surfaces
    the value will likely vary.

    :param config:          the application configuration
    :param level:           the log level
    '''
    def __init__(self, config, level=Level.INFO):
        self._log = Logger('ofs', level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._log.info('initialising ofs…')
        _cfg = config['kros'].get('hardware').get('ofs')
        _rotation    = _cfg.get('rotation') # options: 0, 90, 180, 270; rotation of sensor in degrees
        _slot        = BG_CS_FRONT_BCM # front SPI slot
        # the class for the specified breakout (PWM3901 or PAA5100)
        _near        = _cfg.get('near')
        SensorClass  = PAA5100 if _near else PMW3901
        self._log.info(Fore.GREEN + 'sensor class: {}'.format(SensorClass))
        self._nofs   = SensorClass(spi_port=0, spi_cs=_slot)
        self._nofs.set_rotation(_rotation)
        self._x_trim = _cfg.get('x_trim') # percentage X trim as a multiplier
        self._y_trim = _cfg.get('y_trim') # percentage Y trim as a multiplier
        self._x      = 0.0
        self._y      = 0.0
        self._tx     = 0.0
        self._ty     = 0.0
        self._spmm   = 6.43 # steps per millimeter, calibrated on a bamboo cutting board
        self._decay_rate     = 18    # decay rate per interval
        self._decay_interval = 0.03  # interval for decay logic in seconds
        self._poll_interval  = 0.2   # interval to poll the sensor
        self._poll_task      = None
        self._decay_task     = None
        self._enabled_decay  = True
        self._running        = False
        self._last_poll_time = dt.now().timestamp()
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_led(self, value):
        '''
        Turn the white LEDs on or off.
        '''
#       self._nofs.set_led(value)
        if value is True:
            time.sleep(0.2)
            self._nofs._write(0x7f, 0x14)
            self._nofs._write(0x6f, 0x1c) # DEC 28
            self._nofs._write(0x7f, 0x00)
        else:
            time.sleep(0.2)
            self._nofs._write(0x7f, 0x14)
            self._nofs._write(0x6f, 0x00) 
            self._nofs._write(0x7f, 0x00) 

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def start(self):
        '''
        Start the asyncio tasks for polling and decay.
        '''
        self._log.info('starting polling and decay tasks…')
        self._running = True
        self._poll_task = asyncio.create_task(self._poll_task_fn())  # Create and store the polling task
        if self._enabled_decay:
            self._decay_task = asyncio.create_task(self._decay_task_fn())  # Create and store the decay task

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def stop(self):
        '''
        Stop the asyncio tasks.
        '''
        self._running = False
        if self._poll_task is not None:
            self._poll_task.cancel()  # Explicitly cancel the polling task
            try:
                await self._poll_task  # Await the task to ensure it finishes gracefully
            except asyncio.CancelledError:
                self._log.info("polling task cancelled successfully.")
        if self._decay_task is not None:
            self._decay_task.cancel()  # Explicitly cancel the decay task
            try:
                await self._decay_task  # Await the decay task to ensure it finishes gracefully
            except asyncio.CancelledError:
                self._log.info("decay task cancelled successfully.")
        self._log.info('polling task stopped.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _poll_task_fn(self):
        ''' Async task to poll the sensor for motion data.  '''
        while self._running:
            x, y = await self._get_and_poll_motion()  # Get motion data
            if x is not None and y is not None:
                self._x = x * self._x_trim
                self._y = y * self._y_trim
                self._tx += self._x
                self._ty += self._y
            else:
                self._x = self._y = None
            await self._handle_decay()
            await asyncio.sleep(self._poll_interval)  # sleep for the next poll interval

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _handle_decay(self):
        ''' Handles decay of the positional values over time. '''
        current_time = dt.now().timestamp()
        if current_time - self._last_poll_time > self._decay_interval:
            # Apply decay to the position
            self._tx = self._apply_decay(self._tx)
            self._ty = self._apply_decay(self._ty)
            
            # Update the last poll time to prevent continuous decay without updates
            self._last_poll_time = current_time
    
    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _decay_task_fn(self):
        ''' Async task to handle the decay logic for the positional data.  '''
        while self._running:
            # handle the decay logic
            current_time = dt.now().timestamp()
            if current_time - self._last_poll_time > self._decay_interval:
                self._tx = self._apply_decay(self._tx)
                self._ty = self._apply_decay(self._ty)
            await asyncio.sleep(self._decay_interval)  # sleep for the decay interval

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _get_and_poll_motion(self):
        """Async method to retrieve sensor motion data and return it."""
        try:
            # Directly get the motion data from the sensor
            x, y = await asyncio.to_thread(self._nofs.get_motion)
            self._log.info(Fore.MAGENTA + 'x: {}, {}'.format(x, y))
            return x, y
        except Exception as e:
#           self._log.error(f"Error getting motion data: {e}")
            return None, None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _apply_decay(self, value):
        '''
        Apply decay to a value.
        '''
        if value > 0:
            return max(0, value - self._decay_rate)
        elif value < 0:
            return min(0, value + self._decay_rate)
        return 0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def relative(self):
        '''
        Return the relative positional change since the previous poll.
        '''
#       self._poll()
        return self._x, self._y

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def absolute(self):
        '''
        Return the absolute (cumulative) positional change since the
        previous poll.
        '''
#       self._poll()
        return int(self._tx), int(self._ty)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def millimeters(self):
        '''
        Return the absolute (cumulative) positional change in millimeters
        since the previous poll.
        '''
        _abs_x, _abs_y = int(self._tx), int(self._ty)
        _dist_x = int(_abs_x / self._spmm)
        _dist_y = int(_abs_y / self._spmm)
        return _dist_x, _dist_y

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def y_variance(self):
        return 0 if self._tx == 0 else int( self._ty / self._tx * 100.0 )

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset(self):
        self._tx = 0.0
        self._ty = 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Closes the sensor, turning off the LED.
        '''
        self.set_led(False)
        Component.close(self) # calls disable

#EOF
