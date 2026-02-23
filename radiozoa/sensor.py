#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-01-27
# modified: 2026-02-18

import micropython
import asyncio
import time
from machine import Timer
from colorama import Fore, Style

from logger import Logger, Level
from device import Device
from cardinal import Cardinal, NORTH
from message_util import pack_message
from exceptions import IllegalStateError

class Sensor:
    OUT_OF_RANGE = 9999
    SENSOR_COUNT = 8
    
    def __init__(self, controller=None, level=Level.INFO):
        if controller is None:
            raise ValueError('no controller provided.')
        self._log = Logger('sensor', level=level)
        self._controller = controller
        self._radiozoa = self._controller.radiozoa
        self._ring = self._controller.ring
        self._min_distance_mm = 50 
        self._max_short_range_distance_mm = 1000
        self._max_long_range_distance_mm  = 4000
        self._enabled = False
        self._poll_delay_ms = 50 # 50 = 20Hz
        self._return_max_range = True # return maximum range rather than out of range
        self._distances = (Sensor.OUT_OF_RANGE,) * Sensor.SENSOR_COUNT
#       self._distances_fmt = " ".join(str(Sensor.OUT_OF_RANGE) for _ in range(Sensor.SENSOR_COUNT)) # generator expression
        self._distances_fmt = " ".join([str(Sensor.OUT_OF_RANGE)] * Sensor.SENSOR_COUNT) # list multiplication
        self._distances_packed = pack_message(self._distances_fmt)
        self._device_by_index  = {d.index: d for d in Device._registry}
        self._task = None

    @property
    def enabled(self):
        return self._enabled

    @property
    def distances(self):
        if not self._enabled:
            raise IllegalStateError('sensor not enabled')
        return self._distances

    @property
    def distances_fmt(self):
        if not self._enabled:
            raise IllegalStateError('sensor not enabled')
        return self._distances_fmt

    @property
    def distances_packed(self):
        if not self._enabled:
            raise IllegalStateError('sensor not enabled')
        return self._distances_packed

    def enable(self):
        if not self._enabled:
            self._enabled = True
            self._task = asyncio.create_task(self._poll_loop())

    @property
    def poll_rate_hz(self):
        return self._poll_delay_ms * 1000

    def set_poll_rate_hz(self, rate_hz=20):
        '''
        Set the sensor polling rate in Hz. The valid range is 0.5Hz to 50Hz, the
        default is 20Hz. Calling this with no argument will reset to the default.
        '''
        if not (0.5 <= rate_hz <= 50):
            raise ValueError("rate_hz must be between 0.5 and 50 Hz")
        self._poll_delay_ms = int(1000 / rate_hz)
        self._log.info('sensor poll rate set to {}Hz (delay: {}ms).'.format(rate_hz, self._poll_delay_ms))

    def disable(self):
        if self._enabled:
            self._enabled = False
            if self._task:
                self._task.cancel()

    async def _poll_loop(self):
        self._log.info('starting poll loop…')
        while self._enabled:
            try:
                if self._radiozoa:
                    self._distances = tuple(
                        v if v is not None else Sensor.OUT_OF_RANGE
                        for v in self._radiozoa.get_distances()
                    )
                    self._distances_fmt = " ".join("{:04d}".format(v) for v in self._distances)
                    self._distances_packed = pack_message(self._distances_fmt)
                    for index, dist in enumerate(self._distances):
                        _cardinal = Cardinal.from_id(index)
                        _device = self._device_by_index[index]
                        if _device.impl == "VL53L0X":
                            _color = self._color_for_distance(_cardinal, dist, self._max_short_range_distance_mm)
                        elif _device.impl == "VL53L1X":
                            _color = self._color_for_distance(_cardinal, dist, self._max_long_range_distance_mm)
                        else:
                            raise ValueError("unrecognised sensor type: {}".format(_device.impl))
                        if _color is not None:
                            self._ring.set_color(_cardinal.pixel - 1, _color)
                else:
                    self._log.warning("no radiozoa: disabling…")
                    self.disable()
            except Exception as e:
                self._log.error("{} raised in poll_loop: {}".format(type(e), e))
            
            await asyncio.sleep_ms(self._poll_delay_ms) 
        self._log.info(Fore.MAGENTA + 'completed poll loop.')

    def _color_for_distance(self, cardinal, distance, max_distance_mm):
        if distance is None or distance > max_distance_mm:
            if self._return_max_range:
                distance = max_distance_mm
            else:
                return (0, 0, 0)
        if distance <= self._min_distance_mm:
            return (255, 0, 0)
        ratio = distance / max_distance_mm
        hue = ratio * 300
        return self._hsv_to_rgb(hue, 1.0, 1.0)

    def _hsv_to_rgb(self, h, s, v):
        if s == 0:
            val = int(v * 255)
            return val, val, val
        h = h % 360
        h_sector = h // 60
        f = (h / 60) - h_sector
        p = v * (1 - s)
        q = v * (1 - s * f)
        t = v * (1 - s * (1 - f))
        if h_sector == 0:
            r, g, b = v, t, p
        elif h_sector == 1:
            r, g, b = q, v, p
        elif h_sector == 2:
            r, g, b = p, v, t
        elif h_sector == 3:
            r, g, b = p, q, v
        elif h_sector == 4:
            r, g, b = t, p, v
        else:
            r, g, b = v, p, q
        return int(r * 255), int(g * 255), int(b * 255)

#EOF
