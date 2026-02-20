#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-01-27
# modified: 2026-02-04

import asyncio
from machine import Timer
import time
import micropython

from cardinal import Cardinal, NORTH
from message_util import pack_message

class Sensor:
    OUT_OF_RANGE = 9999
    
    def __init__(self, controller=None):
        if controller is None:
            raise ValueError('no controller provided.')
        self._controller = controller
        self._radiozoa = self._controller.radiozoa
        self._ring = self._controller.ring
        self._min_distance_mm = 50 
        self._max_distance_mm = 1000
        self._enabled = False
        self._distances = (Sensor.OUT_OF_RANGE,) * 8
        self._distances_fmt = "1111 1111 1111 1111 1111 1111 1111 1111"
        self._distances_packed = pack_message(self._distances_fmt)
        self._task = None

    @property
    def enabled(self):
        return self._enabled

    @property
    def distances(self):
        return self._distances

    @property
    def distances_fmt(self):
        return self._distances_fmt

    @property
    def distances_packed(self):
        return self._distances_packed

    def enable(self):
        if not self._enabled:
            self._enabled = True
            self._task = asyncio.create_task(self._poll_loop())

    def disable(self):
        if self._enabled:
            self._enabled = False
            if self._task:
                self._task.cancel()

    async def _poll_loop(self):
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
                        cardinal = Cardinal.from_id(index)
                        color = self._color_for_distance(cardinal, dist)
                        self._ring.set_color(cardinal.pixel - 1, color)
                else:
                    print("no radiozoa: disabling…")
                    self.disable()
            except Exception as e:
                print("error in poll_loop: {}".format(e))
            
            await asyncio.sleep_ms(50)  # 20Hz polling

    def _color_for_distance(self, cardinal, distance):
        if distance is None or distance > self._max_distance_mm:
            return (0, 0, 0)
        if distance <= self._min_distance_mm:
            return (255, 0, 0)
        ratio = distance / self._max_distance_mm
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
