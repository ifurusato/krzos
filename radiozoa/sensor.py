#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-01-27
# modified: 2026-02-01

from machine import Timer
import time
import micropython

from cardinal import Cardinal, NORTH

#micropython.alloc_emergency_exception_buf(100)

class Sensor:
    OUT_OF_RANGE = 9999
    '''
    Wraps access to the set of the Radiozoa's VL53L0X ToF sensors.
    '''
    def __init__(self, controller=None):
        if controller is None:
            raise ValueError('no controller provided.')
        self._controller = controller
        self._timer = Timer(2)
        self._radiozoa = self._controller.radiozoa
        self._ring = self._controller.ring
#       self._scheduled_task = self._test_task
        self._scheduled_task = self._rainbow_task
        self._min_distance_mm = 50 
        self._max_distance_mm = 1000
        self._running = False
        self._pending = False
        self._lower = (Sensor.OUT_OF_RANGE,) * 4
        self._upper = (Sensor.OUT_OF_RANGE,) * 4
        self._lower_fmt = " ".join("{:04d}".format(v) for v in self._lower)
        self._upper_fmt = " ".join("{:04d}".format(v) for v in self._upper)

    @property
    def lower(self):
#       print('lower: {}'.format(self._lower))
        return self._lower

    @property
    def lower_fmt(self):
        return self._lower_fmt

    @property
    def upper(self):
#       print('upper: {}'.format(self._upper))
        return self._upper

    @property
    def upper_fmt(self):
        return self._upper_fmt

    def enable(self):
        if not self._running:
            self._running = True
            self._timer.init(freq=60, mode=Timer.PERIODIC, callback=self._irq_handler)

    def disable(self):
        if not self._running:
            self._running = False
            self._timer.deinit()

    def _test_task(self, t):
        self._pending = False
        if self._radiozoa:
            self._radiozoa.print_distances()
        else:
            print("no radiozoa: disabling…")
            self.disable()

    def _rainbow_task(self, t):
        self._pending = False
        if self._radiozoa:
#           start = time.ticks_ms()
#           distances = self._radiozoa.get_distances()
            distances = tuple(
                v if v is not None else Sensor.OUT_OF_RANGE
                for v in self._radiozoa.get_distances()
            )
            self._lower, self._upper = distances[:4], distances[4:]
            self._lower_fmt = " ".join("{:04d}".format(v) for v in self._lower)
            self._upper_fmt = " ".join("{:04d}".format(v) for v in self._upper)
#           recompose to 8
#           self._distances = self._lower + self._upper
#           print('type: {}; length: {}'.format(type(distances), len(distances)))
#           elapsed_ms = time.ticks_diff(time.ticks_ms(), start)
#           print('poll: {}ms elapsed.'.format(elapsed_ms))
            for index, dist in enumerate(distances):
                cardinal = Cardinal.from_id(index)
                color = self._color_for_distance(cardinal, dist)
#               if cardinal is NORTH:
#                   print('cardinal: {}; distance: {}mm; pixel: {}; color: {}'.format(cardinal.name, dist, cardinal.pixel, color))
                self._ring.set_color(cardinal.pixel - 1, color)
        else:
            print("no radiozoa: disabling…")
            self.disable()

    def _irq_handler(self, t):
        if self._running and not self._pending:
            self._pending = True
            try:
                micropython.schedule(self._scheduled_task, None)
            except RuntimeError as e:
                print(e)
                pass

    def _color_for_distance(self, cardinal, distance):
        if distance is None or distance > self._max_distance_mm:
            return (0, 0, 0)  # black (off)
        if distance <= self._min_distance_mm:
            return (255, 0, 0)  # red
        # normalize to 0.0 .. 1.0
        ratio = distance / self._max_distance_mm
        # hue sweep: red (0°) → purple (~300°)
        hue = ratio * 300
        return self._hsv_to_rgb(hue, 1.0, 1.0)

    def _hsv_to_rgb(self, h, s, v):
        # h: 0..360
        # s, v: 0..1
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
