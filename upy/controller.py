#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-16
# modified: 2025-12-30

import sys
import time
from machine import Pin, Timer

from colors import*
from color_store import ColorStore

class Controller:
    '''
    A controller for command strings received from the I2CSlave.
    '''
    def __init__(self):
        self._slave = None
        self._strip = None
        self._ring  = None
        self._store = ColorStore()
        self._led   = Pin('PB2')
        self._timer_enabled   = False
        # blink support
        self._enable_blink    = True
        self._enable_rotate   = False
        self._blink_index     = -1
        self._blink_direction = 1
        self._blink_color     = COLOR_AMBER
        self._ring_colors = [COLOR_BLACK] * 24
        print('ready.')

    def _blink_led(self, t):
        '''
        Flash the LED.
        '''
        self._led.value(1)
        time.sleep_ms(50)
        self._led.value(0)
        time.sleep_ms(50)

    def set_strip(self, strip):
        self._strip = strip

    def set_ring(self, ring):
        self._ring = ring

    def set_slave(self, slave):
        '''
        Assigns the I2C slave and registers this controller's callback.
        Should be called by your main program after instantiating both.
        '''
        self._slave = slave
        self._slave.add_callback(self.on_command)

    def on_command(self, cmd):
        '''
        Callback invoked by I2C slave when a command is received and processed outside IRQ.
        Delegates to process() for handling.
        '''
        return self.process(cmd)

    def tick(self, delta_ms):
        '''
        Can be called from main to update based on a delta in milliseconds.
        '''
        pass

    def step(self, t):
        if self._enable_blink:
            self.blink()
        if self._enable_rotate:
            self.rotate_ring(1)

    def blink(self):
        '''
        A callback from an external Timer that sequentially blinks the LEDs of the strip.
        The first time this is called it sets up a timer to blink the LED.
        '''
        if self._enable_blink:
            if self._strip:
                self._strip.set_color(index=(0 if self._blink_index < 0 else self._blink_index), color=COLOR_BLACK)
            time.sleep_ms(50)
            # update index for next time
            self._blink_index += self._blink_direction
            if self._strip:
                self._strip.set_color(index=self._blink_index, color=self._blink_color)
            # bounce at the ends
            if self._blink_index >= 7:
                self._blink_direction = -1
            elif self._blink_index <= 0:
                self._blink_direction = 1
        if not self._timer_enabled:
            timer0 = Timer()
            timer0.init(freq=0.5, mode=Timer.PERIODIC, callback=self._blink_led)
            self._timer_enabled = True

    def heading_to_pixel(self, heading_deg):
        '''
        Convert heading in degrees to nearest pixel index on 24-pixel ring.
        Pixel 0 = South, 6 = East, 12 = North, 18 = West
        '''
        pixel = round((180 - heading_deg) / 15) % 24
        return pixel

    def get_color(self, name, second_token):
        '''
        Returns the color from either the store or the enum, optionally
        using a second token for whitespace-delimited names.
        '''
        if second_token: # e.g., "dark cyan"
            name = '{} {}'.format(name, second_token)
        saved = self._store.get(name)
        if saved:
            return saved
        else:
            return Color.get(name)

    def rotate_ring(self, shift):
        '''
        Rotates the underlying ring list, then updates the display.
        '''
        if abs(shift) > 24:
            raise ValueError('shift value outside of bounds.')
        self._ring_colors = self._ring_colors[shift:] + self._ring_colors[:shift]
        self.update_ring()

    def update_ring(self):
        for index in range(24):
            color = self._ring_colors[index]
            self._ring.set_color(index, color)

    def set_ring_color(self, index, color):
        self._ring.set_color(index, color)
        self._ring_colors[index] = color

    def process(self, cmd):
        '''
        Processes the callback from the I2C slave, returning 'ACK', 'NACK'
        or 'ERR'. Data requests and use three transactions, the first is
        followed by 'get' and then 'clear', somewhat arbitrary tokens that
        return the previous response and then clear the buffer.
        '''
        try:
            print("cmd: '{}'".format(cmd))
            parts = cmd.lower().split()
            if len(parts) == 0:
                return 'ERR'
            _arg0 = parts[0]
            _arg1 = parts[1] if len(parts) > 1 else None
            _arg2 = parts[2] if len(parts) > 2 else None
            _arg3 = parts[3] if len(parts) > 3 else None
            _arg4 = parts[4] if len(parts) > 4 else None

            if _arg0 == "strip":
                if self._strip:
                    if _arg1 == 'all':
                        # e.g., strip all blue
                        if _arg2 == 'off':
                            self.clear_strip()
                            return 'ACK'
                        else:
                            color = self.get_color(_arg2, _arg3)
                            for idx in range(8):
                                self._strip.set_color(idx, color)
                            return 'ACK'
                    else:
                        index = int(_arg1)
                        color = self.get_color(_arg2, _arg3)
                        if color:
                            # e.g.:  strip 3 blue
                            self._strip.set_color(index,color)
                            return 'ACK'
                    print("ERROR: could not process input: '{}'".format(cmd))
                else:
                    print('ERROR: no LED strip available.')
                return 'ERR'

            elif _arg0 == "ring":
                if self._ring:
                    if _arg1 == 'clear':
                        self._store.clear()
                        self.clear_ring()
                        return 'ACK'
                    elif _arg1 == 'all':
                        # e.g., ring all off
                        if _arg2 == 'off':
                            self.clear_ring()
                            self._store.clear()
                            return 'ACK'
                        else:
                            color = self.get_color(_arg2, _arg3)
                            for idx in range(24):
                                self.set_ring_color(index, color)
                            return 'ACK'
                    else:
                        index = int(_arg1)
                        color = self.get_color(_arg2, _arg3)
                        if color:
                            # e.g.:  ring 20 yellow
                            self.set_ring_color(index, color)
                            return 'ACK'
                    print("ERROR: could not process input: '{}'".format(cmd))
                else:
                    print('ERROR: no LED ring available.')
                return 'ERR'

            elif _arg0 == "rotate":
                if _arg1:
                    if _arg1 == 'on':
                        self._enable_rotate = True
                    elif _arg1 == 'off':
                        self._enable_rotate = False
                    else:
                        shift = int(_arg1)
                        self.rotate_ring(shift)
                    return 'ACK'
                else:
                    return 'ERR'

            elif _arg0 == "blink":
                if _arg1 == 'on':
                    print('step on.')
                    self._enable_blink = True
                    return 'ACK'
                elif _arg1 == 'off':
                    print('step off.')
                    self._enable_blink = False
                    self.clear_strip()
                    self._blink_index = -1
                    self._blink_direction = 1
                    return 'ACK'
                else:
                    print("ERROR: unrecognised argument: '{}'".format(_arg1))
                    return 'ERR'

            elif _arg0 == "save":
                # saves rgb color by name
                name  = _arg1
                red   = int(_arg2)
                green = int(_arg3)
                blue  = int(_arg4)
                color = (red, green, blue)
                self._store.put(name, color)
                if self._strip:
                    for index in range(8):
                        self._strip.set_color(index, color)
                    time.sleep(1)
                    self.clear_strip()
                return 'ACK'

            elif _arg0 == "rgb":
                # e.g., (ring only) rgb 3 130 40 242
                index = int(_arg1)
                red   = int(_arg2)
                green = int(_arg3)
                blue  = int(_arg4)
                self._ring.set_color(index, (red, green, blue))
                return 'ACK'

            elif _arg0 == "heading":
                # e.g., heading 87 blue | heading 87 off
                degrees = int(_arg1)
                index = self.heading_to_pixel(degrees)
                color = self.get_color(_arg2, _arg3)
                self._ring.set_color(index, color)
                return 'ACK'
            elif _arg0 == "get":
                return 'ACK' # called on 2nd request for data
            elif _arg0 == "clear":
                return 'ACK' # called on 3rd request for data
            else:
                print("unrecognised command: '{}'{}{}{}".format(
                        cmd,
                        "; arg0: '{}'".format(_arg0) if _arg0 else '',
                        "; arg1: '{}'".format(_arg1) if _arg1 else '',
                        "; arg2: '{}'".format(_arg2) if _arg2 else ''))
                return 'NACK'
            return 'ACK'
        except Exception as e:
            print("ERROR: {} raised by controller: {}".format(type(e), e))
            sys.print_exception()
            return 'ERR'

    def clear_ring(self):
        for index in range(24):
            self._ring.set_color(index, COLOR_BLACK)

    def clear_strip(self):
        if self._strip:
            for index in range(8):
                self._strip.set_color(index, COLOR_BLACK)

#EOF
