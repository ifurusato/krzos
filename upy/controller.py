#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License.
#
# author:   Murray Altheim
# created:  2025-11-16
# modified: 2025-11-23

import sys
import time
from machine import RTC

from colors import*
from color_store import ColorStore
from odometer import Odometer

class Controller:
    '''
    A controller for command strings received from the I2CSlave.

    This is a generic controller and simply prints the command
    string to the console. It can be either modified directly
    or subclassed to provide specific application handling.
    '''
    def __init__(self):
        self._chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
        self._slave = None
        self._strip = None
        self._ring  = None
        self._store = ColorStore()
        # blink support
        self._enable_blink    = True
        self._enable_rotate   = False
        self._blink_index     = -1
        self._blink_direction = 1
        self._blink_color     = COLOR_AMBER
        self._ring_colors = [COLOR_BLACK] * 24
        self._last_update_ts = self._get_time()
        # instantiate the odometer
        self._odometer = Odometer()
        # heartbeat feature
        self._heartbeat_enabled     = True
        self._heartbeat_on_time_ms  = 50
        self._heartbeat_off_time_ms = 2950
        self._heartbeat_timer = 0
        self._heartbeat_state = False
        self._position = '0 0'
        self._velocity = '0 0'
        print('ready.')

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

    def _update_odometry(self):
        '''
        Called to update the position and velocity variables from the Odometer.
        This also updates the stored timestamp, regardless of whether the NOFS
        has altered the values.
        '''
        x, y = self._odometer.position
        vx, vy = self._odometer.velocity
        self._position = '{} {}'.format(int(x), int(y))
        self._velocity = '{} {}'.format(int(vx), int(vy))
        self._last_update_ts = self._get_time()

    def tick(self, delta_ms):
        '''
        Can be called from main to update based on a delta in milliseconds.
        '''
        if self._heartbeat_enabled:
            self._heartbeat(delta_ms)

    def _heartbeat(self, delta_ms):
        '''
        Regulates the ticks so that the Odometer is not called too frequently.
        It updates about once per second.
        '''
        self._heartbeat_timer += delta_ms
        if self._heartbeat_state:
            if self._heartbeat_timer >= self._heartbeat_on_time_ms:
                self._update_odometry()
                self._heartbeat_state = False
                self._heartbeat_timer = 0
        else:
            if self._heartbeat_timer >= self._heartbeat_off_time_ms:
                self._odometer.update()
                self._heartbeat_state = True
                self._heartbeat_timer = 0

    def step(self, t):
        if self._enable_blink:
            self.blink()
        if self._enable_rotate:
            self.rotate_ring(1)

    def blink(self):
        '''
        A callback from an external Timer that sequentially blinks the LEDs of the strip.
        '''
        if self._enable_blink:
            self._strip.set_color(index=(0 if self._blink_index < 0 else self._blink_index), color=COLOR_BLACK)
            time.sleep_ms(50)
            # update index for next time
            self._blink_index += self._blink_direction
            self._strip.set_color(index=self._blink_index, color=self._blink_color)
            # bounce at the ends
            if self._blink_index >= 7:
                self._blink_direction = -1
            elif self._blink_index <= 0:
                self._blink_direction = 1

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

    def _parse_timestamp(self, ts):
        year    = int(ts[0:4])
        month   = int(ts[4:6])
        day     = int(ts[6:8])
        hour    = int(ts[9:11])
        minute  = int(ts[11:13])
        second  = int(ts[13:15])
        weekday = 0
        subsecs = 0
        return (year, month, day, weekday, hour, minute, second, subsecs)

    def _rtc_to_iso(self, dt):
        return "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}".format(dt[0], dt[1], dt[2], dt[4], dt[5], dt[6])

    def _get_time(self):
        '''
        Return the current timestamp as an integer representing the
        number of seconds since the Unix epoch.
        '''
        return time.time()

    def _set_time(self, timestamp):
        try:
            print('BEFORE: {}'.format(self._rtc_to_iso(RTC().datetime())))
            RTC().datetime(self._parse_timestamp(timestamp))
            print('AFTER:  {}'.format(self._rtc_to_iso(RTC().datetime())))
            return 'ACK' 
        except Exception as e:
            print("ERROR: {} raised by tinyfx controller: {}".format(type(e), e))
            return 'ERR'

    def process(self, cmd):
        '''
        Processes the callback from the I2C slave, returning 'ACK', 'NACK'
        or 'ERR'. Data requests are for 'pir' and use three transactions, 
        the first is followed by 'get' and then 'clear', somewhat arbitrary
        tokens that return the previous response and then clear the buffer.

        Commands:
            odo pos
                vel
                reset
                led on | off
                rf get 
                rf set <value> | None
            time get | set <timestamp>
            strip off | <color>
                  <n> <color>
            ring off | <color>
                  <n> <color>
            rotate <n> | on | off
            blink on | off
            save <name> <red> <green> <blue>
            rgb <n> <red> <green> <blue>
            heading <degrees> <color>
            get
            clear
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

            # odometer ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            if _arg0 == "odo":
                if _arg1 == "pos":
                    msg = '{} {}'.format(self._last_update_ts, self._position)
                    print("message: '{}'".format(msg))
                    return msg
                elif _arg1 == "vel":
                    msg = '{} {}'.format(self._last_update_ts, self._velocity)
                    print("message: '{}'".format(msg))
                    return msg
                elif _arg1 == "reset":
                    self._odometer.reset()
                    return 'ACK'
                elif _arg1 == "led":
                    if _arg2 == "on":
                        self._odometer.set_sensor_led(True)
                        return 'ACK'
                    elif _arg2 == "off":
                        self._odometer.set_sensor_led(False)
                        return 'ACK'
                elif _arg1 == "rf":
                    if _arg2 == "get":
                        return self._odometer.get_resolution_factor()
                    elif _arg2 == "set":
                        if _arg3 is None:
                            self._odometer.set_resolution_factor(None)
                            return 'ACK'
                        else:
                            try:
                                resolution_factor = float(_arg3)
                                self._odometer.set_resolution_factor(resolution_factor)
                                return 'ACK'
                            except Exception as e:
                                self._log.error('{} raised setting resolution factor: {}'.format(type(e), e))
                                return 'ERR'
                return 'ERR'

            # RTC ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            elif _arg0 == "time":
                print('time: {}, {}'.format(_arg1, _arg2))
                if _arg1 == 'set':
                    return self._set_time(_arg2)
                elif _arg1 == 'get':
                    return "2025" # TEMP
                return 'ERR'

            # LEDs ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            elif _arg0 == "strip":
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
#               self._colors[name] = color
                self._store.put(name, color)
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

            # get/set ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
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
        for index in range(8):
            self._strip.set_color(index, COLOR_BLACK)

    def sensor_led_off(self):
        if elf._odometer:
            self._odometer.sensor_led_off()
    
#EOF
