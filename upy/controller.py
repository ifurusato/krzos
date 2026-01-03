#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-01-03

import sys
import time
import math, random
from pyb import LED, Timer
from machine import RTC

from colors import*
from color_store import ColorStore
from odometer import Odometer
from pixel import Pixel

class PixelState:
    def __init__(self, color=COLOR_BLACK, phase=0.0):
        self.base_color = color
        self.color = color.rgb
        self.phase = phase
    
    def is_active(self):
        return self.base_color != COLOR_BLACK
    
    def reset(self):
        self.base_color = COLOR_BLACK
        self.color = self.base_color.rgb
        self.phase = 0.0

class Controller:
    STRIP_PIN = 'B12'
    RING_PIN  = 'B14'
    '''
    A controller for command strings received from the I2CSlave.
    '''
    def __init__(self):
        self._slave = None
        self._store = ColorStore()
        # blink support
        self._enable_blink    = False
        self._blink_index     = -1
        self._blink_direction = 1
        self._blink_color     = COLOR_TANGERINE
        # odometry heartbeat feature
        self._heartbeat_enabled     = True
        self._heartbeat_on_time_ms  = 50
        self._heartbeat_off_time_ms = 2950
        self._heartbeat_timer = 0
        self._heartbeat_state = False
        self._position        = '0 0'
        self._velocity        = '0 0'
        # rotation
        self._ring_offset     = 0
        self._enable_rotate   = False
        self._ring_model = [PixelState() for _ in range(24)]
        # theme
        self._enable_theme    = False
        self._pulse_steps = 40
        self._theme_target_pixels = 12 # default
        self._all  = Color.all_colors()
        self._cool = [ COLOR_BLUE, COLOR_CYAN, COLOR_DARK_BLUE, COLOR_DARK_CYAN,
                       COLOR_CORNFLOWER, COLOR_INDIGO, COLOR_VIOLET, COLOR_DEEP_CYAN,
                       COLOR_PURPLE, COLOR_SKY_BLUE ]
        self._warm = [ COLOR_RED, COLOR_YELLOW, COLOR_DARK_RED, COLOR_DARK_YELLOW,
                       COLOR_ORANGE, COLOR_TANGERINE, COLOR_PINK, COLOR_FUCHSIA, COLOR_AMBER ]
        self._wild = [ COLOR_MAGENTA, COLOR_DARK_MAGENTA, COLOR_CORNFLOWER, COLOR_INDIGO, COLOR_RED,
                       COLOR_VIOLET, COLOR_PINK, COLOR_FUCHSIA, COLOR_PURPLE, COLOR_SKY_BLUE,
                       COLOR_WHITE, COLOR_APPLE, COLOR_EMERALD, COLOR_TANGERINE, COLOR_AMBER ]
        self._grey = [ COLOR_WHITE, COLOR_GREY_0, COLOR_GREY_1, COLOR_GREY_2, COLOR_GREY_3,
                       COLOR_GREY_4, COLOR_GREY_5, COLOR_GREY_6, COLOR_GREY_7 ]
        self._dark = [ COLOR_DARK_RED, COLOR_DARK_GREEN, COLOR_DARK_BLUE, COLOR_DARK_CYAN,
                       COLOR_DARK_MAGENTA, COLOR_DARK_YELLOW, COLOR_PURPLE ]
        self._palettes = {
            'all':  self._all,
            'cool': self._cool,
            'warm': self._warm,
            'wild': self._wild,
            'grey': self._grey,
            'dark': self._dark
        }
        # instantiate ring, strip & odometer
        self._strip = Pixel(pin=Controller.STRIP_PIN, pixel_count=8, brightness=0.1)
        self._ring  = Pixel(pin=Controller.RING_PIN, pixel_count=24, brightness=0.1)
        self.reset_ring()
        self._last_update_ts  = self._get_time()
        self._odometer = Odometer()
        self._timer3 = Timer(3)
        self._timer3.init(freq=24, callback=self._action, hard=False)
        # LED support
        self._led = LED(1)
        self._timer7 = Timer(7)
        print('ready.')

    # led ........................................

    def _led_pulse(self, duration_ms=20):
        self._led.on()
        self._timer7.deinit()
        self._timer7.init(period=duration_ms, callback=self._off)

    def _off(self, timer):
        self._led.off()

    # ............................................

    def set_slave(self, slave):
        self._slave = slave
        self._slave.add_callback(self.on_command)

    def on_command(self, cmd):
        return self.process(cmd)

    def _get_odometry_info(self):
        x, y = self._odometer.position
        vx, vy = self._odometer.velocity
        self._position = '{} {}'.format(int(x), int(y))
        self._velocity = '{} {}'.format(int(vx), int(vy))
        self._last_update_ts = self._get_time()

    def tick(self, delta_ms):
        if self._heartbeat_enabled:
            self._heartbeat(delta_ms)

    def _heartbeat(self, delta_ms):
        self._heartbeat_timer += delta_ms
        if self._heartbeat_state:
            if self._heartbeat_timer >= self._heartbeat_on_time_ms:
                self._get_odometry_info()
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
        else:
            self._led_pulse()

    def heading_to_pixel(self, heading_deg):
        pixel = round((180 + heading_deg) / 15) % 24
        return pixel

    def get_color(self, name, second_token):
        if second_token: # e.g., "dark cyan"
            name = '{} {}'.format(name, second_token)
        saved = self._store.get(name)
        if saved:
            return saved
        else:
            return Color.get(name)

    # ring .......................................

    def _set_rotation_pending(self, t):
        self._rotation_pending = True

    def _action(self, t):
        if self._enable_rotate:
            self.rotate_ring()
        if self._enable_theme:
            self.theme()

    def set_ring(self, ring):
        self._ring = ring
        self.reset_ring()

    def reset_ring(self):
        for pixel in self._ring_model:
            pixel.reset()
        self.update_ring()

    def rotate_ring(self, shift=1):
        if abs(shift) > 24:
            raise ValueError('shift value outside of bounds.')
        self._ring_offset = (self._ring_offset + shift) % 24
        self.update_ring()

    def update_ring(self):
        for index in range(24):
            rotated_index = (index - self._ring_offset) % 24
            self._ring. set_color(index, self._ring_model[rotated_index].color)

    def set_ring_color(self, index, color):
        actual_index = (index + self._ring_offset) % 24
        self._ring_model[actual_index].base_color = color
        self._ring_model[actual_index].color = color.rgb
        self._ring.set_color(index, color.rgb)

    def populate(self, count, palette_name):
        palette = self._palettes.get(palette_name)
        if palette is None:
            print("no such palette: '{}'".format(palette_name))
            return
        for pixel in self._ring_model:
            pixel.reset()
            pixel.phase = random.random()
        selected = []
        available = list(range(24))
        for _ in range(count):
            idx = random.randrange(len(available))
            selected.append(available.pop(idx))
        for i in selected:
            color = random.choice(palette)
            self._ring_model[i]. base_color = color
            self._ring_model[i].color = color.rgb
        self.update_ring()

    # themes .....................................

    def _init_theme(self, reset=False):
        if reset:
            self.reset_ring()
            existing_count = 0
        else:
            existing_count = sum(1 for p in self._ring_model if p.is_active())
        new_pixels_needed = max(0, self._theme_target_pixels - existing_count)
        available_colors = [c for c in Color.all_colors() if c != COLOR_BLACK]
        if new_pixels_needed > 0:
            empty_positions = [i for i in range(24) if not self._ring_model[i].is_active()]
            for _ in range(new_pixels_needed):
                if not empty_positions:
                    break
                idx = random.randrange(len(empty_positions))
                pos = empty_positions.pop(idx)
                color = random.choice(available_colors)
                self._ring_model[pos].base_color = color
                self._ring_model[pos].color = color.rgb
                self._ring_model[pos].phase = random.random()
        self.update_ring()

    def theme(self):
        for index in range(24):
            pixel = self._ring_model[index]
            if not pixel.is_active():
                continue
            pixel.phase = (pixel.phase + 1.0 / self._pulse_steps) % 1.0
            brightness = (math.sin(pixel.phase * 2 * math.pi) + 1) / 2
            r, g, b = pixel.base_color.rgb
            pixel.color = (int(r * brightness), int(g * brightness), int(b * brightness))
        self.update_ring()

    # strip ......................................

    def reset_strip(self):
        for index in range(8):
            self._strip.set_color(index, COLOR_BLACK)

    def blink(self):
        if self._enable_blink:
            self._led.off()
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

    # ............................................

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
        return time.time()

    def _set_time(self, timestamp):
        try:
            print('time before: {}'.format(self._rtc_to_iso(RTC().datetime())))
            RTC().datetime(self._parse_timestamp(timestamp))
            print('time after:  {}'.format(self._rtc_to_iso(RTC().datetime())))
            self._blink_color = COLOR_AMBER
            return 'ACK'
        except Exception as e:
            print("ERROR: {} raised by tinyfx controller: {}".format(type(e), e))
            return 'ERR'

    def process(self, cmd):
        '''
        Processes the callback from the I2C slave, returning 'ACK', 'NACK' or 'ERR'.

        Commands:
            odo pos | vel | reset
                led on | off
                rf get
                rf set <value> | None
            time get | set <timestamp>
            strip off | all <color> | <n> <color>
            ring off | <color> | <n> <color>
            rotate <n> | on | off | hz <n>
            theme on | off | hz <n> | pixels <n> | steps <n> | cool <n> | warm <n>
            heartbeat on | off
            blink on | off
            save <name> <red> <green> <blue>
            rgb <n> <red> <green> <blue>
            heading <degrees> <color>
            get
            clear
            close
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

            # odometer
            if _arg0 == "odo":
                if _arg1 == "pos":
                    return self._position
                elif _arg1 == "vel":
                    return self._velocity
                elif _arg1 == "ts":
                    return str(self._last_update_ts)
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

            elif _arg0 == "time":
#               print('time: {}, {}'.format(_arg1, _arg2))
                if _arg1 == 'set':
                    return self._set_time(_arg2)
                elif _arg1 == 'get':
                    return self._rtc_to_iso(RTC().datetime())
                return 'ERR'

            elif _arg0 == "strip":
                if self._strip:
                    if _arg1 == 'clear':
                        self.reset_strip()
                        return 'ACK'
                    elif _arg1 == 'all':
                        # e.g., strip all blue
                        if _arg2 == 'off':
                            self.reset_strip()
                            return 'ACK'
                        else:
                            color = self.get_color(_arg2, _arg3)
                            for idx in range(8):
                                self._strip.set_color(idx, color)
                            return 'ACK'
                    else:
                        # e.g.:  strip 3 blue
                        index = int(_arg1) - 1
                        if 0 <= index <= 7:
                            color = self.get_color(_arg2, _arg3)
                            if color:
                                self._strip.set_color(index,color)
                                return 'ACK'
                        else:
                            print("index value {} out of bounds (1-8).".format(index))
                            return 'ERR'

                    print("ERROR: could not process input: '{}'".format(cmd))
                else:
                    print('ERROR: no LED strip available.')
                return 'ERR'

            elif _arg0 == "ring":
                if self._ring:
                    try:
                        _rotating = self._enable_rotate
                        if _arg1 == 'clear':
                            self._store.clear()
                            self.reset_ring()
                            return 'ACK'
                        elif _arg1 == 'all':
                            if _arg2 == 'off':
                                self.reset_ring()
                                self._store.clear()
                                return 'ACK'
                            else:
                                color = self.get_color(_arg2, _arg3)
                                for idx in range(24):
                                    self.set_ring_color(index, color)
                                return 'ACK'
                        else:
                            index = int(_arg1) - 1
                            if 0 <= index <= 23:
                                color = self.get_color(_arg2, _arg3)
                                if color:
                                    self.set_ring_color(index, color)
                                    return 'ACK'
                            else:
                                print("index value {} out of bounds (1-24).".format(index))
                                return 'ERR'
                        print("ERROR: could not process input: '{}'".format(cmd))
                    finally:
                        self._enable_rotate = _rotating
                else:
                    print('ERROR: no LED ring available.')
                return 'ERR'

            elif _arg0 == "rotate":
                if _arg1:
                    if _arg1 == 'on':
                        self._enable_rotate = True
                        return 'ACK'
                    elif _arg1 == 'off':
                        self._enable_rotate = False
                        return 'ACK'
                    elif _arg1 == 'hz':
                        hz = int(_arg2)
                        if hz > 0:
                            self._timer3.deinit()
                            self._timer3.init(freq=hz, callback=self._action, hard=False)
                            return 'ACK'
                        else:
                            return 'ERR'
                    else:
                        shift = int(_arg1)
                        self.rotate_ring(shift)
                    return 'ACK'
                else:
                    return 'ERR'

            elif _arg0 == "theme": 
                if _arg1:
                    if _arg1 == 'on':
                        self._init_theme()
                        self._enable_theme = True
                        return 'ACK'
                    elif _arg1 == 'off':
                        self._enable_theme = False
                        return 'ACK'
                    elif _arg1 == 'hz': 
                        hz = int(_arg2)
                        if hz > 0:
                            self._timer3.deinit()
                            self._timer3.init(freq=hz, callback=self._action, hard=False)
                            return 'ACK'
                        return 'ERR'
                    elif _arg1 == 'pixels':
                        _themed = self._enable_theme
                        try:
                            self._enable_theme = False
                            target = int(_arg2)
                            if 1 <= target <= 24:
                                self._theme_target_pixels = target
                                self._init_theme(reset=True)
                                return 'ACK'
                            return 'ERR'
                        finally:
                            self._enable_theme = _themed
                    elif _arg1 in self._palettes:
                        print('a. theme')
                        _themed = self._enable_theme
                        _rotating = self._enable_rotate
                        self._enable_rotate = False
                        self._enable_theme = False
                        self._ring_offset = 0
                        try:
                            target = int(_arg2)
                            print('b. theme; target: {}'.format(target))
                            self._theme_target_pixels = target
                            if 1 <= target <= 24:
                                self.populate(target, _arg1)
                                print('c. ACK theme; target: {}'.format(target))
                                return 'ACK'
                            else:
                                print('d. ERR theme; target: {}'.format(target))
                                return 'ERR'
                        except Exception as e:
                            print('{} raised with palette name: {}'.format(type(e), e))
                            return 'ERR'
                        finally:
                            self._enable_theme = _themed
                            self._enable_rotate = _rotating
                        return 'ACK'

                    elif _arg1 == 'steps':
                        steps = int(_arg2)
                        if steps > 0:
                            self._pulse_steps = steps
                            return 'ACK'
                        return 'ERR'
                    else: 
                        print("ERROR: could not process input:  '{}'".format(cmd))
                        return 'ERR'
                    return 'ACK'
                else:
                    return 'ERR'

            elif _arg0 == "heartbeat":
                if _arg1 == 'on':
                    self._heartbeat_enabled = True
                    return 'ACK'
                elif _arg1 == 'off':
                    self._heartbeat_enabled = False
                    return 'ACK'
                else:
                    print("ERROR: unrecognised argument: '{}'".format(_arg1))
                    return 'ERR'

            elif _arg0 == "blink":
                if _arg1 == 'on':
                    self._enable_blink = True
                    return 'ACK'
                elif _arg1 == 'off':
                    self._enable_blink = False
                    self.reset_strip()
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
                for index in range(8):
                    self._strip.set_color(index, color)
                time.sleep(1)
                self.reset_strip()
                return 'ACK'

            elif _arg0 == "rgb":
                # e.g., rgb 3 130 40 242
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

            elif _arg0 == "ping":
                return 'ACK'

            elif _arg0 == "close":
                self._enable_rotate     = False
                self._heartbeat_enabled = False
                self._enable_blink      = False
                self.reset_ring()
                self.reset_strip()
                return 'ACK'

            # get/set (data request)
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

    def sensor_led_off(self):
        if elf._odometer:
            self._odometer.sensor_led_off()

#EOF
