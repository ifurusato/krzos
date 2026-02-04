#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-02-04

import sys
import time
import math, random
#from machine import LED
from machine import Timer
from machine import RTC
from colorama import Fore, Style

import neopixel
import tinys3

from colors import*
from pixel import Pixel
from message_util import pack_message
from pixel_state import PixelState
from configure import Configure
from radiozoa_sensor import RadiozoaSensor

# pre-packed constant responses
_PACKED_ACK  = pack_message('ACK')
_PACKED_NACK = pack_message('NACK')
_PACKED_ERR  = pack_message('ERR')
_PACKED_PING = pack_message('PING')

class Controller:
    STRIP_PIN = 'B12'
    RING_PIN  = 44 #'B14'
    '''
    A controller for command strings received from the I2CSlave.
    '''
    def __init__(self):
        self._slave = None
        self._sensor = None
        self._radiozoa = None
        # odometry heartbeat feature
        self._heartbeat_enabled     = False
        self._heartbeat_on_time_ms  = 50
        self._heartbeat_off_time_ms = 2950
        self._heartbeat_timer = 0
        self._heartbeat_state = False
        # rotation
        self._ring_offset     = 0
        self._rotate_direction = 1 # 1 or -1
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
        # neopixel (née LED) support
        self._pixel = Pixel(pin=tinys3.RGB_DATA, pixel_count=1)
        tinys3.set_pixel_power(1)
        # instantiate ring
        self._ring  = Pixel(pin=Controller.RING_PIN, pixel_count=24, brightness=0.1)
        self.reset_ring()
        self._last_update_ts  = self._get_time()
        self._timer0 = Timer(0)
        self._timer1_hz = 24
        self._timer1 = Timer(1)
        self._timer1.init(freq=self._timer1_hz, mode=Timer.PERIODIC, callback=self._action)
        print(Fore.CYAN + 'controller ready.' + Style.RESET_ALL)

    @property
    def ring(self):
        return self._ring

    @property
    def radiozoa(self):
        return self._radiozoa

    def _radiozoa_init(self):
        '''
        Initialise the Radiozoa sensor.
        '''
        try:
            config = Configure()
            if config.configure():
                from sensor import Sensor

                self._radiozoa = RadiozoaSensor()
                self._sensor = Sensor(controller=self)
            else:
                print('fail.')
        except Exception as e:
            print("ERROR: {} raised by radiozoa_init: {}".format(type(e), e))
            sys.print_exception()

    def _radiozoa_start(self):
        if not self._radiozoa:
            self._radiozoa_init()
            time.sleep_ms(250)
        if self._radiozoa:
            self._radiozoa.start_ranging()
            self._sensor.enable()
        else:
            print('failed to start: radiozoa not configured.')

    def _radiozoa_stop(self):
        if self._radiozoa:
            self._sensor.disable()
            self._radiozoa.stop_ranging()
        else:
            print('failed to stop: radiozoa not configured.')

    def step(self):
        self._pixel.set_color(0, COLOR_CYAN)
        self._timer0.deinit()
        self._timer0.init(period=20, mode=Timer.PERIODIC, callback=self._led_off)

    def _led_off(self, timer=None):
        self._pixel.set_color(0, COLOR_BLACK)

    def set_slave(self, slave):
        self._slave = slave
        self._slave.add_callback(self.on_command)

    def on_command(self, cmd):
        return self.process(cmd)

    def tick(self, delta_ms):
        if self._heartbeat_enabled:
            self._heartbeat(delta_ms)

    def _heartbeat(self, delta_ms):
        self._heartbeat_timer += delta_ms
        if self._heartbeat_state:
            if self._heartbeat_timer >= self._heartbeat_on_time_ms:
                self._heartbeat_state = False
                self._heartbeat_timer = 0
        else:
            if self._heartbeat_timer >= self._heartbeat_off_time_ms:
                self._heartbeat_state = True
                self.step()
                self._heartbeat_timer = 0

    def heading_to_pixel(self, heading_deg):
        pixel = round((180 + heading_deg) / 15) % 24
        return pixel

    def get_color(self, name, second_token):
        if second_token: # e.g., "dark cyan"
            name = '{} {}'.format(name, second_token)
        return Color.get(name)

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
        shift *= self._rotate_direction
        self._ring_offset = (self._ring_offset + shift) % 24
        self.update_ring()

    def update_ring(self):
        for index in range(24):
            rotated_index = (index - self._ring_offset) % 24
            self._ring.set_color(index, self._ring_model[rotated_index].color)

    def set_ring_color(self, index, color):
#       print("set ring {} color to: {}".format(index, color))
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
#           print('time set.')
            return _PACKED_ACK
        except Exception as e:
            print("ERROR: {} raised by tinyfx controller: {}".format(type(e), e))
            return _PACKED_ERR

    def _restart_timer(self, freq=None):
        '''
        Restart timer 1 if either rotate or theme is enabled.
        '''
        if freq:
            self._timer1_hz = freq
        self._timer1.deinit()
        if self._enable_rotate or self._enable_theme:
            self._timer1.init(freq=self._timer1_hz, mode=Timer.PERIODIC, callback=self._action)

    def process(self, cmd):
        '''
        Processes the callback from the I2C slave, returning 'ACK', 'NACK' or 'ERR'.

        Commands:
            radiozoa init | start | stop
            time get | set <timestamp>
            strip off | all <color> | <n> <color>
            ring off | <color> | <n> <color>
            rotate <n> | on | off | hz <n> | fwd/cw | rev/ccw
            theme on | off | hz <n> | pixels <n> | steps <n> | <theme-name> <n>
            heartbeat on | off
            blink on | off
            rgb <n> <red> <green> <blue>
            heading <degrees> <color>
            get
            clear
            close
        '''
        _pixel_off = True
        _exit_color = COLOR_BLACK
        self._pixel.set_color(0, COLOR_CYAN)
        try:
#           print("cmd: '{}'".format(cmd))
            parts = cmd.lower().split()
            if len(parts) == 0:
                _exit_color = COLOR_RED
                return _PACKED_ERR
            _arg0 = parts[0]
            _arg1 = parts[1] if len(parts) > 1 else None
            _arg2 = parts[2] if len(parts) > 2 else None
            _arg3 = parts[3] if len(parts) > 3 else None
            _arg4 = parts[4] if len(parts) > 4 else None
#           print("cmd: '{}'; arg0: '{}'; arg1: '{}'; arg2: '{}'; arg3: '{}'; arg4: '{}'".format(cmd, _arg0, _arg1, _arg2, _arg3, _arg4))

            if _arg0 == "time":
#               print('time: {}, {}'.format(_arg1, _arg2))
                if _arg1 == 'set':
                    _exit_color = COLOR_DARK_GREEN
                    return self._set_time(_arg2)
                elif _arg1 == 'get':
                    _exit_color = COLOR_DARK_GREEN
                    return pack_message(self._rtc_to_iso(RTC().datetime()))
                _exit_color = COLOR_RED
                return _PACKED_ERR

            elif _arg0 == "radiozoa":
                if _arg1 == 'init':
                    self._radiozoa_init()
                    _exit_color = COLOR_DARK_GREEN
                    return _PACKED_ACK
                elif _arg1 == 'start':
                    self._radiozoa_start()
                    _exit_color = COLOR_DARK_GREEN
                    return _PACKED_ACK
                elif _arg1 == 'stop':
                    self._radiozoa_stop()
                    self.reset_ring()
                    _exit_color = COLOR_DARK_GREEN
                    return _PACKED_ACK
                _exit_color = COLOR_RED
                return _PACKED_ERR

            elif _arg0 == "distances":
                if self._sensor:
                    _exit_color = COLOR_DARK_GREEN
                    return self._sensor.distances_packed
                else:
                    print('no sensor.')
                _exit_color = COLOR_RED
                return _PACKED_ERR

            elif _arg0 == "ring":
                if self._ring:
                    try:
                        _rotating = self._enable_rotate
                        if _arg1 == 'clear':
                            self.reset_ring()
                            _exit_color = COLOR_BLACK
                            return _PACKED_ACK
                        elif _arg1 == 'all':
                            if _arg2 == 'off' or _arg2 == 'clear':
                                self.reset_ring()
                                _exit_color = COLOR_DARK_GREEN
                                return _PACKED_ACK
                            else:
                                color = self.get_color(_arg2, _arg3)
                                if not color:
                                    print("ERROR: could not find color: arg2: '{}'; arg3: '{}'".format(_arg2, _arg3))
                                    _exit_color = COLOR_RED
                                    return _PACKED_ERR
                                for idx in range(24):
                                    self.set_ring_color(idx, color)
                                _exit_color = COLOR_DARK_GREEN
                                return _PACKED_ACK
                        else:
                            index = int(_arg1) - 1
                            if 0 <= index <= 23:
                                color = self.get_color(_arg2, _arg3)
                                if color:
                                    self.set_ring_color(index, color)
                                    _exit_color = COLOR_DARK_GREEN
                                    return _PACKED_ACK
                            else:
                                print("index value {} out of bounds (1-24).".format(index))
                                _exit_color = COLOR_RED
                                return _PACKED_ERR
                        print("ERROR: could not process input: '{}'".format(cmd))
                    finally:
                        self._enable_rotate = _rotating
                else:
                    print('ERROR: no LED ring available.')
                _exit_color = COLOR_RED
                return _PACKED_ERR

            elif _arg0 == "pixel":
                _pixel_off = False
                if self._pixel:
                    if _arg1 == 'off' or _arg1 == 'clear':
                        color = COLOR_BLACK
                    else:
                        color = self.get_color(_arg1, _arg2)
                    if not color:
                        print("ERROR: could not find color: arg1: '{}'; arg2: '{}'".format(_arg1, _arg2))
                        _exit_color = COLOR_RED
                        return _PACKED_ERR
                    self._pixel.set_color(0, color)
                    return _PACKED_ACK
                else:
                    print('ERROR: no pixel available.')
                _exit_color = COLOR_RED
                return _PACKED_ERR

            elif _arg0 == "rotate":
                if _arg1:
                    if _arg1 == 'on':
                        self._enable_rotate = True
                        self._restart_timer()
                        _exit_color = COLOR_DARK_GREEN
                        return _PACKED_ACK
                    elif _arg1 == 'off':
                        self._enable_rotate = False
                        self._restart_timer()
                        _exit_color = COLOR_DARK_GREEN
                        return _PACKED_ACK
                    elif _arg1 == 'fwd' or _arg1 == 'cw':
                        self._rotate_direction = 1
                        _exit_color = COLOR_DARK_GREEN
                        return _PACKED_ACK
                    elif _arg1 == 'rev' or _arg1 == 'ccw':
                        self._rotate_direction = -1
                        _exit_color = COLOR_DARK_GREEN
                        return _PACKED_ACK
                    elif _arg1 == 'hz':
                        hz = int(_arg2)
                        if hz > 0:
                            self._restart_timer(hz)
                            _exit_color = COLOR_DARK_GREEN
                            return _PACKED_ACK
                        else:
                            _exit_color = COLOR_RED
                            return _PACKED_ERR
                    else:
                        shift = int(_arg1)
                        self.rotate_ring(shift)
                    _exit_color = COLOR_DARK_GREEN
                    return _PACKED_ACK
                else:
                    _exit_color = COLOR_RED
                    return _PACKED_ERR

            elif _arg0 == "theme":
                if _arg1:
                    if _arg1 == 'on':
                        self._init_theme()
                        self._enable_theme = True
                        _exit_color = COLOR_DARK_GREEN
                        return _PACKED_ACK
                    elif _arg1 == 'off':
                        self._enable_theme = False
                        self._restart_timer()
                        _exit_color = COLOR_DARK_GREEN
                        return _PACKED_ACK
                    elif _arg1 == 'hz':
                        hz = int(_arg2)
                        if hz > 0:
                            self._restart_timer(hz)
                            _exit_color = COLOR_DARK_GREEN
                            return _PACKED_ACK
                        _exit_color = COLOR_RED
                        return _PACKED_ERR
                    elif _arg1 == 'pixels':
                        _themed = self._enable_theme
                        try:
                            self._enable_theme = False
                            target = int(_arg2)
                            if 1 <= target <= 24:
                                self._theme_target_pixels = target
                                self._init_theme(reset=True)
                                _exit_color = COLOR_DARK_GREEN
                                return _PACKED_ACK
                            _exit_color = COLOR_RED
                            return _PACKED_ERR
                        finally:
                            self._enable_theme = _themed

                    elif _arg1 in self._palettes:
                        _themed = self._enable_theme
                        _rotating = self._enable_rotate
                        self._enable_rotate = False
                        self._enable_theme = False
                        self._ring_offset = 0
                        try:
                            target = int(_arg2)
                            self._theme_target_pixels = target
                            if 1 <= target <= 24:
                                self.populate(target, _arg1)
                                _exit_color = COLOR_DARK_GREEN
                                return _PACKED_ACK
                            else:
                                _exit_color = COLOR_RED
                                return _PACKED_ERR
                        except Exception as e:
                            print('{} raised with palette name: {}'.format(type(e), e))
                            _exit_color = COLOR_RED
                            return _PACKED_ERR
                        finally:
                            self._enable_theme = _themed
                            self._enable_rotate = _rotating
                        _exit_color = COLOR_DARK_GREEN
                        return _PACKED_ACK

                    elif _arg1 == 'steps':
                        steps = int(_arg2)
                        if steps > 0:
                            self._pulse_steps = steps
                            _exit_color = COLOR_DARK_GREEN
                            return _PACKED_ACK
                        _exit_color = COLOR_RED
                        return _PACKED_ERR
                    else:
                        print("ERROR: could not process input:  '{}'".format(cmd))
                        _exit_color = COLOR_RED
                        return _PACKED_ERR
                    _exit_color = COLOR_DARK_GREEN
                    return _PACKED_ACK
                else:
                    _exit_color = COLOR_RED
                    return _PACKED_ERR

            elif _arg0 == "heartbeat":
                if _arg1 == 'on':
                    self._heartbeat_enabled = True
                    _exit_color = COLOR_DARK_GREEN
                    return _PACKED_ACK
                elif _arg1 == 'off':
                    self._heartbeat_enabled = False
                    _exit_color = COLOR_DARK_GREEN
                    return _PACKED_ACK
                else:
                    print("ERROR: unrecognised argument: '{}'".format(_arg1))
                    _exit_color = COLOR_RED
                    return _PACKED_ERR

            elif _arg0 == "rgb":
                # e.g., rgb 3 130 40 242
                index = int(_arg1)
                red   = int(_arg2)
                green = int(_arg3)
                blue  = int(_arg4)
                self._ring.set_color(index, (red, green, blue))
                _exit_color = COLOR_DARK_GREEN
                return _PACKED_ACK

            elif _arg0 == "heading":
                # e.g., heading 87 blue | heading 87 off
                degrees = int(_arg1)
                index = self.heading_to_pixel(degrees)
                color = self.get_color(_arg2, _arg3)
                self._ring.set_color(index, color)
                _exit_color = COLOR_DARK_GREEN
                return _PACKED_ACK

            elif _arg0 == "ping":
                _exit_color = COLOR_DARK_GREEN
                return _PACKED_PING

            # get/set (data request)
            elif _arg0 == "data":
                # send test data
                _exit_color = COLOR_FUCHSIA
                return packed_message('0000 1111 2222 3333')

            elif _arg0 == "close":
                self._enable_rotate     = False
                self._heartbeat_enabled = False
                self.reset_ring()
                _exit_color = COLOR_BLACK
                return _PACKED_ACK

            else:
                print("unrecognised command: '{}'{}{}{}".format(
                        cmd,
                        "; arg0: '{}'".format(_arg0) if _arg0 else '',
                        "; arg1: '{}'".format(_arg1) if _arg1 else '',
                        "; arg2: '{}'".format(_arg2) if _arg2 else ''))
                _exit_color = COLOR_ORANGE
                return _PACKED_NACK
            _exit_color = COLOR_DARK_GREEN
            return _PACKED_ACK

        except Exception as e:
            print("ERROR: {} raised by controller: {}".format(type(e), e))
            sys.print_exception()
            _exit_color = COLOR_RED
            return _PACKED_ERR
        finally:
            if _pixel_off:
                self._pixel.set_color(0, _exit_color)

#EOF
