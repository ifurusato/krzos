#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License.
#
# author:   Ichiro Furusato
# created:  2026-02-09
# modified: 2026-02-27

import sys
import time
import math, random
from colorama import Fore, Style

from pixel_state import PixelState
from controller import Controller
from logger import Logger, Level
#from stm32controller import STM32Controller
from colors import *
from pixel import Pixel

#class RingController(STM32Controller):
class RingController(Controller):
    '''
    An implementation connected to a 24 pixel NeoPixel ring.
    '''
    def __init__(self, config, pixel=None, strip=None, level=Level.INFO):
        self._log = Logger('ring-ctrl', level=level)
        self._ring = None
        super().__init__(config, pixel, strip)
        # pixel ring
        self._ring_count = config['ring_count']
        if self._ring_count is None:
            raise ValueError('ring count is undefined.')
        elif self._ring_count == 0:
            raise ValueError('ring count is 0.')
        self._status_enabled   = True
        self._status_pixel     = 1
        # rotation
        self._ring_offset      = 0
        self._rotate_direction = 1 # 1 or -1
        self._enable_rotate    = False
        self._rotate_pending   = False # flag for deferred execution
        self._ring_model = [PixelState() for _ in range(self._ring_count)]
        # theme
        self._enable_theme     = False
        self._pulse_steps      = 40
        self._theme_pending    = False # flag for deferred execution
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
        self._forest = [ COLOR_APPLE, COLOR_BANANA, COLOR_BROWN, COLOR_CUCUMBER, COLOR_DARK_CYAN,
                       COLOR_DARK_GREEN, COLOR_EMERALD, COLOR_GREEN, COLOR_MELON, COLOR_MUSTARD,
                       COLOR_PEAR, COLOR_ROBIN ]
        self._palettes = {
            'all':  self._all,
            'cool': self._cool,
            'warm': self._warm,
            'wild': self._wild,
            'grey': self._grey,
            'dark': self._dark,
            'forest': self._forest
        }
        self._ring_brightness = 0.1  # default: 0.33
        self._ring = self._create_ring()
        self.reset_ring()
        self._last_update_ts  = self._get_time()
        self._ring_timer_hz = 24
        self._ring_timer = self._create_ring_timer()
        self._radiozoa_started = False
        self._log.info('ready.')
        # ready

    def _start_services(self):
        '''
        This method is called upon startup, after a preset delay. It can be overridden
        as necessary to start any application-level services.
        '''    
        self._strip.set_color(0, COLOR_SKY_BLUE)
        super()._start_services()
        self._strip.set_color(0, COLOR_BLACK)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def strip(self):
        return self._strip

    def _create_strip(self, config):
        from pixel import Pixel

        _strip_brightness = 0.1  # default: 0.33
        _strip_pin   = config['strip_pin']
        _strip_count = config['strip_count']
        _strip = Pixel(
            pin=_strip_pin,
            pixel_count=_strip_count,
            color_order=config['strip_color_order'],
            brightness=_strip_brightness)
        self._log.info('NeoPixel strip with {} pixels configured on pin {}'.format(_strip_count, _strip_pin))
        _strip.set_color(0, COLOR_CYAN)
#       time.sleep_ms(100)
#       _strip.set_color(0, COLOR_BLACK)
        return _strip

    def _led_off(self, timer=None):
        super()._led_off(timer)
        if self._strip and self._status_enabled:
            self._strip.set_color(self._status_pixel, COLOR_BLACK)

    @property
    def ring(self):
        return self._ring

    def _create_ring(self):
        from pixel import Pixel

        _ring_pin   = self._config['ring_pin']
        _ring = Pixel(
            pin=_ring_pin,
            pixel_count=self._ring_count,
            color_order=self._config['ring_color_order'],
            brightness=self._ring_brightness
        )
        self._log.info('NeoPixel ring with {} pixels configured on pin {}'.format(self._ring_count, _ring_pin))
        _ring.set_color(0, COLOR_CYAN)
        time.sleep_ms(100)
        _ring.set_color(0, COLOR_BLACK)
        return _ring

    def _create_ring_timer(self):
        family = self._config['family']
        if "STM32" in family:
            from pyb import Timer

            _ring_timer = Timer(1)
            _ring_timer.init(freq=self._ring_timer_hz, callback=self._action)
            return _ring_timer
        else:
            from machine import Timer

            _ring_timer = Timer(1)
            _ring_timer.init(freq=self._ring_timer_hz, mode=Timer.PERIODIC, callback=self._action)
            return _ring_timer

    # ring processing ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def reset_ring(self):
        for pixel in self._ring_model:
            pixel.reset()
        self._update_ring()

    def _rotate_ring(self, shift=1):
        if abs(shift) > 24:
            raise ValueError('shift value outside of bounds.')
        shift *= self._rotate_direction
        self._ring_offset = (self._ring_offset + shift) % 24
        self._update_ring()

    def _update_ring(self):
        for index in range(24):
            rotated_index = (index - self._ring_offset) % 24
            self._ring.set_color(index, self._ring_model[rotated_index].color)

    def set_ring_color(self, index, color):
#       self._log.info('set ring color at {} to {}'.format(index, color))
        if isinstance(color, Color):
            pass
        elif isinstance(color, tuple):
            color = Color("COLOR_({},{},{})".format(*color), (color[0], color[1], color[2]))
        else:
            raise TypeError('expected Color or tuple.')
        actual_index = (index + self._ring_offset) % 24
        self._ring_model[actual_index].base_color = color
        self._ring_model[actual_index].color = color.rgb
        self._ring.set_color(index, color.rgb)

    def _restart_timer(self, freq=None):
        '''
        Restart timer 1 if either rotate or theme is enabled.
        '''
        if freq:
            self._ring_timer_hz = freq
        self._ring_timer.deinit()
        if self._enable_rotate or self._enable_theme:
#           self._ring_timer.init(freq=self._ring_timer_hz)
            self._ring_timer.init(freq=self._ring_timer_hz, callback=self._action)

    def _action(self, t):
        if self._enable_rotate:
            self._rotate_pending = True
        if self._enable_theme:
            self._theme_pending = True

    def tick(self, delta_ms):
        # handle deferred ring updates first
        if self._rotate_pending:
            self._rotate_pending = False
            self._rotate_ring()
        if self._theme_pending:
            self._theme_pending = False
            self._theme()
        # then handle pixel off and parent tick
        super().tick(delta_ms)

    def _quiet(self):
        self._status_enabled = False
        self._enable_rotate  = False
        self._enable_theme   = False
        self._log.info("set quiet.")

    # theme processing ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _populate(self, count, palette_name):
        palette = self._palettes.get(palette_name)
        if palette is None:
            self._log.error("no such palette: '{}'".format(palette_name))
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
        self._update_ring()

    def _init_theme(self, reset=False):
        # disable heartbeat when theme starts
        self._enable_heartbeat(False)
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
        self._update_ring()

    def _theme(self):
        for index in range(24):
            pixel = self._ring_model[index]
            if not pixel.is_active():
                continue
            pixel.phase = (pixel.phase + 1.0 / self._pulse_steps) % 1.0
            brightness = (math.sin(pixel.phase * 2 * math.pi) + 1) / 2
            r, g, b = pixel.base_color.rgb
            pixel.color = (int(r * brightness), int(g * brightness), int(b * brightness))
        self._update_ring()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def print_help(self):
        super().print_help()
        print('''    ring clear |                            # set all ring pixels off
       | all ( off | clear | <name> )       # set all ring pixels off or to color
    rgb index red green blue                # set a ring pixel to the RGB color
    rotate on | off | fwd | cw | rev | ccw  # control ring pixel rotation
       | hz <n>                             # set rotation frequency
    theme on | off                          # enable/disable theme pulsation
       | hz <n>                             # set theme pulse frequency
       | pixels <count>                     # enable randomly-placed pixels in current palette
       | palette <name> <count>             # set palette with count of randomly-placed pixels
    quiet                                   # turn off all services
''')

    def pre_process(self, cmd, arg0, arg1, arg2, arg3, arg4):
        '''
        Pre-process the arguments, returning a response and color if a match occurs.
        Such a match precludes further processing.
        '''
#       self._log.info("ring: pre-process command '{}' with arg0: '{}'; arg1: '{}'; arg2: '{}'; arg3: '{}'; arg4: '{}'".format(cmd, arg0, arg1, arg2, arg3, arg4))
#       if arg0 == "__extend_here__":
#           return None, None

        if arg0 not in {"rgb", "ring", "rotate", "theme", "quiet"}: # preemptively check
            return super().pre_process(cmd, arg0, arg1, arg2, arg3, arg4)

        elif arg0 == "rgb":
            # e.g., rgb 3 130 40 242
            self._enable_heartbeat(False)
            _show_state = False
            try:
                if arg4 is not None:
                    index = int(arg1)
                    red   = int(arg2)
                    green = int(arg3)
                    blue  = int(arg4)
                    self._log.info("rgb: index: {}; red: '{}'; green: '{}'; blue: '{}'".format(index, red, green, blue))
#                   self._ring.set_color(index - 1, (red, green, blue))
                    self.set_ring_color(index - 1, (red, green, blue))
                    return Controller._PACKED_ACK, COLOR_DARK_GREEN
                else:
                    raise Exception('invalid number of arguments for rgb.')
            except Exception as e:
                self._log.error('{} raised by rgb command: {}'.format(type(e), e))
                sys.print_exception(e)
                return Controller._PACKED_ERR, COLOR_RED

        elif arg0 == "ring":
            try:
                _rotating = self._enable_rotate
                if arg1 == 'clear':
                    self.reset_ring()
                    return Controller._PACKED_ACK, COLOR_BLACK
                elif arg1 == 'all':
                    if arg2 == 'off' or arg2 == 'clear':
                        self.reset_ring()
                        return Controller._PACKED_ACK, COLOR_DARK_GREEN
                    else:
                        color = self._get_color(arg2, arg3)
                        if not color:
                            self._log.error("could not find color: arg2: '{}'; arg3: '{}'".format(arg2, arg3))
                            return Controller._PACKED_ERR, COLOR_RED
                        for idx in range(self._ring_count):
                            self.set_ring_color(idx, color)
                        return Controller._PACKED_ACK, COLOR_DARK_GREEN

                else:
#                   self._log.info("ring command '{}' with arg0: '{}'; arg1: '{}'; arg2: '{}'; arg3: '{}'; arg4: '{}'".format(cmd, arg0, arg1, arg2, arg3, arg4))
                    index = int(arg1) - 1
                    if 0 <= index <= 23:
                        color = self._get_color(arg2, arg3)
                        if color:
                            self.set_ring_color(index, color)
                            return Controller._PACKED_ACK, COLOR_DARK_GREEN
                    else:
                        self._log.error("index value {} out of bounds (1-{}).".format(index, self._ring_count))
                        return Controller._PACKED_ERR, COLOR_RED
                self._log.error("could not process input: '{}'".format(cmd))
            except Exception as e:
                self._log.error('{} raised by ring command: {}'.format(type(e), e))
                sys.print_exception(e)
                raise
            finally:
                self._enable_rotate = _rotating
            return Controller._PACKED_ERR, COLOR_RED

        elif arg0 == "rotate":
            if arg1:
                if arg1 == 'on':
                    self._enable_rotate = True
                    self._restart_timer()
                    return Controller._PACKED_ACK, COLOR_DARK_GREEN
                elif arg1 == 'off':
                    self._enable_rotate = False
                    self._restart_timer()
                    return Controller._PACKED_ACK, COLOR_DARK_GREEN
                elif arg1 == 'fwd' or arg1 == 'cw':
                    self._rotate_direction = 1
                    return Controller._PACKED_ACK, COLOR_DARK_GREEN
                elif arg1 == 'rev' or arg1 == 'ccw':
                    self._rotate_direction = -1
                    return Controller._PACKED_ACK, COLOR_DARK_GREEN
                elif arg1 == 'hz':
                    hz = int(arg2)
                    if hz > 0:
                        self._restart_timer(hz)
                        return Controller._PACKED_ACK, COLOR_DARK_GREEN
                    else:
                        return Controller._PACKED_ERR, COLOR_RED
                else:
                    shift = int(arg1)
                    self._rotate_ring(shift)
                return Controller._PACKED_ACK, COLOR_DARK_GREEN
            else:
                return Controller._PACKED_ERR, COLOR_RED

        elif arg0 == "theme":
            try:
#               self._log.info("theme '{}' with arg0: '{}'; arg1: '{}'; arg2: '{}'; arg3: '{}'; arg4: '{}'".format(cmd, arg0, arg1, arg2, arg3, arg4))
                if arg1:
                    if arg1 == 'on':
                        self._init_theme()
                        self._enable_theme = True
                        return Controller._PACKED_ACK, COLOR_DARK_GREEN
                    elif arg1 == 'off':
                        self._enable_theme = False
                        self._restart_timer()
                        return Controller._PACKED_ACK, COLOR_DARK_GREEN
                    elif arg1 == 'hz':
                        hz = int(arg2)
                        if hz > 0:
                            self._restart_timer(hz)
                            return Controller._PACKED_ACK, COLOR_DARK_GREEN
                        return Controller._PACKED_ERR, COLOR_RED
                    elif arg1 == 'pixels':
                        _themed = self._enable_theme
                        try:
                            self._enable_theme = False
                            target = int(arg2)
                            if 1 <= target <= self._ring_count:
                                self._theme_target_pixels = target
                                self._init_theme(reset=True)
                                return Controller._PACKED_ACK, COLOR_DARK_GREEN
                            return Controller._PACKED_ERR, COLOR_RED
                        finally:
                            self._enable_theme = _themed

                    elif arg1 in self._palettes:
                        _themed = self._enable_theme
                        _rotating = self._enable_rotate
                        self._enable_rotate = False
                        self._enable_theme = False
                        self._ring_offset = 0
                        try:
                            target = int(arg2)
                            self._theme_target_pixels = target
                            if 1 <= target <= self._ring_count:
                                self._populate(target, arg1)
                                return Controller._PACKED_ACK, COLOR_DARK_GREEN
                            else:
                                return Controller._PACKED_ERR, COLOR_RED
                        except Exception as e:
                            self._log.error('{} raised with palette name: {}'.format(type(e), e))
                            return Controller._PACKED_ERR, COLOR_RED
                        finally:
                            self._enable_theme = _themed
                            self._enable_rotate = _rotating
                        return Controller._PACKED_ACK, COLOR_DARK_GREEN

                    elif arg1 == 'steps':
                        steps = int(arg2)
                        if steps > 0:
                            self._pulse_steps = steps
                            return Controller._PACKED_ACK, COLOR_DARK_GREEN
                        return Controller._PACKED_ERR, COLOR_RED
                    else:
                        self._log.error("could not process input:  '{}'".format(cmd))
                        return Controller._PACKED_ERR, COLOR_RED
                    return Controller._PACKED_ACK, COLOR_DARK_GREEN

                else:
                    return Controller._PACKED_ERR, COLOR_RED
            except Exception as e:
                self._log.error('{} raised by theme command: {}'.format(type(e), e))
                sys.print_exception(e)
                raise

        elif arg0 == "quiet":
            self._quiet()
            return Controller._PACKED_ACK, COLOR_DARK_GREEN

        else:
            return super().pre_process(cmd, arg0, arg1, arg2, arg3, arg4)

    def process(self, cmd):
        '''
        Processes the callback from the I2C slave.
        '''
        if self._strip and self._status_enabled:
            self._strip.set_color(self._status_pixel, COLOR_CYAN)
        super().process(cmd)

    def post_process(self, cmd, arg0, arg1, arg2, arg3, arg4):
        '''
        Post-process the arguments, returning a NACK and color if no match on arg0 occurs.
        '''
#       self._log.info("post-process command '{}' with arg0: '{}'; arg1: '{}'; arg2: '{}'; arg3: '{}'; arg4: '{}'".format(cmd, arg0, arg1, arg2, arg3, arg4))
        if arg0 == "__extend_here__":
            return None, None
        else:
            return super().post_process(cmd, arg0, arg1, arg2, arg3, arg4)

#EOF
