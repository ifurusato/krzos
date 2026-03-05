#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-03-05

import sys
import time
import math, random
from machine import RTC
from colorama import Fore, Style

from colors import*
from logger import Logger, Level
from message_util import pack_message

class Controller:
    _AUTOSTART_SERVICES = True           # auto-start services after delay
    _AUTOSTART_DELAY_MS = 3000           # delay in milliseconds before auto-start
    # pre-packed constant responses
    _PACKED_ACK  = pack_message('ACK')   # acknowledge okay
    _PACKED_NACK = pack_message('NACK')  # acknowledge bad command
    _PACKED_ERR  = pack_message('ERR')   # processing error occurred
    _PACKED_PING = pack_message('PING')  # processing error occurred
    _PACKED_DATA = pack_message('0000 1111 2222 3333 4444 5555 6666 7777') # sample data, packed
    '''
    A controller for command strings received from the I2CSlave.

    Args:
        config: the application configuration
        pixel:  the optional NeoPixel
        strip:  the optional NeoPixel strip
    '''
    def __init__(self, config, pixel=None, strip=None, level=Level.INFO):
        self._log = Logger('controller', level=level)
        self._startup_ms            = time.ticks_ms()
        self._config                = config
        self._name                  = config['name']
        self._family                = config['family']
        self._slave                 = None
        if pixel is None:
            self._pixel = self._create_pixel()
        else:
            self._pixel = pixel
        if strip is None:
            self._strip = self._create_strip()
        else:
            self._strip = strip
        # heartbeat feature
        self._heartbeat_enabled     = False
        self._heartbeat_on_time_ms  = 50
        self._heartbeat_off_time_ms = 2950
        self._heartbeat_timer       = 0
        self._heartbeat_state       = False
        self._stop_at               = None
        self._pixel_persist         = False
        # instantiate pixel and timer
        self._pixel_timer           = None
        self._pixel_timer_freq_hz   = 50
        self._create_pixel_timer()
        self._services_started      = False
        self._autostart_delay_ms    = Controller._AUTOSTART_DELAY_MS
        self._log.info('ready.')

    @property
    def name(self):
        return self._name

    @property
    def pixel(self):
        return self._pixel

    @property
    def strip(self):
        return self._strip

    def _create_pixel(self):
        from pixel import Pixel

        _pixel_pin = self._config['pixel_pin']
        if self._family == 'TINYS3':
            import tinys3

            _pixel_pin = tinys3.RGB_DATA
            tinys3.set_pixel_power(1)
        _pixel = Pixel(pin=_pixel_pin, pixel_count=1, color_order=self._config['color_order'])
        self._log.info('NeoPixel configured on pin {}'.format(_pixel_pin))
        _pixel.set_color(0, COLOR_CYAN)
        time.sleep_ms(250)
        _pixel.set_color(0, COLOR_BLACK)
        return _pixel

    def _create_pixel_timer(self):
        try:
            from machine import Timer

            self._pixel_timer = Timer(0)
            self._pixel_timer.init(freq=self._pixel_timer_freq_hz, callback=self._led_off)
        except Exception as e:
            sys.print_exception(e)

    def _create_strip(self):
        from pixel import Pixel

        _strip_brightness = 0.1  # default: 0.33
        _strip_pin   = self._config['strip_pin']
        _strip_count = self._config['strip_count']
        _strip = Pixel(
            pin=_strip_pin,
            pixel_count=_strip_count,
            color_order=self._config['strip_color_order'],
            brightness=_strip_brightness)
        self._log.info('NeoPixel strip with {} pixels configured on pin {}'.format(_strip_count, _strip_pin))
        _strip.set_color(0, COLOR_BLACK)
        return _strip

    # public API ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def pixel(self):
        return self._pixel

    def tick(self, delta_ms):
        if self._heartbeat_enabled:
            self._heartbeat(delta_ms)
        if self._stop_at:
            if time.ticks_diff(time.ticks_ms(), self._stop_at) >= 0:
                self._stop_at = None
                if not self._pixel_persist:
                    self._pixel.set_color(0, COLOR_MIDNIGHT)
        if Controller._AUTOSTART_SERVICES and not self._services_started:
            if time.ticks_diff(time.ticks_ms(), self._startup_ms) >= self._autostart_delay_ms:

                self._services_started = True
                self._start_services()

    def set_slave(self, slave):
        self._slave = slave
        self._slave.add_callback(self._on_command)

    def pre_process(self, cmd, arg0, arg1, arg2, arg3, arg4):
        '''
        Pre-process the arguments, returning a response and color if a match occurs.
        Such a match precludes further processing.
        '''
#       self._log.info("controller: pre-process command '{}' with arg0: '{}'; arg1: '{}'; arg2: '{}'; arg3: '{}'; arg4: '{}'".format(cmd, arg0, arg1, arg2, arg3, arg4))
        if arg0 == "__extend_here__":
            return None, None
        else:
            return None, None

    def print_help(self):
        print('''
Commands:

    name                                    # return the name of the configured board
    time get | set <timestamp>              # set/get RTC time
    pixel off | <color> | <n> <color>       # control NeoPixel
    persist on | off                        # persist pixel after setting
    rgb <red> <green> <blue>                # set NeoPixel to RGB
    heartbeat on | off                      # control heartbeat flash
    ping                                    # returns "PING"
    data                                    # return sample data
    reset                                   # force hardware reset''')

    def process(self, cmd):
        '''
        Processes the callback from the I2C slave, returning 'ACK', 'NACK' or 'ERR'.
        This calls pre_process() prior as a way of overriding this method.

        See get_help() for list of available commands.
        '''
        _show_state = True
        if _show_state:
            self._stop_at = time.ticks_add(time.ticks_ms(), 1000)  # stop 1 second later
        _exit_color = COLOR_BLACK # default
#       self._pixel.set_color(0, COLOR_CYAN)
        try:
#           self._log.info("cmd: '{}'".format(cmd))
            parts = cmd.lower().split()
            if len(parts) == 0:
                _exit_color = COLOR_RED
                return Controller._PACKED_ERR
            _arg0 = parts[0]
            _arg1 = parts[1] if len(parts) > 1 else None
            _arg2 = parts[2] if len(parts) > 2 else None
            _arg3 = parts[3] if len(parts) > 3 else None
            _arg4 = parts[4] if len(parts) > 4 else None
#           self._log.info("cmd: '{}'; arg0: '{}'; arg1: '{}'; arg2: '{}'; arg3: '{}'; arg4: '{}'".format(cmd, _arg0, _arg1, _arg2, _arg3, _arg4))

            # pre-process
            _response, __exit_color = self.pre_process(cmd, _arg0, _arg1, _arg2, _arg3, _arg4)
            if _response is not None:
                _exit_color = __exit_color
                return _response

            if _arg0 == "name":
                _exit_color = COLOR_DARK_GREEN
                return pack_message(self._name)

            elif _arg0 == "help":
                self.print_help()
                _exit_color = COLOR_DARK_GREEN
                return Controller._PACKED_ACK

            elif _arg0 == "time":
#               self._log.info('time: {}, {}'.format(_arg1, _arg2))
                if _arg1 == 'set':
                    _exit_color = COLOR_DARK_GREEN
                    return self._set_time(_arg2)
                elif _arg1 == 'get':
                    _exit_color = COLOR_DARK_GREEN
                    return pack_message(self._rtc_to_iso(RTC().datetime()))
                _exit_color = COLOR_RED
                return Controller._PACKED_ERR

            elif _arg0 == "pixel":
                # any calls to pixel disable heartbeat
                self._enable_heartbeat(False)
                _show_state = False
                if self._pixel:
                    if _arg1 == 'off' or _arg1 == 'clear':
                        color = COLOR_BLACK
                    else:
                        color = self._get_color(_arg1, _arg2)
                    if not color:
                        self._log.warning("could not find color: arg1: '{}'; arg2: '{}'".format(_arg1, _arg2))
                        _exit_color = COLOR_RED
                        return Controller._PACKED_ERR
                    self._pixel.set_color(0, color)
                    return Controller._PACKED_ACK
                else:
                    self._log.warning('no pixel available.')
                _exit_color = COLOR_RED
                return Controller._PACKED_ERR

            elif _arg0 == "persist":
                if _arg1 == 'on':
                    self._pixel_persist = True
                    _exit_color = COLOR_BLACK # otherwise it'd be on
                    return Controller._PACKED_ACK
                elif _arg1 == 'off':
                    self._pixel_persist = False
                    _exit_color = COLOR_DARK_GREEN
                    return Controller._PACKED_ACK
                else:
                    self._log.error("unrecognised persist argument: {}'".format(_arg1))
                    _exit_color = COLOR_RED
                    return Controller._PACKED_ERR

            elif _arg0 == "heartbeat":
                if _arg1 == 'on':
                    self._pixel_persist = False # contradictory, so off
                    self._enable_heartbeat(True)
                    _exit_color = COLOR_DARK_GREEN
                    return Controller._PACKED_ACK
                elif _arg1 == 'off':
                    self._enable_heartbeat(False)
                    _exit_color = COLOR_DARK_GREEN
                    return Controller._PACKED_ACK
                else:
                    self._log.error("unrecognised argument: '{}'".format(_arg1))
                    _exit_color = COLOR_RED
                    return Controller._PACKED_ERR

            elif _arg0 == "rgb":
                print('controller.rgb')
                # e.g., rgb 130 40 242
                self._enable_heartbeat(False)
                red   = int(_arg1)
                green = int(_arg2)
                blue  = int(_arg3)
                self._log.info("rgb: red: '{}'; green: '{}'; blue: '{}'".format(index, red, green, blue))
                self._pixel.set_color(0, (red, green, blue))
                return Controller._PACKED_ACK

            elif _arg0 == "ping":
                _exit_color = COLOR_DARK_GREEN
                return Controller._PACKED_PING

            elif _arg0 == "data": # data request (TBD)
#               _exit_color = COLOR_FUCHSIA
#               return Controller._PACKED_DATA
                _message, _exit_color = self._get_data()
                return pack_message(_message)

            elif _arg0 == "reset":
                import machine

                self._log.info('performing microcontroller reset…')
                machine.reset()
                return Controller._PACKED_ACK

            else:
                self._log.warning("unrecognised command '{}' as arguments: {}{}{}{}{}".format(
                        cmd,
                        "; arg0: '{}'".format(_arg0) if _arg0 else '',
                        "; arg1: '{}'".format(_arg1) if _arg1 else '',
                        "; arg2: '{}'".format(_arg2) if _arg2 else '',
                        "; arg3: '{}'".format(_arg3) if _arg3 else '',
                        "; arg2: '{}'".format(_arg4) if _arg4 else ''))
                return Controller._PACKED_NACK, COLOR_ORANGE

        except Exception as e:
            self._log.error("{} raised by controller: {}".format(type(e), e))
            sys.print_exception(e)
            _exit_color = COLOR_RED
            return Controller._PACKED_ERR
        finally:
            if _show_state:
                self._pixel.set_color(0, _exit_color)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _get_data(self):
        '''
        Return data (TBD).
        '''
        _data = '0000 1111 2222 33333' # 20 chars
#       _data = '0000 1111 2222 3333 4444 5555 6666 77777' # 40 characters
#       _data = '0000 1111 2222 3333 4444 5555 6666 7777 8888 9999 AAAA BBBB CC' # 62 chars
#       self._log.info('data: {} chars.'.format(len(_data)))
        _exit_color = COLOR_FUCHSIA
        return _data, _exit_color

    def _on_command(self, cmd):
        '''
        The callback from the I2C slave, passes the command on for processing,
        returning the result.
        '''
        return self.process(cmd)

    def _start_services(self):
        '''
        This method is called upon startup, after a preset delay. It can be overridden
        as necessary to start any application-level services.
        '''
        time_elapsed = time.ticks_ms() - self._startup_ms
        self._log.info('starting services after {}ms'.format(time_elapsed))
        self._enable_heartbeat(True)
        pass # whatever services to be started after a delay

    def _enable_heartbeat(self, enabled):
        self._heartbeat_enabled = enabled
        if not enabled:
            self._pixel_timer.deinit()

    def _led_off(self, timer=None):
        if self._pixel:
            self._pixel.set_color(0, COLOR_BLACK)

    def _beat(self):
        if self._pixel:
            self._pixel.set_color(0, COLOR_DARK_CYAN)
        if self._pixel_timer:
            self._pixel_timer.deinit()
#           self._pixel_timer.init(freq=self._pixel_timer_freq_hz)
            self._pixel_timer.init(freq=self._pixel_timer_freq_hz, callback=self._led_off)

    def _heartbeat(self, delta_ms):
        self._heartbeat_timer += delta_ms
        if self._heartbeat_state:
            if self._heartbeat_timer >= self._heartbeat_on_time_ms:
                self._heartbeat_state = False
                self._heartbeat_timer = 0
        else:
            if self._heartbeat_timer >= self._heartbeat_off_time_ms:
                self._heartbeat_state = True
                self._beat()
                self._heartbeat_timer = 0

    def _get_color(self, name, second_token):
        if second_token: # e.g., "dark cyan"
            name = '{} {}'.format(name, second_token)
        return Color.get(name)

    def _set_rotation_pending(self, t):
        self._rotation_pending = True

    def _action(self, t):
        pass

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
            self._log.info('time before: ' + Fore.GREEN + '{}'.format(self._rtc_to_iso(RTC().datetime())))
            RTC().datetime(self._parse_timestamp(timestamp))
            self._log.info('time after:  ' + Fore.GREEN + '{}'.format(self._rtc_to_iso(RTC().datetime())))
            return Controller._PACKED_ACK
        except Exception as e:
            self._log.error("{} raised by controller: {}".format(type(e), e))
            return Controller._PACKED_ERR

#EOF
