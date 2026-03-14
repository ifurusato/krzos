#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2024-07-04
# modified: 2026-03-09

import os
from pathlib import Path
from datetime import datetime
from threading import Thread
from collections import OrderedDict
from psutil import cpu_percent, virtual_memory, disk_usage, boot_time, net_if_addrs, net_if_stats
import itertools
import subprocess
import socket
from colorama import init, Fore, Style
init()

from luma.core.render import canvas
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from luma.core.error import DeviceNotFoundError
from PIL import ImageFont
from ads1015 import ADS1015

from core.rate import Rate
from core.util import Util
from core.logger import Logger, Level
from core.component import Component
from hardware.ina260_sensor import Ina260

class Monitor(Component):
    '''
    Displays multiple lines of system monitoring data on an OLED display.

    Always uses a daemon thread for updates, keeping it off the IRQ clock
    to avoid blocking motor control timing.

    The Monitor is disabled by default. If the device cannot be found the
    class will log an error but not otherwise function differently.

    :param: config            the application configuration
    :param: external_clock    ignored; retained for call-site compatibility
    :param: use_thread        ignored; always uses a thread
    :param: level             the logging level
    '''
    def __init__(self, config, external_clock=None, use_thread=False, level=Level.INFO):
        self._log = Logger('monitor', level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        if config is None:
            raise ValueError('no configuration provided.')
        if Util.already_running('monitor_exec.py'):
            raise RuntimeError('monitor is already running.')
        self._message = None
        # configuration
        _cfg = config['kros'].get('hardware').get('monitor')
        _i2c_port       = _cfg.get('i2c_port')
        _i2c_address    = _cfg.get('i2c_address')
        _rotate         = _cfg.get('rotate')
        _update_rate_hz = _cfg.get('update_rate_hz')
        self._permit_callback = _cfg.get('permit_callback')
        self._batt_max  = 21.5
        self._pi_max    = 5.25
        self._logic_max = 3.5
        self._max_current = 5.0
        self._network_interface_name = None
        self._bar_width       = 52
        self._bar_width_full  = 95
        self._bar_height      = 8
        self._bar_margin_top  = 3
        self._margin_x_bar    = 31
        self._margin_x_figure = 83
        self._margin_y_line   = [0, 12, 24, 36, 48, 60, 72, 84, 96, 108]
        self._margin_y_line_m = [0, 12, 24, 40, 56, 72, 88, 104, 108, 108]
        _font_size            = _cfg.get('font_size')
        _font_size_full       = _cfg.get('font_size_full')
        _font_size_message    = _cfg.get('font_size_message')
        _font_name            = _cfg.get('font_name')
        _font_file            = os.path.join(os.path.dirname(__file__), 'fonts/' + _font_name)
        self._font_default    = ImageFont.truetype(_font_file, _font_size)
        self._font_full       = ImageFont.truetype(_font_file, _font_size_full)
        self._font_message    = ImageFont.truetype(_font_file, _font_size_message)
        _contrast             = _cfg.get('contrast')
        self.__callback = None
        # get device
        self._device = None
        try:
            self._device = sh1106(i2c(port=_i2c_port, address=_i2c_address), width=128, height=128, rotate=_rotate)
            self._device.contrast(_contrast)
        except DeviceNotFoundError as e:
            self._log.error('no monitor available: display not found at address 0x{:02X}: {}'.format(_i2c_address, e))
        # ADS1015
        self._ads1015 = ADS1015()
        chip_type = self._ads1015.detect_chip_type()
        self._log.info('found chip type: {}'.format(chip_type))
        self._ads1015.set_mode("single")
        self._ads1015.set_programmable_gain(2.048)
        if chip_type == 'ADS1015':
            self._ads1015.set_sample_rate(1600)
        else:
            self._ads1015.set_sample_rate(860)
        self._reference = self._ads1015.get_reference_voltage()
        self._log.info('Reference voltage: {:6.3f}v'.format(self._reference))
        _component_registry = Component.get_registry()
        self._ina260 = _component_registry.get('ina260')
        if self._ina260 is None:
            self._ina260 = Ina260(config, level=level)
        # always use a thread; never block the IRQ clock
        self._log.info('using thread for update loop at {:d}Hz.'.format(_update_rate_hz))
        self._update_loop_thread = Thread(name='monitor-update', target=Monitor._update_loop, args=[self], daemon=True)
        self._rate = Rate(_update_rate_hz, level=Level.INFO)
        self.enable()
        self._update_loop_thread.start()
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _update_loop(self):
        try:
            os.nice(10)
        except Exception:
            pass
        while self.enabled:
            self.update()
            self._rate.wait()

    @property
    def permit_callback(self):
        return self._permit_callback

    def set_callback(self, callback):
        '''
        Set a callback for a substitution on the last line, which should
        be a lambda function.
        '''
        self.__callback = callback

    def get_callback_value(self):
        return self.__callback()

    def get_temp(self):
        temp = float(subprocess.getoutput("vcgencmd measure_temp").split("=")[1].split("'")[0])
        return temp

    def get_cpu(self):
        return cpu_percent()

    def get_mem(self):
        return virtual_memory().percent

    def get_disk_usage(self):
        usage = disk_usage("/")
        return usage.used / usage.total * 100

    def get_uptime(self):
        uptime = ("%s" % (datetime.now() - datetime.fromtimestamp(boot_time()))).split(".")[0]
        return "UpTime: {}".format(uptime)

    def get_timestamp(self):
        _now = datetime.now()
        return 'TS: {}'.format(_now.strftime('%Y-%m-%dT%H:%M:%S'))

    def find_single_ipv4_address(self, addrs):
        for addr in addrs:
            if addr.family == socket.AddressFamily.AF_INET:
                return addr.address

    def get_ipv4_address(self, interface_name=None):
        if_addrs = net_if_addrs()
        if isinstance(interface_name, str) and interface_name in if_addrs:
            addrs = if_addrs.get(interface_name)
            address = self.find_single_ipv4_address(addrs)
            return address if isinstance(address, str) else ""
        else:
            if_stats = net_if_stats()
            if_stats_filtered = {key: if_stats[key] for key, stat in if_stats.items() if "loopback" not in stat.flags}
            if_names_sorted = [stat[0] for stat in sorted(if_stats_filtered.items(), key=lambda x: (x[1].isup, x[1].duplex), reverse=True)]
            if_addrs_sorted = OrderedDict((key, if_addrs[key]) for key in if_names_sorted if key in if_addrs)
            for _, addrs in if_addrs_sorted.items():
                address = self.find_single_ipv4_address(addrs)
                if isinstance(address, str):
                    return address
            return ""

    def get_pose(self):
        return "Pose: 12345, 12345, 360°"

    def get_ip(self, network_interface_name):
        '''
        None : find suitable IPv4 address among all network interfaces
        or specify the desired interface name as string.
        '''
        self._network_interface_name = network_interface_name
        return "IP: {}".format(self.get_ipv4_address(self._network_interface_name))

    def get_battery(self):
        return self._ads1015.get_compensated_voltage(channel='in0/ref', reference_voltage=self._reference)

    def get_5v_regulator(self):
        return self._ads1015.get_compensated_voltage(channel='in2/ref', reference_voltage=self._reference)

    def get_3v3(self):
        return self._ads1015.get_compensated_voltage(channel='in1/ref', reference_voltage=self._reference)

    def get_current(self):
        return self._ina260.current

    def _draw_text(self, draw, margin_x, line_num, text):
        draw.text((margin_x, self._margin_y_line[line_num]), text, font=self._font_default, fill="white")

    def _draw_message(self, draw, margin_x, line_num, text):
        draw.text((margin_x, self._margin_y_line_m[line_num]), text, font=self._font_message, fill="white")

    def _draw_bar(self, draw, line_num, percent):
        top_left_y = self._margin_y_line[line_num] + self._bar_margin_top
        draw.rectangle((self._margin_x_bar, top_left_y, self._margin_x_bar + self._bar_width, top_left_y + self._bar_height), outline="white")
        draw.rectangle((self._margin_x_bar, top_left_y, self._margin_x_bar + self._bar_width * percent / 100, top_left_y + self._bar_height), fill="white")

    def _draw_bar_full(self, draw, line_num):
        top_left_y = self._margin_y_line[line_num] + self._bar_margin_top
        draw.rectangle((self._margin_x_bar, top_left_y, self._margin_x_bar + self._bar_width_full, top_left_y + self._bar_height), fill="white")
        draw.text((65, top_left_y - 2), "100 %", font=self._font_full, fill="black")

    def clear(self):
        if self._device:
            self._device.clear()

    def set_message(self, message):
        self._message = message

    def _display_message(self, draw):
        _nlines = self._message.count('\n')
        if _nlines < 2:
            self._draw_message(draw, 0, 3, self._message)
        else:
            _lines = self._message.split('\n')
            for n in range(0,len(_lines)):
                _line = _lines[n]
                self._draw_message(draw, 0, n+2, _line)

    def update(self):
        if self._device and self.enabled:
            with canvas(self._device) as draw:
                if self._message is not None:
                    self._display_message(draw)
                    return

                SHOW_DISK = True
                SHOW_3V3  = True
                SHOW_POSE = False

                # line 0 : temperature ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                line = 0
                _temp = self.get_temp()
                self._draw_text(draw, 0, line, "Temp")
                self._draw_text(draw, self._margin_x_figure, line, " {:5.1f}C".format(_temp))

                # line 1 : cpu ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                line += 1
                _cpu = self.get_cpu()
                self._draw_text(draw, 0, line, "CPU")
                if _cpu < 100:
                    self._draw_text(draw, self._margin_x_figure, 1, " {:5.2f}%".format(_cpu))
                    self._draw_bar(draw, line, _cpu)
                else:
                    self._draw_bar_full(draw, line)

                # line 2 : memory ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                line += 1
                _mem = self.get_mem()
                self._draw_text(draw, 0, line, "Mem")
                if _mem < 100:
                    self._draw_text(draw, self._margin_x_figure, line, " {:5.1f}%".format(_mem))
                    self._draw_bar(draw, line, _mem)
                else:
                    self._draw_bar_full(draw, line)

                # line 3 : disk usage ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                if SHOW_DISK:
                    line += 1
                    _disk = self.get_disk_usage()
                    self._draw_text(draw, 0, line, "Disk")
                    if _disk < 100:
                        self._draw_text(draw, self._margin_x_figure, line, " {:5.1f}%".format(_disk))
                        self._draw_bar(draw, line, _disk)
                    else:
                        self._draw_bar_full(draw, line)

                # line 4 : battery ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                line += 1
                _battery = self.get_battery()
                self._draw_text(draw, 0, line, "Batt")
                if _battery < self._batt_max:
                    self._draw_text(draw, self._margin_x_figure, line, " {:5.1f}V".format(_battery))
                    self._draw_bar(draw, line, ( _battery / self._batt_max * 100.0) )
                else:
                    self._draw_bar_full(draw, line)

                # line 5 : 5V regulator ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                line += 1
                _5v_reg = self.get_5v_regulator()
                self._draw_text(draw, 0, line, "5vRg")
                if _5v_reg < self._pi_max:
                    self._draw_text(draw, self._margin_x_figure, line, " {:5.2f}V".format(_5v_reg))
                    self._draw_bar(draw, line, ( _5v_reg / self._pi_max * 100.0) )
                else:
                    self._draw_bar_full(draw, line)

                if SHOW_3V3:
                    # line 6 : 3V3 logic ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                    line += 1
                    _3v3_reg = self.get_3v3()
                    self._draw_text(draw, 0, line, "3v3")
                    if _3v3_reg < self._logic_max:
                        self._draw_text(draw, self._margin_x_figure, line, " {:5.2f}V".format(_3v3_reg))
                        self._draw_bar(draw, line, ( _3v3_reg / self._logic_max * 100.0) )
                    else:
                        self._draw_bar_full(draw, line)

                # line 6 : current ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                line += 1
                _current = self.get_current()
                self._draw_text(draw, 0, line, "Curr")
                if _current < 1.0:
                    _display_current = int(_current * 1000.0)
                    _c_un = " {:5.0f}mA"
                else:
                    _display_current = _current
                    _c_un = " {:5.2f}A"
                if _current < self._max_current:
                    self._draw_text(draw, self._margin_x_figure, line, _c_un.format(_display_current))
                    self._draw_bar(draw, line, ( _current / self._max_current * 100.0) )
                else:
                    self._draw_bar_full(draw, line)

                # line 7 : pose ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                if SHOW_POSE:
                    line += 1
                    self._draw_text(draw, 0, line, self.get_pose())

                # line 8 : IP address  ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                line += 1
                self._draw_text(draw, 0, line, self.get_ip(self._network_interface_name))

                # line 9 : uptime ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                line += 1
                self._draw_text(draw, 0, line, self.get_uptime())

    def enable(self):
        if not self.enabled:
            super().enable()
            if self._device is None:
                self._log.info('monitor not enabled (no device).')
            else:
                self._log.info('enabled monitor.')
        elif self.closed:
            self._log.warning('cannot enable monitor: already closed.')

    def disable(self):
        self.clear()
        super().disable()
        self._log.info('disabled monitor.')

#EOF
