#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License.
#
# author:   Ichiro Furusato
# created:  2020-01-14
# modified: 2025-12-28

import math
import time
from colorama import Fore, Style

def enum(**enums: int):
    return type('Enum', (), enums)

Level = enum(DEBUG=10, INFO=20, WARN=30, ERROR=40)

class Logger:

    __color_debug    = Fore.BLUE    + Style.DIM
    __color_info     = Fore.CYAN    + Style.NORMAL
    __color_warning  = Fore.YELLOW  + Style.NORMAL
    __color_error    = Fore.RED     + Style.NORMAL
    __color_reset    = Style.RESET_ALL

    def __init__(self, name, level=Level.INFO):
        self._include_timestamp = True
        self.__DEBUG_TOKEN = 'DEBUG'
        self.__INFO_TOKEN  = 'INFO '
        self.__WARN_TOKEN  = 'WARN '
        self.__ERROR_TOKEN = 'ERROR'
        self._name_format  = '{:<14} : '
        self._boot_time = time.time()
        self._boot_ticks = time.ticks_ms()
        self._name   = name
        self.level = level

    def _get_timestamp(self):
        while True:
            ticks1 = time.ticks_ms()
            t = time.localtime()
            ticks2 = time.ticks_ms()
            if ticks1 // 1000 == ticks2 // 1000:
                ms = ticks1 % 1000
                return "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}.{:03d}Z".format(
                    t[0], t[1], t[2], t[3], t[4], t[5], ms
                )

    def _get_time(self):
        return self._get_timestamp()

    @property
    def name(self):
        return self._name

    def close(self):
        pass

    @property
    def level(self):
        return self._level

    @level.setter
    def level(self, level):
        self._level = level

    def is_at_least(self, level):
        return level >= self._level

    def debug(self, message):
        if self.is_at_least(Level.DEBUG):
            timestamp = self._get_time()
            print(Fore.BLUE + "{} : ".format(timestamp)
                    + Style.DIM + Fore.RESET
                    + self._name_format.format(self._name)
                    + Logger.__color_debug + "{} : ".format(self.__DEBUG_TOKEN)
                    + Fore.CYAN + "{}".format(message) + Logger.__color_reset)

    def info(self, message):
        if self.is_at_least(Level.INFO):
            timestamp = self._get_time()
            print(Fore.BLUE + "{} : ".format(timestamp)
                    + Style.DIM + Fore.RESET
                    + self._name_format.format(self._name)
                    + Logger.__color_info + "{} : ".format(self.__INFO_TOKEN)
                    + Fore.CYAN + "{}".format(message) + Logger.__color_reset)

    def warning(self, message):
        if self.is_at_least(Level.WARN):
            timestamp = self._get_time()
            print(Fore.BLUE + "{} : ".format(timestamp)
                    + Style.DIM + Fore.RESET
                    + self._name_format.format(self._name)
                    + Logger.__color_warning + "{} : ".format(self.__WARN_TOKEN)
                    + Fore.CYAN + "{}".format(message) + Logger.__color_reset)

    def error(self, message):
        if self.is_at_least(Level.ERROR):
            timestamp = self._get_time()
            print(Fore.BLUE + "{} : ".format(timestamp)
                    + Style.DIM + Fore.RESET
                    + self._name_format.format(self._name)
                    + Logger.__color_error + "{} : ".format(self.__ERROR_TOKEN)
                    + Fore.CYAN + "{}".format(message) + Logger.__color_reset)

#EOF
