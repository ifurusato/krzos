#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-07-13
# modified: 2024-10-31
#

import sys, platform
import os, psutil
from pathlib import Path
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from hardware.ina260_sensor import Ina260
from ads1015 import ADS1015

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class System:
    '''
    A collection of system control/info/statistical methods.
    Args:

    config:     the application configuration
    kros:       optional instance of KROS
    level:      the log level
    '''
    def __init__(self, config=None, kros=None, level=Level.INFO):
        self._log = Logger('system', level)
        self._kros = kros
        _cfg = config['kros'].get('hardware').get('system')
        self._12v_battery_min  = _cfg.get('low_12v_battery_threshold')   # 11.5  e.g., in0/ref: 11.953v
        self._5v_reg_min       = _cfg.get('low_5v_regulator_threshold')  #  5.0  e.g., in2/ref:  5.028v
        self._3v3_reg_min      = _cfg.get('low_3v3_regulator_threshold') #  3.25 e.g., in1/ref:  3.302v
        self._system_current_max = _cfg.get('system_current_max')


        self._batt_12v_channel = _cfg.get('battery_12v_channel')
        self._reg_5v_channel   = _cfg.get('reg_5v_channel')
        self._reg_3v3_channel  = _cfg.get('reg_3v3_channel')
        self._log.info("testing power supplies…")
        self._ads1015 = ADS1015()
        chip_type = self._ads1015.detect_chip_type()
        self._log.info("found device:  " + Fore.GREEN + "{}".format(chip_type))
        self._ads1015.set_mode("single")
        self._ads1015.set_programmable_gain(2.048)
        if chip_type == "ADS1015":
            self._ads1015.set_sample_rate(1600)
        else:
            self._ads1015.set_sample_rate(860)
        self._reference = self._ads1015.get_reference_voltage()
        self._log.info("reference:     " + Fore.GREEN + "{:6.3f}V".format(self._reference))
        self._ina260 = Ina260(config, level=level)
        self._log.info('ready.')

    # ADS1015 ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def get_voltages(self):
        '''
        Return a tuple containing 12V battery voltage, the 5V and 3.3V regulators.
        '''
        return self.battery_12v, self.reg_5v, self.reg_3v3

    def get_voltage_thresholds(self):
        '''
        Return a tuple containing minimum thresholds of the 12V battery voltage, the 5V and 3.3V regulators.
        '''
        return self._12v_battery_min, self._5v_reg_min, self._3v3_reg_min

    def get_battery_12v(self):
        '''
        Return the 12V battery voltage.
        '''
        return self._ads1015.get_compensated_voltage(channel=self._batt_12v_channel, reference_voltage=self._reference)

    def get_reg_5v(self):
        '''
        Return the output of the 5V regulator.
        '''
        return self._ads1015.get_compensated_voltage(channel=self._reg_5v_channel, reference_voltage=self._reference)

    def get_reg_3v3(self):
        '''
        Return the output of the 3.3V regulator.
        '''
        return self._ads1015.get_compensated_voltage(channel=self._reg_3v3_channel, reference_voltage=self._reference)

    def get_battery_12v_min(self):
        '''
        Return the minimum threshold of the 12V battery voltage.
        '''
        return self._12v_battery_min

    def get_reg_5v_min(self):
        '''
        Return the minimum threshold of the 5V regulator.
        '''
        return self._5v_reg_min

    def get_reg_3v3_min(self):
        '''
        Return the minimum threshold of the 3V3 regulator.
        '''
        return self._3v3_reg_min

    # INA260 ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def get_system_voltage(self):
        '''
        Return the measured system voltage from the INA160.
        '''
        return self._ina260.voltage

    def get_system_current(self):
        '''
        Return the measured system current from the INA160.
        '''
        return self._ina260.current


    def get_system_power(self):
        '''
        Return the measured system power from the INA160.
        '''
        return self._ina260.power

    def get_system_current_max(self):
        '''
        Return the maximum system current (threshold).
        '''
        return self._system_current_max

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_nice(self):
        '''
        Set KROS as high priority process.
        '''
        self._log.info('setting process as high priority…')
        proc = psutil.Process(os.getpid())
        proc.nice(10)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def is_raspberry_pi(self):
        '''
        Returns True if the system is named 'Linux' and shows as either a 32
        or 64 bit architecture, indicating this is running on a Raspberry Pi.
        This is nowhere near foolproof.

          • Linux/armv71:   32 bit Raspberry Pi
          • Linux/aarch64:  64 bit Raspberry Pi (or NanoPi Fire3
          • Linux/x86_64:   Ubuntu on Intel
          • Darwin/x86_64:  Mac OS on Intel

        '''
        _arch = platform.machine()
        return  platform.system() == 'Linux' and ( _arch == 'armv71' or _arch == 'aarch64' ) # 32 and 64 bit architecture names

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_platform_info(self):
        '''
        Returns a list of information about the current environment, or None
        if this fails for any reason. The list includes:

            * Python version
            * system name
            * machine architecture
            * platform description
            * uname
            * OS version
            * mac_ver
        '''
        try:
            _info = ( sys.version.split('\n'),
                      platform.system(),
                      platform.machine(),
                      platform.platform(),
                      platform.uname(),
                      platform.version(),
                      platform.mac_ver() )
            return _info
        except Exception as e:
            self._log.error('error getting platform information:  {}'.format(e))
            return None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_battery_info(self):

        try:
            _battery_12v, _reg_5v, _reg_3v3 = self.voltages
            self._log.info("battery (in0): " + Fore.GREEN + "{:6.3f}V".format(_battery_12v))
            self._log.info("5V ref (in2):  " + Fore.GREEN + "{:6.3f}V".format(_reg_5v))
            self._log.info("3V3 reg (in1): " + Fore.GREEN + "{:6.3f}V".format(_reg_3v3))

            assert _battery_12v > self._12v_battery_min, 'measured in0 value less than threshold {}v'.format(self._12v_battery_min)
            assert _reg_5v      > self._5v_reg_min, 'measured in2 value less than threshold {}v'.format(self._5v_reg_min)
            assert _reg_3v3     > self._3v3_reg_min, 'measured in1 value less than threshold {}v'.format(self._3v3_reg_min)

            self._log.info(Fore.GREEN + "power supplies are functional.")

        except Exception as e:
            self._log.error('{} raised in power supply test: {}'.format(type(e), e))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def print_sys_info(self):
        if self._kros:
            self._log.info('kros:  state: ' + Fore.YELLOW + '{}  \t'.format(self._kros.state.name) \
                    + Fore.CYAN + 'enabled: ' + Fore.YELLOW + '{}'.format(self._kros.enabled))
        # disk space ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._log.info('root file system:')
        _rootfs = psutil.disk_usage('/')
        _div = float(1<<30)
        _div = 1024.0 ** 3
#       self._log.info('  total:  \t' + Fore.YELLOW + '{:>6.2f}GB'.format(_rootfs.total / _div))
#       self._log.info('  used:   \t' + Fore.YELLOW + '{:>6.2f}GB ({}%)'.format((_rootfs.used / _div), _rootfs.percent))
#       self._log.info('  free:   \t' + Fore.YELLOW + '{:>6.2f}GB'.format(_rootfs.free / _div))
        self._log.info('  total: ' + Fore.YELLOW + '{:>4.2f}GB'.format(_rootfs.total / _div)
                + Fore.CYAN + '; used: ' + Fore.YELLOW + '{:>4.2f}GB ({}%)'.format((_rootfs.used / _div), _rootfs.percent)
                + Fore.CYAN + '; free: ' + Fore.YELLOW + '{:>4.2f}GB'.format(_rootfs.free / _div))
        # memory ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _MB = 1000000
        _vm = psutil.virtual_memory()
        # svmem(total=n, available=n, percent=n, used=n, free=n, active=n, inactive=n, buffers=n, cached=n, shared=n)
        self._log.info('virtual memory:')
#       self._log.info('  total:    \t' + Fore.YELLOW + '{:>6.2f}MB'.format(_vm[0]/_MB))
#       self._log.info('  available:\t' + Fore.YELLOW + '{:>6.2f}MB'.format(_vm[1]/_MB))
#       self._log.info('  used:     \t' + Fore.YELLOW + '{:>6.2f}GB ({:4.1f}%)'.format(_vm[3]/_MB, _vm[2]))
#       self._log.info('  free:     \t' + Fore.YELLOW + '{:>6.2f}GB'.format( _vm[4]/_MB))
        self._log.info('  total: ' + Fore.YELLOW + '{:>4.2f}MB'.format(_vm[0]/_MB)
                + Fore.CYAN + '; available: ' + Fore.YELLOW + '{:>4.2f}MB'.format(_vm[1]/_MB)
                + Fore.CYAN + '; used: ' + Fore.YELLOW + '{:>4.2f}GB ({:4.1f}%)'.format(_vm[3]/_MB, _vm[2])
                + Fore.CYAN + '; free: ' + Fore.YELLOW + '{:>4.2f}GB'.format( _vm[4]/_MB))
        # sswap(total=n, used=n, free=n, percent=n, sin=n, sout=n)
        _sw = psutil.swap_memory()
        self._log.info('swap memory:')
#       self._log.info('  total:  \t' + Fore.YELLOW + '{:>6.2f}MB'.format(_sw[0]/_MB))
#       self._log.info('  used:   \t' + Fore.YELLOW + '{:>6.2f}MB ({:3.1f}%)'.format(_sw[1]/_MB, _sw[3]))
#       self._log.info('  free:   \t' + Fore.YELLOW + '{:>6.2f}MB'.format(_sw[2]/_MB))
        self._log.info('  total: ' + Fore.YELLOW + '{:>4.2f}MB'.format(_sw[0]/_MB)
                + Fore.CYAN + '; used: ' + Fore.YELLOW + '{:>4.2f}MB ({:3.1f}%)'.format(_sw[1]/_MB, _sw[3])
                + Fore.CYAN + '; free: ' + Fore.YELLOW + '{:>4.2f}MB'.format(_sw[2]/_MB))
        # CPU temperature ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        temperature = self.read_cpu_temperature()
        if temperature:
            self._log.info('cpu temperature:\t' + Fore.YELLOW + '{:5.2f}°C'.format(temperature))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def read_cpu_temperature(self):
        temp_file = Path('/sys/class/thermal/thermal_zone0/temp')
        if temp_file.is_file():
            with open(temp_file, 'r') as f:
                data = int(f.read())
                temperature = data / 1000
                return temperature
        else:
            return None

#EOF
