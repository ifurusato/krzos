#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   altheim
# created:  2020-02-14
# modified: 2025-09-09
#
#  Scans the I²C bus, returning a list of devices. If i2cdetect is available
#  it is used, otherwise a less reliable Python-native approach is used (a
#  known error is that this won't find multiple devices occupying the same
#  address).
#
# DeviceNotFound class at bottom.

import re
import subprocess
import errno
import datetime as dt
from colorama import init, Fore, Style
init()

# import smbus
import pkg_resources
SMBUS='smbus2'
for dist in pkg_resources.working_set:
    if dist.project_name == 'smbus':
        break
    if dist.project_name == 'smbus2':
        SMBUS='smbus2'
        break
if SMBUS == 'smbus':
    import smbus
elif SMBUS == 'smbus2':
    import smbus2 as smbus

from core.logger import Level, Logger
from core.config_loader import ConfigLoader

class I2CScanner:
    NAME = 'i2c-scanner'
    '''
    Scans the I²C bus, returning a list of devices.

    If the I2C bus is not supplied it is created.

    Args:

        config:          the application configuration.
        i2c_bus_number:  the SMBus instance (default 1)
        i2c_bus:         the optional SMBus instance
        level:           the logging level
    '''
    def __init__(self, config=None, i2c_bus_number=1, i2c_bus=None, level=Level.INFO):
        super().__init__()
        if not isinstance(level, Level):
            raise ValueError('expected log level as a Level enum.')
        self._log = Logger(I2CScanner.NAME, level=level)
        self._config = config
        self._i2c_bus_number = i2c_bus_number
        if i2c_bus is None:
            self._i2c_bus = smbus.SMBus()
            self._i2c_bus.open(bus=self._i2c_bus_number)
        else:
            self._i2c_bus = i2c_bus
        self._int_list = []
        self._hex_list = []
        self._verbose = False
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def get_hex_addresses(self, force_scan=False):
        '''
        Returns a hexadecimal version of the list.

        Args:

          force_scan:  if True, forces a new scan regardless of the current map
        '''
        if force_scan or len(self._int_list) == 0:
            self._scan_addresses()
        return self._hex_list

    def get_int_addresses(self, force_scan=False):
        '''
        Returns an integer version of the list.

        Args:

          force_scan:  if True, forces a new scan regardless of the current map
        '''
        if force_scan or len(self._int_list) == 0:
            self._scan_addresses()
        return self._int_list

    def has_address(self, addresses, force_scan=False):
        '''
        Performs the address scan (if necessary) and returns true if a device
        is available at any of the specified int addresses, the argument a list
        of strings.

        Args:

          force_scan:  if True, forces a new scan regardless of the current map
        '''
        if force_scan or len(self._int_list) == 0:
            self._scan_addresses()
        for address in addresses:
            if address in self._int_list:
                return True
        return False

    def has_hex_address(self, addresses, force_scan=False):
        '''
        Performs the address scan (if necessary) and returns true if a device
        is available at any of the specified hexadecimal addresses, the argument
        a list of strings.

        Args:

          force_scan:  if True, forces a new scan regardless of the current map
        '''
        if force_scan or len(self._int_list) == 0:
            self._scan_addresses()
        for address in addresses:
            if address in self._hex_list:
                return True
        return False

    def normalise(self, address):
        '''
        Uppercases the numerical part of the hex string.
        '''
        return '0x{}'.format(address[2:].upper())

    def _scan_addresses(self, timeout=5):
        '''
        Executes i2cdetect as a subprocess and parses its output to return a
        list of found I2C addresses.

        Args:
            timeout (int):   the number of seconds to wait before timing out
        '''
        self._log.info('scanning I²C address bus {} using i2cdetect…'.format(self._i2c_bus_number))
        try:
            output = subprocess.check_output(
                    ["i2cdetect", "-y", str(self._i2c_bus_number)],
                    universal_newlines=True,
                    stderr=subprocess.PIPE,
                    timeout=timeout)
            if self._verbose:
                self._log.info(Fore.BLUE + "i2cdetect output:\n\n{}".format(output))
        except FileNotFoundError:
            self._log.warning("i2cdetect command not found. Please install i2c-tools.")
            self._py_scan_addresses()
            return
        except subprocess.TimeoutExpired:
            self._log.error("i2cdetect command timed out after {} seconds; trying python scan…".format(timeout))
            self._py_scan_addresses()
            return
        except subprocess.CalledProcessError as e:
            self._log.error("error executing i2cdetect: {}".format(e.stderr.strip()))
            self._py_scan_addresses()
            return
        lines = output.strip().split("\n")
        # Skip the first line as it contains column headers.
        for line in lines[1:]:
            parts = line.split()
            # The first part is the row identifier (e.g., '00:'). Skip it.
            # The remaining parts are the addresses or '--'.
            for addr_str in parts[1:]:
                if addr_str != "--":
                    _str_value = "0x{}".format(addr_str.upper())
                    self._log.debug("adding address: '{}'".format(_str_value))
                    self._hex_list.append(_str_value)
                    self._int_list.append(int(_str_value, 16))
        self._log.info('completed scan.')

    def _py_scan_addresses(self, timeout=dt.timedelta(seconds=5)):
        '''
        A native Python method that scans the I2C bus, populating the int
        and hex lists.

        Args:
            timeout (int):   the number of seconds to wait before timing out
        '''
        self._log.warning('scanning I²C address bus using python method…')
        device_count = 0
        start_time = dt.datetime.now()
        try:
            for address in range(3, 128):
                if dt.datetime.now() - start_time > timeout:
                    self._log.error('I2C scan timed out after {} seconds.'.format(timeout.total_seconds()))
                    break
                try:
                    self._i2c_bus.write_byte(address, 0)
                    _hex_address = hex(address)
                    _str_value = '0x{:02X}'.format(address)
                    self._log.info("found I²C device at 0x{:02X} (hex: '{}'; str: '{}')".format(address, _hex_address, _str_value))
                    self._int_list.append(address)
                    self._hex_list.append(_str_value)
                    device_count = device_count + 1
                except IOError as e:
                    if e.errno != errno.EREMOTEIO:
                        self._log.debug('{0} on address {1}'.format(e, hex(address)))
                except Exception as e: # exception if read_byte fails
                    self._log.error('{0} error on address {1}'.format(e, hex(address)))
            self._log.info('scanning complete.')
        except ImportError:
            self._log.warning('import error, unable to initialise: this script requires smbus2. Scan will return an empty result.')
        except Exception as e:
            self._log.warning('{} while initialising I²C bus: scan will return an empty result.'.format(e))
        if device_count == 1:
            self._log.info("found one I²C device.".format(device_count))
        elif device_count > 1:
            self._log.info("found {:d} I²C devices.".format(device_count))
        else:
            self._log.info("found no devices (no devices are available, or no smbus is available).")

    def print_device_list(self):
        self._addrDict = dict(list(map(lambda x, y:(x,y), self.get_int_addresses(), self.get_hex_addresses())))
        for _address in self.get_int_addresses():
            _device_name = self.get_device_for_address(_address)
            self._log.info('  I²C address 0x{:02X}: '.format(_address) + Fore.YELLOW + '{}'.format(_device_name))

    def get_device_for_address(self, address):
        '''
        Returns the lookup device name from the device registry found in
        the YAML configuration.
        '''
        _device = self._config['devices'].get(address)
        return 'Unknown' if _device is None else _device

class DeviceNotFound(Exception):
    '''
    Thrown when an expected device cannot be found.
    '''
    pass

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def main():

    level = Level.INFO
    log = Logger('main', level)
    i2c_bus_number = 0
    log.info('scanning for I²C devices on bus {}…'.format(i2c_bus_number))
    # read YAML configuration
    _config = ConfigLoader(Level.INFO).configure()
    scanner = I2CScanner(config=_config, i2c_bus_number=i2c_bus_number, level=Level.INFO)

    _addresses = scanner.get_int_addresses()
    log.info('available I²C device(s):')
    if len(_addresses) == 0:
        log.warning('no devices found.')
        return
    else:
        for n in range(len(_addresses)):
            address = _addresses[n]
            log.info('device: {0} ({1})'.format(address, hex(address)))

if __name__== "__main__":
    main()

#EOF
