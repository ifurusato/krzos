#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-09-07
# modified: 2025-10-11
#
# As a modification of the library by John Bryan Moore. This adds our own
# Logger, and doesn't manage smbus itself, instead passing the instance into
# the constructor, enabling better management of multiple instances.
#
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
# MIT License
#
# Copyright (c) 2017 John Bryan Moore
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
from ctypes import CDLL, CFUNCTYPE, POINTER, c_int, c_uint, pointer, c_ubyte, c_uint8, c_uint32
# import sysconfig # we use our own library

import site
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level

class Vl53l0xError(RuntimeError):
    pass

class Vl53l0xAccuracyMode:
    GOOD = 0        # 33 ms timing budget 1.2m range
    BETTER = 1      # 66 ms timing budget 1.2m range
    BEST = 2        # 200 ms 1.2m range
    LONG_RANGE = 3  # 33 ms timing budget 2m range
    HIGH_SPEED = 4  # 20 ms timing budget 1.2m range

class Vl53l0xDeviceMode:
    SINGLE_RANGING = 0
    CONTINUOUS_RANGING = 1
    SINGLE_HISTOGRAM = 2
    CONTINUOUS_TIMED_RANGING = 3
    SINGLE_ALS = 10
    GPIO_DRIVE = 20
    GPIO_OSC = 21

class Vl53l0xGpioAlarmType:
    OFF = 0
    THRESHOLD_CROSSED_LOW = 1
    THRESHOLD_CROSSED_HIGH = 2
    THRESHOLD_CROSSED_OUT = 3
    NEW_MEASUREMENT_READY = 4

class Vl53l0xInterruptPolarity:
    LOW = 0
    HIGH = 1

# Read/write function pointer types.
_I2C_READ_FUNC = CFUNCTYPE(c_int, c_ubyte, c_ubyte, POINTER(c_ubyte), c_ubyte)
_I2C_WRITE_FUNC = CFUNCTYPE(c_int, c_ubyte, c_ubyte, POINTER(c_ubyte), c_ubyte)

class VL53L0X:
    '''
    VL53L0X ToF.
    '''
    def __init__(self, i2c_bus=None, i2c_address=0x29, label='tof', tca9548a_num=255, tca9548a_addr=0):
        '''
        Initialize the VL53L0X ToF Sensor from ST.
        '''
        self._log = Logger('vl53l0x-0x{:02X}'.format(i2c_address), level=Level.INFO)
#       self._i2c_bus_number = i2c_bus_number
        self._i2c_address = i2c_address
        self._label = label
        self._log.debug('creating sensor at 0x{:02X}…'.format(i2c_address))
        self._tca9548a_num = tca9548a_num
        self._tca9548a_addr = tca9548a_addr
        self._i2c_bus     = i2c_bus
#       self._i2c_bus     = smbus.SMBus()
        self._dev         = None
        self._tof_library = None
        self._is_ranging  = False
        # Register Address
        self.ADDR_UNIT_ID_HIGH = 0x16 # Serial number high byte
        self.ADDR_UNIT_ID_LOW  = 0x17 # Serial number low byte
        self.ADDR_I2C_ID_HIGH  = 0x18 # Write serial number high byte for I2C address unlock
        self.ADDR_I2C_ID_LOW   = 0x19 # Write serial number low byte for I2C address unlock
        self.ADDR_I2C_SEC_ADDR = 0x8a # Write new I2C address after unlock
        self._get_tof_library()
        self._log.info(Fore.GREEN + '{} '.format(self._label) + Fore.CYAN + 'sensor ready at 0x{:02X}…'.format(self._i2c_address))

    @property
    def i2c_address(self):
        return self._i2c_address

    def _get_tof_library(self):
        # Load VL53L0X shared lib
        suffix = None #sysconfig.get_config_var('EXT_SUFFIX')
        if suffix is None:
            suffix = ".so"
        # Get the directory containing this VL53L0X.py file
        MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
        # Relative path to the binary (from this .py file)
        LOCAL_BIN_PATH = os.path.join(MODULE_DIR, 'bin')
        _POSSIBLE_LIBRARY_LOCATIONS = (
            [LOCAL_BIN_PATH] +
            site.getsitepackages() +
            [site.getusersitepackages()]
        )
        #_POSSIBLE_LIBRARY_LOCATIONS = ['../bin'] + site.getsitepackages() + [site.getusersitepackages()]
        for lib_location in _POSSIBLE_LIBRARY_LOCATIONS:
            try:
                _TOF_LIBRARY_PATH = lib_location + '/vl53l0x_python' + suffix
#               print(Fore.CYAN + '_TOF_LIBRARY_PATH: {}'.format(_TOF_LIBRARY_PATH) + Style.RESET_ALL)
                self._tof_library = CDLL(_TOF_LIBRARY_PATH)
                break
            except OSError:
                pass
        else:
            raise OSError('Could not find vl53l0x_python' + suffix)

    def open(self):
        self._log.debug('open sensor at 0x{:02X}'.format(self._i2c_address))
        if self._tof_library:
            self._configure_i2c_library_functions()
            self._dev = self._tof_library.initialise(self._i2c_address, self._tca9548a_num, self._tca9548a_addr)
            self._log.info("Device pointer after initialise for '{}' at 0x{:02X}: {}".format(self._label, self._i2c_address, self._dev))
            if not self._dev:
                self._log.error("VL53L0X initialisation failed for '{}' at 0x{:02X}: device pointer is NULL/zero.".format(self._label, self._i2c_address))
                raise Vl53l0xError("VL53L0X initialisation failed for '{}' at 0x{:02X}: device pointer is NULL/zero.".format(self._label, self._i2c_address))
            return True
        else:
            raise Exception('no ToF library available.')

    def close(self):
#       self._i2c_bus.close()
        self._dev = None

    def _configure_i2c_library_functions(self):
        # I2C bus read callback for low level library.
        def _i2c_read(address, reg, data_p, length):
            ret_val = 0
            result = []
            try:
                result = self._i2c_bus.read_i2c_block_data(address, reg, length)
            except IOError as e:
#               self._log.warning('{} raised processing I2C bus callback for VL53L0X: {}'.format(type(e), e))
                ret_val = -1
            except Exception as e:
                raise Vl53l0xError('{} raised processing I2C bus callback for VL53L0X: {}'.format(type(e), e))

            if ret_val == 0:
                for index in range(length):
                    data_p[index] = result[index]

            return ret_val
        # I2C bus write callback for low level library.
        def _i2c_write(address, reg, data_p, length):
            ret_val = 0
            data = []
            for index in range(length):
                data.append(data_p[index])
            try:
                self._i2c_bus.write_i2c_block_data(address, reg, data)
            except IOError:
                ret_val = -1
            return ret_val
        # pass i2c read/write function pointers to VL53L0X library.
        self._i2c_read_func = _I2C_READ_FUNC(_i2c_read)
        self._i2c_write_func = _I2C_WRITE_FUNC(_i2c_write)
        self._tof_library.VL53L0X_set_i2c(self._i2c_read_func, self._i2c_write_func)

    def start_ranging(self, mode=Vl53l0xAccuracyMode.GOOD):
        """Start VL53L0X ToF Sensor Ranging"""
        if not self._dev:
            self._log.error("Cannot start ranging: device pointer is NULL/zero for '{}' at 0x{:02X}.".format(self._label, self._i2c_address))
            raise Vl53l0xError("Cannot start ranging: device pointer is NULL/zero for '{}' at 0x{:02X}.".format(self._label, self._i2c_address))
        result = self._tof_library.startRanging(self._dev, mode)
        self._log.info("startRanging on '{}' at 0x{:02X} returned: {}".format(self._label, self._i2c_address, result))
        if hasattr(self._tof_library, 'getRangingStatus'):
            status = self._tof_library.getRangingStatus(self._dev)
            self._log.info("Ranging status for '{}' at 0x{:02X}: {}".format(self._label, self._i2c_address, status))
            if status != 0:
                self._log.error("Ranging failed for '{}' at 0x{:02X} with status {}".format(self._label, self._i2c_address, status))
                raise Vl53l0xError("Ranging failed for '{}' at 0x{:02X} with status {}".format(self._label, self._i2c_address, status))
        self._is_ranging = True

    def stop_ranging(self):
        """Stop VL53L0X ToF Sensor Ranging"""
        self._log.info('stop ranging sensor {} at 0x{:02X}…'.format(self._label, self._i2c_address))
        if self._dev:
            self._tof_library.stopRanging(self._dev)
        self._is_ranging = False

    def get_distance(self):
        """Get distance from VL53L0X ToF Sensor"""
        if not self._is_ranging:
            self._log.error("get_distance called, but sensor '{}' at 0x{:02X} is not ranging!".format(self._label, self._i2c_address))
            raise Vl53l0xError("get_distance called, but sensor '{}' at 0x{:02X} is not ranging!".format(self._label, self._i2c_address))
        distance = self._tof_library.getDistance(self._dev)
        self._log.info("getDistance for '{}' at 0x{:02X}: {}".format(self._label, self._i2c_address, distance))
        return distance

    def x_open(self):
        self._log.debug('open sensor at 0x{:02X}'.format(self._i2c_address))
        if self._tof_library:
#           self._i2c_bus.open(bus=self._i2c_bus_number)
            self._configure_i2c_library_functions()
            self._dev = self._tof_library.initialise(self._i2c_address, self._tca9548a_num, self._tca9548a_addr)
            return True # added to force wait for value
        else:
            raise Exception('no ToF library available.')

    def x_start_ranging(self, mode=Vl53l0xAccuracyMode.GOOD):
        """Start VL53L0X ToF Sensor Ranging"""
        self._tof_library.startRanging(self._dev, mode)

    def x_stop_ranging(self):
        """Stop VL53L0X ToF Sensor Ranging"""
        self._log.info('stop ranging sensor at 0x{:02X}…'.format(self._i2c_address))
        self._tof_library.stopRanging(self._dev)

    def x_get_distance(self):
        """Get distance from VL53L0X ToF Sensor"""
#       self._log.info('getting distance from sensor at 0x{:02X}'.format(self._i2c_address))
        return self._tof_library.getDistance(self._dev)

    # This function included to show how to access the ST library directly
    # from python instead of through the simplified interface
    def get_timing(self):
        budget = c_uint(0)
        budget_p = pointer(budget)
        status = self._tof_library.VL53L0X_GetMeasurementTimingBudgetMicroSeconds(self._dev, budget_p)
        if status == 0:
            return budget.value + 1000
        else:
            return 0

    def configure_gpio_interrupt(
            self, proximity_alarm_type=Vl53l0xGpioAlarmType.THRESHOLD_CROSSED_LOW,
            interrupt_polarity=Vl53l0xInterruptPolarity.HIGH, threshold_low_mm=250, threshold_high_mm=500):
        """
        Configures a GPIO interrupt from device, be sure to call "clear_interrupt" after interrupt is processed.
        """
        pin = c_uint8(0)  # 0 is only GPIO pin.
        device_mode = c_uint8(Vl53l0xDeviceMode.CONTINUOUS_RANGING)
        functionality = c_uint8(proximity_alarm_type)
        polarity = c_uint8(interrupt_polarity)
        status = self._tof_library.VL53L0X_SetGpioConfig(self._dev, pin, device_mode, functionality, polarity)
        if status != 0:
            raise Vl53l0xError('error setting VL53L0X GPIO config.')

        threshold_low = c_uint32(threshold_low_mm << 16)
        threshold_high = c_uint32(threshold_high_mm << 16)
        status = self._tof_library.VL53L0X_SetInterruptThresholds(self._dev, device_mode, threshold_low, threshold_high)
        if status != 0:
            raise Vl53l0xError('error setting VL53L0X thresholds.')

        # Ensure any pending interrupts are cleared.
        self.clear_interrupt()

    def clear_interrupt(self):
        mask = c_uint32(0)
        status = self._tof_library.VL53L0X_ClearInterruptMask(self._dev, mask)
        if status != 0:
            raise Vl53l0xError('error clearing VL53L0X interrupt.')

    def change_address(self, new_address):
        if self._dev is not None:
            raise Vl53l0xError('error changing VL53L0X address.')
        if new_address == None:
            raise ValueError('no new address argument provided.')
        elif new_address == self._i2c_address:
            self._log.warning('no address change: argument matches current address.')
            return
        try:
            self._log.debug('changing address to 0x{:02X}…'.format(new_address))
#           self._i2c_bus.open(bus=self._i2c_bus_number)
            # read value from 0x16,0x17
            high = self._i2c_bus.read_byte_data(self._i2c_address, self.ADDR_UNIT_ID_HIGH)
            low = self._i2c_bus.read_byte_data(self._i2c_address, self.ADDR_UNIT_ID_LOW)
            # write value to 0x18,0x19
            self._i2c_bus.write_byte_data(self._i2c_address, self.ADDR_I2C_ID_HIGH, high)
            self._i2c_bus.write_byte_data(self._i2c_address, self.ADDR_I2C_ID_LOW, low)
            # write new_address to 0x1a
            self._i2c_bus.write_byte_data(self._i2c_address, self.ADDR_I2C_SEC_ADDR, new_address)
            self._i2c_address = new_address
#           self._i2c_bus.close()
            self._log.info('address changed to 0x{:02X}'.format(new_address))
        except Exception as e:
            self._log.error('{} raised changing address: {}'.format(type(e), e))
            raise

