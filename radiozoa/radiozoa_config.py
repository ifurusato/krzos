#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-01-27
# modified: 2026-01-27

import time
from machine import Pin, I2C
from colorama import Fore, Style

from logger import Logger, Level
from i2c_scanner import I2CScanner
from device import Device

class RadiozoaConfig:
    '''
    Configures all VL53L0X sensors on the Radiozoa sensor board to their unique
    I2C addresses by toggling XSHUT pins and setting addresses as specified in
    the Device pseudo-enum.
    '''
    def __init__(self, i2c_bus=1, level=Level.INFO):
        self._log = Logger('radiozoa-conf', level=level)
        self._level = level
        self._i2c_bus_number = i2c_bus
        self._default_i2c_address = 0x29
        self._i2c = None
        self._xshut_pins = {}
        self._setup_i2c()
        self._setup_pins()
        self._i2c_scanner = I2CScanner(i2c_bus=self._i2c_bus_number)
        self._log.info('ready.')

    def configure(self):
        self._shutdown_all_sensors()
        self._configure_sensor_addresses()
        self._log.info('all sensor addresses configured.')

    def reset(self):
        from device import N0
        dev = N0
        self._log.info(Fore.YELLOW + "reset: temporarily shutting down sensor {} at XSHUT pin {}…".format(dev.label, dev.xshut))
        self._set_xshut(dev.index, False)
        time.sleep_ms(250)
        self._set_xshut(dev.index, True)
        self._log.info(Fore.YELLOW + 'radiozoa reset.\n')

    def _setup_i2c(self):
        self._i2c = I2C(self._i2c_bus_number)
        self._log.info('I2C{} open.'.format(self._i2c_bus_number))

    def _setup_pins(self):
        '''
        Configure all XSHUT pins as outputs based on Device pseudo-enum.
        '''
        for dev in Device.all():
            pin = Pin(dev.xshut, Pin.OUT) # on pyb, OUT_PP
            self._xshut_pins[dev.index] = pin
            self._log.info("configured XSHUT pin {} for sensor {} on 0x{:02X} as output.".format(dev.xshut, dev.label, dev.i2c_address))

    def close(self):
        '''
        Cleanup if necessary.
        '''
        self._log.info('closing radiozoa config.')

    def _shutdown_all_sensors(self):
        '''
        Shuts down all sensors by setting their XSHUT pins LOW.
        '''
        for dev in Device.all():
            self._log.info("shutting down sensor {} at XSHUT pin {}…".format(dev.label, dev.xshut))
            self._set_xshut(dev.index, False)
            time.sleep_ms(50)
        self._log.info('all sensors shut down.\n')

    def _configure_sensor_addresses(self):
        '''
        Sequentially brings up each sensor, sets its I2C address, leaving it enabled.
        '''
        needs_scan = True
        for dev in Device.all():
            self._log.info(Style.DIM + "configuring sensor {} at XSHUT pin {}…".format(dev.label, dev.xshut))
            self._set_xshut(dev.index, True)
            found = False
            for i in range(5):
                time.sleep_ms(750)
                self._i2c_scanner.scan()
                found = self._i2c_scanner.has_hex_address(0x29)
                if found:
                    self._log.info(Style.DIM + "[{}] sensor appeared at 0x29.".format(i))
                    break
                else:
                    self._log.info(Style.DIM + "[{}] waiting for sensor…".format(i))
            if not found:
                self._log.warning("sensor {} did not appear at 0x29.".format(dev.label))
                continue
            try:
                self._set_i2c_address(0x29, dev.i2c_address)
                self._log.info("set address for sensor {} to 0x{:02X}".format(dev.label, dev.i2c_address))
            except Exception as e:
                self._log.error("{} raised setting address for sensor {}: {}".format(type(e), dev.label, e))
            time.sleep_ms(50)

    def _set_xshut(self, device_index, value):
        '''
        Set the XSHUT pin state for a given device.
        '''
        pin = self._xshut_pins.get(device_index)
        if pin:
            if value:
                pin.on() # pin.high()
                self._log.debug(Style.DIM + "set device {} pin HIGH.".format(device_index))
            else:
                pin.off() # pin.low()
                self._log.debug(Style.DIM + "set device {} pin LOW.".format(device_index))
        else:
            raise RuntimeError('pin not available for device {}.'.format(device_index))

    def _set_i2c_address(self, current_addr, new_addr):
        '''
        Change VL53L0X I2C address from current_addr to new_addr.
        Assumes sensor is up at current_addr.
        '''
        reg = 0x8A
        self._i2c.writeto_mem(current_addr, reg, bytes([new_addr]))
#       self._i2c.mem_write(new_addr, current_addr, reg)
        time.sleep_ms(50)

#EOF
