#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-01-27
# modified: 2026-02-17

import sys
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
    def __init__(self, i2c_id=1, level=Level.INFO):
        self._log = Logger('config', level=level)
        self._i2c_id = i2c_id
        self._default_i2c_address = 0x29
        self._i2c = None
        self._i2c_baud_rate = 400_000 # default 100,000
        self._xshut_pins = {}
        self._setup_i2c()
        self._setup_pins()
        self._i2c_scanner = I2CScanner(i2c_id=self._i2c_id)
        self._log.info('ready.')

    def configure(self):
        self._shutdown_all_sensors()
        self._configure_sensor_addresses()
        self._log.info('all sensor addresses configured.')

    def reset(self):
        from device import N0
        device = N0
        self._log.info("reset: temporarily shutting down sensor {} at XSHUT pin {}…".format(device.label, device.xshut))
        self._set_xshut(device.index, False)
        time.sleep_ms(250)
        self._set_xshut(device.index, True)
        self._log.info('radiozoa reset.\n')

    def _setup_i2c(self):
        self._i2c = I2C(self._i2c_id, freq=self._i2c_baud_rate)
        self._log.info('I2C{} open.'.format(self._i2c_id))

    def _setup_pins(self):
        '''
        Configure all XSHUT pins as outputs based on Device pseudo-enum.
        We set up the pins even a the Device configuration indicates no
        hardware sensor is available for a given slot.
        '''
        for device in Device.all():
            if device.impl is not None:
                pin = Pin(device.xshut, Pin.OUT) # on pyb, OUT_PP
                self._xshut_pins[device.index] = pin
                self._log.info("configured XSHUT pin {} for sensor {} on 0x{:02X} as output.".format(device.xshut, device.label, device.i2c_address))

    def close(self):
        '''
        Cleanup if necessary.
        '''
        self._log.info('closing radiozoa config.')

    def _shutdown_all_sensors(self):
        '''
        Shuts down all sensors by setting their XSHUT pins LOW.
        '''
        for device in Device.all():
            if device.impl is not None:
                self._log.info("shutting down sensor {} at XSHUT pin {}…".format(device.label, device.xshut))
                self._set_xshut(device.index, False)
                time.sleep_ms(50)
        self._log.info('all sensors shut down.\n')

    def _configure_sensor_addresses(self):
        '''
        Sequentially brings up each sensor, sets its I2C address, leaving it enabled.
        '''
        from vl53l0x import VL53L0X
        from vl53l1x import VL53L1X

        _device_delay_ms = 250
        _scan_delay_ms   = 750
        
        for device in Device.all():
            if device and device.impl is not None:
                self._log.info("configuring sensor {} at XSHUT pin {}…".format(device.label, device.xshut))
                self._set_xshut(device.index, True)
                found = False
                for i in range(5):
                    time.sleep_ms(_scan_delay_ms)
                    self._i2c_scanner.scan()
                    found = self._i2c_scanner.has_hex_address(0x29)
                    if found:
                        self._log.info(Style.DIM + "[{}] sensor appeared at 0x29.".format(i))
                        break
                    else:
                        self._log.info(Style.DIM + "[{}] waiting for sensor…".format(i))
                if not found:
                    self._log.warning("sensor {} did not appear at 0x29.".format(device.label))
                    continue
                try:
                    # create temporary sensor instance at default address
                    if device.impl == 'VL53L0X':
                        temp_sensor = VL53L0X(self._i2c, address=0x29)
                    elif device.impl == 'VL53L1X':
                        temp_sensor = VL53L1X(self._i2c, address=0x29)
                    else:
                        self._log.warning("unknown sensor type {} for device {}".format(device.impl, device.label))
                        continue
                    
                    # change address using sensor's method
                    self._set_i2c_address(device, device.i2c_address)
#                   temp_sensor.set_i2c_address(device.i2c_address)
                    self._log.info("set address for sensor {} to 0x{:02X}".format(device.label, device.i2c_address))
                    
                except Exception as e:
                    self._log.error("{} raised setting address for sensor {}: {}".format(type(e), device.label, e))
                    sys.print_exception(e)
                time.sleep_ms(_device_delay_ms)

    def _set_xshut(self, device_index, value):
        '''
        Set the XSHUT pin state for a given device.
        '''
        pin = self._xshut_pins.get(device_index)
        if pin:
            if value:
                pin.on() # pin.high()
#               self._log.info("set device {} pin HIGH.".format(device_index))
            else:
                pin.off() # pin.low()
#               self._log.info("set device {} pin LOW.".format(device_index))
        else:
            raise RuntimeError('pin not available for device {}.'.format(device_index))

    def _set_i2c_address(self, device, new_addr):
        '''
        Change I2C address for VL53L0X or VL53L1X sensor.
        Assumes sensor is available at 0x29.
        
        @param device: Device instance with impl property
        @param new_addr: new I2C address
        '''
        current_addr = 0x29
        if device.impl == 'VL53L1X':
            # VL53L1X: write to register 0x0001
            self._i2c.writeto(current_addr, bytearray([0x00, 0x01, new_addr]))
            time.sleep_ms(50)
        elif device.impl == 'VL53L0X':
            # VL53L0X: write to register 0x8A
            self._i2c.writeto_mem(current_addr, 0x8A, bytes([new_addr]))
            time.sleep_ms(50)
        elif device.impl is None:
            self._log.info(Fore.WHITE + "no device at {}.".format(device.label))
        else:
            raise ValueError("unknown sensor type: {}".format(device.impl))

#EOF
