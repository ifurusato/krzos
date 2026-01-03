#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-01-03

from core.logger import Logger, Level
from hardware.i2c_master import I2CMaster

class Stm32Controller(I2CMaster):
    NAME = 'stm32-ctrl'
    I2C_BUS_ID  = 1
    I2C_ADDRESS = 0x45
    '''
    Extends I2CMaster to control an STM32F405.
    ''' 
    def __init__(self, config=None, i2c_address=None, timeset=True, level=Level.INFO):
        if config: 
            _cfg = config.get('kros').get('hardware').get('stm32-controller')
            _i2c_bus_id  = _cfg.get('i2c_bus_id')
            _i2c_address = _cfg.get('i2c_address')
        else:
            # use defaults
            _i2c_bus_id  = Stm32Controller.I2C_BUS_ID
            _i2c_address = Stm32Controller.I2C_ADDRESS if i2c_address is None else i2c_address
        I2CMaster.__init__(self, log_or_name=Stm32Controller.NAME, i2c_bus_id=_i2c_bus_id, i2c_address=_i2c_address, timeset=timeset, level=level)
        # ready

#EOF
