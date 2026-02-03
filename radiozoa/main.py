#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-02-01
#
# I2C1:  SCL=PB6   SDA=PB7
# I2C2:  SCL=PB10  SDA=PB11

I2C_ID        = 0
I2C_ADDRESS   = 0x47
RELOAD        = True
USE_BLINKER   = False
USE_I2C_SLAVE = True
AUTOSTART_RADIOZOA = True

import sys
import time
import gc
from colorama import Fore, Style

from colors import*
from controller import Controller
from blinker import Blinker

if RELOAD:
    # force module reload
    for mod in ['main', 'device', 'i2c_slave', 'controller']:
        if mod in sys.modules:
            del sys.modules[mod]
    gc.collect()

def get_slave():
    try:
        print('🤢 a.')
#       from i2c_slave import I2CSlave    # IRQ based
        from i2c_slave_mem import I2CSlave # memory-based
        print('🤢 b. setting up I2C slave on I2C{} at address 0x{:02X}…'.format(I2C_ID, I2C_ADDRESS))
        # set up I2C slave
        slave = I2CSlave(i2c_id=I2C_ID, i2c_address=I2C_ADDRESS)
        print('🤢 c. ready.')
        return slave
    except Exception as e:
        print(Fore.RED + '{} raised creating I2C slave: {}'.format(type(e), e) + Style.RESET_ALL)
        sys.print_exception(e)
        raise

def start():

    blinker       = None
    timer5        = None
    slave         = None

    try:

        controller = Controller()

        if USE_BLINKER:
            blinker = Blinker(on_ms=50, off_ms=1950)

        if USE_I2C_SLAVE:
#           from i2c_slave import I2CSlave    # IRQ based
#           from i2c_slave_mem import I2CSlave # memory-based
#           slave = I2CSlave(i2c_id=I2C_ID, i2c_address=I2C_ADDRESS)

            # set up I2C slave
            slave = get_slave()
            slave.add_callback(controller.process)
            controller.set_slave(slave)
            slave.enable()
            last_time = time.ticks_ms()
            print('I2C slave active…')
            while True:
                current_time = time.ticks_ms()
                delta_ms = time.ticks_diff(current_time, last_time)
                last_time = current_time
                controller.tick(delta_ms)
                slave.check_and_process()
                time.sleep_ms(1)

        if AUTOSTART_RADIOZOA:
            controller.process('radiozoa start')

    except KeyboardInterrupt:
        print('\nCtrl-C caught; exiting…')

start()

#EOF
