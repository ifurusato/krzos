#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-01-29
#
# I2C1:  SCL=PB6   SDA=PB7
# I2C2:  SCL=PB10  SDA=PB11

import sys
import time
import stm
from pyb import Pin, Timer

from colors import*
from controller import Controller
from blinker import Blinker

# force module reload
for mod in ['main', 'i2c_slave', 'controller']:
    if mod in sys.modules:
        del sys.modules[mod]

def start():

    USE_BLINKER   = False
    USE_TIMER5    = False
    USE_I2C_SLAVE = False
    AUTOSTART_RADIOZOA = True

    blinker       = None
    timer5        = None
    slave         = None

    try:

        controller = Controller()

        if USE_BLINKER:
            blinker = Blinker(50, 1950)

        if AUTOSTART_RADIOZOA:
            controller.process('radiozoa start')

        if USE_TIMER5:
            # set up a 2Hz timer to call the controller's step()
            timer5 = Timer(5)
            timer5.init(freq=2,
                        callback=controller.step,
                        hard=False)

        if USE_I2C_SLAVE:
           #from i2c_slave import I2CSlave    # IRQ based
            from i2c_slave_mem import I2CSlave # memory-based

            # set up I2C slave
            slave = I2CSlave(i2c_id=2, i2c_address=0x45)
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

    except KeyboardInterrupt:
        print('\nCtrl-C caught; exiting…')

start()

#EOF
