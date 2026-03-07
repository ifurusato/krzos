#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-03-08

import sys
import time
import asyncio
from machine import Pin, Timer
from colorama import Fore, Style

from i2c_slave import I2CSlave
from colors import *
from tinyfx_controller import TinyFxController

# configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈

RELOAD_MODULES = False
BOARD = 'TINYFX'

BOARD_CONFIGS = {
    'TINYFX': {
        'name': 'Pimoroni Tiny FX',
        'i2c_id': 0,
        'i2c_address': 0x45,
        'scl_pin': 17,
        'sda_pin': 16,
        'family': 'RP2'
    },
}

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

config = BOARD_CONFIGS[BOARD]
print('configuring for {}…'.format(config['name']))

enabled = True
slave   = None

if RELOAD_MODULES:
    import gc
    for mod in ['main', 'i2c_slave', 'controller']:
        if mod in sys.modules:
            del sys.modules[mod]
    gc.collect()

async def i2c_loop(controller, slave):
    global enabled
    _last_time = time.ticks_ms()
    while enabled:
        _current_time = time.ticks_ms()
        controller.tick(time.ticks_diff(_current_time, _last_time))
        slave.check_and_process()
        _last_time = _current_time
        await asyncio.sleep_ms(1)

try:

    # create controller
    controller = TinyFxController(config)

    # create I2C slave
    slave = I2CSlave(
        i2c_id=config['i2c_id'],
        scl=config['scl_pin'],
        sda=config['sda_pin'],
        i2c_address=config['i2c_address']
    )
    slave.add_callback(controller.process)
    controller.set_slave(slave)
    slave.enable()

    # start async control loop
    asyncio.run(i2c_loop(controller, slave))

except KeyboardInterrupt:
    print('\nCtrl-C caught; exiting…')
except Exception as e:
    print('ERROR: {} raised in start: {}'.format(type(e), e))
    sys.print_exception(e)
finally:
    enabled = False
    print('complete.')

#EOF
