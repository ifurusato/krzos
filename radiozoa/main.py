#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-02-07

import sys
import time
import asyncio
from colorama import Fore, Style

from colors import*
from controller import Controller
from i2c_slave import I2CSlave
from pixel import Pixel

# configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈

RELOAD_MODULES  = True
I2C_ID          = 0
I2C_ADDRESS     = 0x47

IS_TINYS3 = True
if IS_TINYS3:
    import tinys3

    BOARD_NAME   = 'UM TinyS3'
    SCL_PIN      = 7
    SDA_PIN      = 6
    NEOPIXEL_PIN = tinys3.RGB_DATA
    COLOR_ORDER  = 'GRB'
else:
    BOARD_NAME   = 'WaveShare ESP32-S3 Tiny'
    SCL_PIN      = 1
    SDA_PIN      = 2
    NEOPIXEL_PIN = 21
    COLOR_ORDER  = 'RGB'

print('configuring for {}…'.format(BOARD_NAME))

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

if RELOAD_MODULES:
    import gc

    # force module reload
    for mod in ['main', 'device', 'i2c_slave', 'controller']:
        if mod in sys.modules:
            del sys.modules[mod]
    gc.collect()

def create_pixel():
    '''
    Initialises NeoPixel support. If not using a UM TinyS3, set IS_TINYS3 to False
    and modify this method accordingly.
    '''
    _pixel = Pixel(pin=NEOPIXEL_PIN, pixel_count=1, color_order=COLOR_ORDER)
    if IS_TINYS3:
        tinys3.set_pixel_power(1)
    print('NeoPixel configured on pin {}'.format(NEOPIXEL_PIN))
    _pixel.set_color(0, (0, 0, 0))
    return _pixel

async def i2c_loop(controller, slave):
    last_time = time.ticks_ms()
    while True:
        current_time = time.ticks_ms()
        delta_ms = time.ticks_diff(current_time, last_time)
        last_time = current_time
        controller.tick(delta_ms)
        slave.check_and_process()
        await asyncio.sleep_ms(1)

def start():
    try:
        pixel = create_pixel()
        controller = Controller(pixel)
        slave = I2CSlave(i2c_id=I2C_ID, scl=SCL_PIN, sda=SDA_PIN, i2c_address=I2C_ADDRESS)
        slave.add_callback(controller.process)
        controller.set_slave(slave)
        slave.enable()
        last_time = time.ticks_ms()
        # run event loop
        asyncio.run(i2c_loop(controller, slave))
    except Exception as e:
        print(Fore.RED + '{} raised starting in main: {}'.format(type(e), e) + Style.RESET_ALL)
        sys.print_exception(e)
    except KeyboardInterrupt:
        print('\nCtrl-C caught; exiting…')

start()

#EOF
