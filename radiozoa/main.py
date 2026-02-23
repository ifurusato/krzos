#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-02-11

import sys
import time
import asyncio

from i2c_slave import I2CSlave

# configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈

RELOAD_MODULES = True
BOARD = 'TINYS3'  # 'TINYS3' | 'TINYFX' | 'RPI_PICO' | 'STM32F405' | 'ESP32_TINY'

BOARD_CONFIGS = {
    'TINYS3': {
        'name': 'UM TinyS3',
        'i2c_id': 0,
        'i2c_address': 0x47,
        'scl_pin': 7,
        'sda_pin': 6,
#       'controller_class': 'RingController',
        'controller_class': 'RadiozoaController',
        'family': 'TINYS3', # or more generally, ESP32
        'pixel_pin': None,  # set by tinys3.RGB_DATA
        'pixel_count': 1,
        'ring_pin': 44,
        'ring_count': 24,
        'color_order': 'GRB',
    },
    'TINYFX': {
        'name': 'Pimoroni Tiny FX',
        'i2c_id': 0,
        'i2c_address': 0x45,
        'scl_pin': 17,
        'sda_pin': 16,
        'controller_class': 'TinyFxController',
        'family': 'RP2',
        'pixel_pin': None,
        'color_order': None,
    },
    'RPI_PICO': {
        'name': 'Raspberry Pi Pico',
        'i2c_id':  0,
        'i2c_address': 0x47,
        'scl_pin': 3,
        'sda_pin': 2,
        'controller_class': 'PicoController',
        'family': 'RP2',
        'color_order': None,
    },
    'STM32F405': {
        'name': 'WeAct STM32F405',
        'i2c_id': 2,
        'i2c_address': 0x47,
        'scl_pin': None,
        'sda_pin': None,
        'controller_class': 'RingController',
        'family': 'STM32',
        'pixel_pin': 'B14',
        'pixel_count': 24,
        'color_order': 'GRB',
    },
    'ESP32_TINY': {
        'name': 'WaveShare ESP32-S3 Tiny',
        'i2c_id':  0,
        'i2c_address': 0x47,
        'scl_pin': 1,
        'sda_pin': 2,
        'controller_class': 'Controller',
        'family': 'ESP32',
        'pixel_pin': 21,
        'color_order': 'RGB',
    },
}

config = BOARD_CONFIGS[BOARD]
print('configuring for {}…'.format(config['name']))

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

if RELOAD_MODULES:
    import gc
    for mod in ['main', 'i2c_slave', 'controller']:
        if mod in sys.modules:
            del sys.modules[mod]
    gc.collect()

def create_controller(config):
    """
    Dynamically import and instantiate the controller class based on config.
    """
    class_name = config['controller_class']
    module_name = class_name.lower()
    module = __import__(module_name)
    cls = getattr(module, class_name)
    return cls(config)

async def i2c_loop(controller, slave):
    global enabled
    _last_time = time.ticks_ms()
    while enabled:
        _current_time = time.ticks_ms()
        controller.tick(time.ticks_diff(_current_time, _last_time))
        slave.check_and_process()
        _last_time = _current_time
        await asyncio.sleep_ms(1)

def start():
    global enabled
    enabled = True
    slave = None
    try:
        controller = create_controller(config)
        slave = I2CSlave(
            i2c_id=config['i2c_id'],
            scl=config['scl_pin'],
            sda=config['sda_pin'],
            i2c_address=config['i2c_address']
        )
        slave.add_callback(controller.process)
        controller.set_slave(slave)
        slave.enable()
        asyncio.run(i2c_loop(controller, slave))

    except KeyboardInterrupt:
        print('\nCtrl-C caught; exiting…')
    except Exception as e:
        print('ERROR: {} raised in start: {}'.format(type(e), e))
        sys.print_exception(e)
    finally:
        enabled = False
        print('complete.')

start()

#EOF
