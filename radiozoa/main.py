#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-02-26

import sys
import time
import asyncio
from machine import Pin, Timer
from colorama import Fore, Style

from pixel import Pixel
from i2c_slave import I2CSlave
from colors import *

# configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈

USE_EXT_CLOCK  = True
EXT_CLOCK_PIN  = 21
RELOAD_MODULES = False
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
        'color_order': 'GRB',
        'strip_pin': 43,
        'strip_count': 8,
        'strip_color_order': 'GRB',
        'ring_pin': 44,
        'ring_count': 24,
        'ring_color_order': 'GRB',
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

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

config = BOARD_CONFIGS[BOARD]
print('configuring for {}…'.format(config['name']))

enabled = True
slave   = None
ext_clock_timer = None

if RELOAD_MODULES:
    import gc
    for mod in ['main', 'i2c_slave', 'controller']:
        if mod in sys.modules:
            del sys.modules[mod]
    gc.collect()

def _create_strip(config):
    _strip_brightness = 0.1  # default: 0.33
    _strip = Pixel(
        pin=config['strip_pin'],
        pixel_count=config['strip_count'],
        color_order=config['strip_color_order'],
        brightness=_strip_brightness)
    _strip.set_color(0, COLOR_BLACK)
    return _strip

# do this quick to shut down pixels ASAP
__strip = _create_strip(config)

def _create_pixel(config):
    _family = config['family']
    if _family == 'TINYS3':
        import tinys3
        
        _pixel_pin = tinys3.RGB_DATA
        tinys3.set_pixel_power(1)
    else:
        _pixel_pin = config['pixel_pin']
    _pixel = Pixel(
        pin=_pixel_pin,
        pixel_count=1,
        color_order=config['color_order'])
    _pixel.set_color(0, COLOR_BLACK)
    return _pixel

def create_controller(config):
    '''
    Dynamically import and instantiate the controller class based on config.
    '''
    _pixel = _create_pixel(config)
    class_name = config['controller_class']
    module_name = class_name.lower()
    module = __import__(module_name)
    cls = getattr(module, class_name)
    return cls(config, _pixel, __strip)

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
    controller = create_controller(config)

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

    # create external clock
    pin = Pin(EXT_CLOCK_PIN, Pin.OUT)
    ext_clock_timer = Timer(3)
    ext_clock_timer.init(
        freq=40,
        mode=Timer.PERIODIC,
        callback=lambda t: pin.toggle()
    )

    # start async control loop
    asyncio.run(i2c_loop(controller, slave))

except KeyboardInterrupt:
    print('\nCtrl-C caught; exiting…')
except Exception as e:
    print('ERROR: {} raised in start: {}'.format(type(e), e))
    sys.print_exception(e)
finally:
    enabled = False
    if ext_clock_timer:
        ext_clock_timer.deinit()
    print('main complete.')

#EOF
