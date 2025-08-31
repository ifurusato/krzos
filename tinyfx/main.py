#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-14
# modified: 2025-05-27
#
# Starts an I2C slave with a Controller handler for incoming payloads.

import sys
import os
import gc
import _thread
import utime
import machine
from machine import Pin, Timer

import itertools
from colorama import Fore, Style

from tiny_fx import TinyFX
from picofx import MonoPlayer
from picofx.mono import StaticFX
from triofx import TrioFX
from rgb_blink import RgbBlinkFX
from i2c_settable_blink import I2CSettableBlinkFX
from pir import PassiveInfrared
from sound_dictionary import _sounds

from core.logger import Level, Logger
from stringbuilder import StringBuilder
from colors import*
from colorama import Fore, Style
from i2c_slave import I2CSlave
from tinyfx_controller import TinyFxController
from payload import Payload
from response import*

# constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

I2C_BUS_ID    = 0
SDA_PIN       = 16
SCL_PIN       = 17
I2C_ADDRESS   = 0x45
VERBOSE       = False
ERROR_LIMIT   = 10       # max errors before exiting main loop
DISPLAY_TYPE  = 'tinyfx' # 'neopixel' | 'ws2812' | 'pico'
START_COUNT   = 3

# keys of PIR triggered sound
#   arming-tone, beep, beep-hi, blip, boink, buzz, chatter, chirp, chirp-4,
#   chirp-7, cricket, dit_a, dit_b, dit_c, dwerp, earpit, glince, glitch,
#   gwolp, honk, hzah, ippurt, itiz, izit, pew-pew-pew, pizzle, silence,
#   skid-fzzt, sonic-bat, telemetry, tick, tika-tika, tsk-tsk-tsk, tweak,
#   twiddle-pop, twit, wow, zzt
PIR_SOUND     = 'cricket'

# init ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('main', Level.INFO)
_log.info(Fore.WHITE + "initialising…")

tinyfx = TinyFX(wav_root='/sounds')       # create a new TinyFX object to interact with the board
rgbled = tinyfx.rgb                       # get internal RGBLED
pir_sensor = PassiveInfrared()

# functions ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def get_display(name="neopixel"):
    if name == "neopixel":
        from neopixel_display import NeopixelDisplay
        return NeopixelDisplay()
    elif name == "ws2812":
        from ws2812_display import WS2812Display
        return WS2812Display()
    elif name == "pico":
        from pico_display import PicoDisplay
        return PicoDisplay()
    elif name == "tinyfx":
        from tinyfx_display import TinyFxDisplay
        return TinyFxDisplay(rgbled)
    else:
        raise ValueError("unrecognised display type: {}".format(DISPLAY_TYPE))

def show_color(color):
    '''
    Display the color on the NeoPixel.
    '''
#   _log.info(Style.DIM + "show color: {}".format(color.description))
    display.show_color(color)

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

# indicate startup, waiting 5 seconds so it can be interrupted…

display = get_display(DISPLAY_TYPE)

for i in range(START_COUNT):
    show_color(COLOR_CYAN)
    utime.sleep_ms(50)
    show_color(COLOR_BLACK)
    _log.info(Style.DIM + '[{}/{}] starting…'.format(i, START_COUNT))
    utime.sleep_ms(950)
utime.sleep_ms(50)

i2c_slave = None
try:
    _name = 'tinyfx'
    _log.info('start I2C slave…')
    _controller = TinyFxController(tinyfx, display)
    i2c_slave = I2CSlave(name=_name, i2c_bus_id=I2C_BUS_ID, sda=SDA_PIN, scl=SCL_PIN, i2c_address=I2C_ADDRESS, display=display, controller=_controller)
    _controller.startTimer()
    i2c_slave.enable()
    _log.warning('I2C slave started; it should have blocked.')
except KeyboardInterrupt:
    _log.info('Ctrl-C caught; exiting…')
except Exception as e:
    _log.error('{} raised in main loop: {}'.format(type(e), e))
    sys.print_exception(e)
finally:
    if i2c_slave:
        i2c_slave.disable()

# this only happens following an 'exit', which also disables the I2C slave
_log.info('exit: loop complete.')
show_color(COLOR_DARK_RED)
if tinyfx:
    tinyfx.wav.play_wav('buzz.wav')
utime.sleep(1.0)
# so we reset...
machine.reset()

#EOF
