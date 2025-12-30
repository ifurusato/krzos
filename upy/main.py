#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2025-12-28
#
# I2C1:  SCL=PB6   SDA=PB7
# I2C2:  SCL=PB10  SDA=PB11

import sys
import time
import stm
from pyb import Pin, Timer

from colors import*
from i2c_slave import I2CSlave
from controller import Controller
from pixel import Pixel
#from pixel_cycler import PixelCycler
#from blink_pattern import BlinkPattern
#from rainbow_cycler import RainbowCycler

# auto-clear: remove cached modules to force reload
for mod in ['main', 'i2c_slave', 'controller']:
    if mod in sys.modules:
        del sys.modules[mod]

def main():
    TIMER2_SOFT = False
    TIMER2_HARD = True
    TIMER5      = True
    I2C_SLAVE   = True
    slave  = None
    timer2 = None
    timer5 = None

    try:

        controller = Controller()

        count = 24
        ring = Pixel(pin='B14', pixel_count=count, brightness=0.1)
        strip = Pixel(pin='B12', pixel_count=8, brightness=0.1)

        if TIMER2_SOFT:
            clock_pin = Pin('A0', Pin.OUT_PP)
            # set up 20Hz timer2 on pin A0 (requires 2x frequency since toggle is half freq)
            timer2 = Timer(2)
            timer2.init(freq=40,
                        callback=lambda t: clock_pin.toggle(),
                        hard=False)

        if TIMER2_HARD:
            PIN_BIT = 0  # PA0
            PIN_MASK = 1 << PIN_BIT
            # configure PA0 as push-pull output (mode = 0b01, otyper = 0)
            stm.mem32[stm.GPIOA + stm.GPIO_MODER] &= ~(0b11 << (PIN_BIT*2))  # clear mode
            stm.mem32[stm.GPIOA + stm.GPIO_MODER] |=  (0b01 << (PIN_BIT*2))  # set output mode
            stm.mem32[stm.GPIOA + stm.GPIO_OTYPER] &= ~PIN_MASK             # push-pull
            # hard IRQ toggle
            def toggle_hard(timer):
                stm.mem32[stm.GPIOA + stm.GPIO_ODR] ^= PIN_MASK
            # timer at 40 Hz (20 Hz toggle)
            timer2 = Timer(2)
            timer2.init(freq=40,
                        callback=toggle_hard,
                        hard=True)
        if TIMER5:
            # set up a 2Hz timer to call the controller's step()
            timer5 = Timer(5)
            timer5.init(freq=2,
                        callback=controller.step, 
                        hard=False)
        
        if I2C_SLAVE:
            # set initial pixel
            strip.set_color(index=0, color=COLOR_AMBER)
            # set up I2C slave
            slave = I2CSlave(i2c_id=2, i2c_address=0x45)
            controller.set_strip(strip)
            controller.set_ring(ring)
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
        if slave:
            slave.disable()
    finally:
        if timer2:
            timer2.deinit()
        if timer5:
            timer5.deinit()

main()

#EOF
