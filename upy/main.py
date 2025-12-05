#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2025-12-03

import sys
import time
from machine import Pin, Timer

from colors import*
from i2c_slave import I2CSlave
from controller import Controller
from pixel import Pixel
from pixel_cycler import PixelCycler
from blink_pattern import BlinkPattern
from rainbow_cycler import RainbowCycler

# auto-clear: remove cached modules to force reload
for mod in ['main', 'i2c_slave', 'controller']:
    if mod in sys.modules:
        del sys.modules[mod]

def main():
    I2C_SLAVE  = True
    CYCLE_TEST = False
    BLINK      = True
    timer0 = None
    timer1 = None
    timer2 = None
    strip  = None
    cycler = None

    try:

        controller = Controller()

        count = 24
        ring = Pixel(pin='B14', pixel_count=count, brightness=0.1)
        strip = Pixel(pin='B12', pixel_count=8, brightness=0.1)

        clock_pin = Pin('C12', Pin.OUT)

#       def tick():
#           clock_pin.value(not clock_pin.value())

        # set up 50Hz timer0 on pin GP4 (requires 2x frequency since toggle is half freq)
        timer0 = Timer(hard=True)
        timer0.init(freq=40, mode=Timer.PERIODIC, callback=lambda t: clock_pin.value(not clock_pin.value()))
#       timer0.init(freq=40, mode=Timer.PERIODIC, callback=lambda t: tick())
        
        if BLINK:
            timer1 = Timer()
            timer1.init(freq=2, mode=Timer.PERIODIC, callback=controller.step)
            strip.set_color(index=0, color=COLOR_AMBER)
        
        if CYCLE_TEST:
    #       cycler = BlinkPattern(ring, count, offset=12, auto_rotate=True)
    #       cycler = PixelCycler(ring, count)
            cycler = RainbowCycler(ring, count, hue_step=0.02) # 0.002 is slow
            timer2 = Timer()
            timer2.init(freq=48, mode=Timer.PERIODIC, callback=lambda t: cycler.step())

            while True:
                time.sleep(1)

        if I2C_SLAVE:
            # set up I2C slave
            slave = I2CSlave(i2c_id=1, i2c_address=0x45)
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
        slave.disable()
    finally:
        if timer0:
            timer0.deinit()
        if timer1:
            timer1.deinit()
        if timer2:
            timer2.deinit()
        if cycler:
            cycler.close()

main()

#EOF
