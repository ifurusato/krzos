#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2024 by Murray Altheim. All rights reserved. This file is part of
# the Robot Operating System project and is released under the "Apache Licence,
# Version 2.0". Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-04-13
# modified: 2025-05-15
#

import sys
import time
import pyb
from pyb import LED
from pyb import Timer
from pyb import Pin


# constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

LED_PIN = 1

# methods ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def hello():
    '''
    A little blinking greeting.
    '''
    for _ in range(3):
        led.on()
        pyb.delay(100)
        led.off()
        pyb.delay(900)
    pyb.delay(500)

def blink():
    '''
    Flash the LED.
    '''
    led.on()
#   led.intensity(128)
    time.sleep_ms(50)
    led.off()
    time.sleep_ms(50)

# ..............................................................................

#_timer8 = Timer(8, freq=1)
#_timer8.callback(lambda n: blink())

# set up clock output pin
#_clock_pin = Pin('Y10', Pin.OUT_PP)
# start external clock loop with frequency of 20Hz (50ms)
#_timer9 = Timer(9, freq=20)
#_timer9.callback(lambda n:_clock_pin.value(not _clock_pin.value()))


print('start…')

led = LED(LED_PIN)

try:

    # this gives us a little time to manually interrupt things at the start
    hello()

    # do something intelligent here
    pass

except KeyboardInterrupt:
    print('Ctrl-C caught; exiting…')
except Exception as e:
    print('{} raised in main loop: {}'.format(type(e), e))
    sys.print_exception(e)
finally:
    print('complete.')

#EOF
