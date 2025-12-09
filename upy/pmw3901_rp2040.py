#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-12-09
# modified: 2025-12-09

import machine
from machine import Pin
import pmw3901

def create_paa5100(spi_id=0, cs_pin=17, sck_pin=18, mosi_pin=19, miso_pin=16,
                   int_pin=None, led_pin=None, baudrate=500000, secret_sauce=True):
    '''
    Helper wrapper for RP2040 / Raspberry Pi Pico to instantiate PAA5100 with
     sane defaults, creates and returns a configured PAA5100 instance for RP2040.

    Defaults (SPI0):
      MISO (RX) : GP16
      CS        : GP17
      SCK       : GP18
      MOSI (TX) : GP19

    The function accepts overrides for all pins and parameters.

    Parameters:
      spi_id      - SPI bus id (0 or 1)
      cs_pin      - chip select GPIO (default GP17)
      sck_pin     - SPI SCK pin (default GP18)
      mosi_pin    - SPI MOSI pin (default GP19)
      miso_pin    - SPI MISO pin (default GP16)
      int_pin     - optional INT pin number (returned as second value if provided)
      led_pin     - optional MCU LED pin number (Pin will be created if provided)
      baudrate    - SPI baudrate (default 500000)
      secret_sauce- if True, runs Pimoroni vendor init on creation (default True)

    Returns:
      (instance, irq_pin) where irq_pin is None unless int_pin was provided.
    '''
    print('create_paa5100; spi_id: {}; cs: {}; sck: {}; mosi: {}; miso: {}; int: {}'.format(
        spi_id, cs_pin, sck_pin, mosi_pin, miso_pin, int_pin))
    spi = machine.SPI(spi_id, baudrate=baudrate, polarity=1, phase=1,
                      sck=Pin(sck_pin), mosi=Pin(mosi_pin), miso=Pin(miso_pin))
    cs = Pin(cs_pin, Pin.OUT)
    led = None
    if led_pin is not None:
        led = Pin(led_pin, Pin.OUT)
    inst = pmw3901.PAA5100(spi=spi, cs=cs, led_pin=led, secret_sauce=secret_sauce)
    irq_pin = None
    if int_pin is not None:
        irq_pin = Pin(int_pin, Pin.IN)
    return inst, irq_pin

#EOF
