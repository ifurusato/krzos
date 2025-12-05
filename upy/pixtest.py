#!/micropython

import time
from machine import Timer

from colors import *
from pixel import Pixel
from pixel_cycler import PixelCycler
from blink_pattern import BlinkPattern
from rainbow_cycler import RainbowCycler


pixel = None
cycler = None

RGB_TEST = False
RAINBOW_TEST = True
SEQUENCE_TEST = False

try:

    pin = 5
    count = 24
    pixel = Pixel(pin=pin, pixel_count=count, brightness=0.1)

    timer2 = Timer()
    timer2.init(freq=48, mode=Timer.PERIODIC, callback=lambda t: cycler.step())

    if RAINBOW_TEST:
#       cycler = BlinkPattern(pixel, count, offset=12, auto_rotate=True)
#       cycler = PixelCycler(pixel, count)
        cycler = RainbowCycler(pixel, count, hue_step=0.02) # 0.002 is slow
        while True:
            time.sleep(1)

    if RGB_TEST:
        pixel.set_color(color=COLOR_RED)
        time.sleep(0.2)
        pixel.set_color(color=COLOR_GREEN)
        time.sleep(0.5)
        pixel.set_color(color=COLOR_BLUE)
        time.sleep(0.2)

    if SEQUENCE_TEST:
        _colors = [ COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW ]
        while True:
            # each pixel changes color:
#           for index in range(0, count):
#               _color = _colors[index % len(_colors)]
#               pixel.set_color(index=index, color=_color)
#               time.sleep(0.02)
#               pixel.set_color(index=index, color=COLOR_BLACK)
#               time.sleep(0.01)
            # each cycle changes color:
            for color in _colors:
                for index in range(count):   # one full rotation
                    pixel.set_color(index=index, color=color)
                    time.sleep(0.0667)
                    pixel.set_color(index=index, color=COLOR_BLACK)
                    time.sleep(0.005)

except KeyboardInterrupt:
    print('Ctrl-C caught; exiting…')
except Exception as e:
    print('{} raised in main loop: {}'.format(type(e), e))
finally:
    if pixel:
        pixel.off()
    if cycler:
        cycler.close()
    print('complete.')

#EOF
