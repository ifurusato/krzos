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

try:

    # pin 0 is strip
    pin = 1
    count = 24
    pixel = Pixel(pin=pin, pixel_count=count, brightness=0.1)

    if RAINBOW_TEST:
#       cycler = BlinkPattern(pixel, count, offset=12, auto_rotate=True)
#       cycler = PixelCycler(pixel, count)
        cycler = RainbowCycler(pixel, count, hue_step=0.02) # 0.002 is slow

        timer2 = Timer()
        timer2.init(freq=48, mode=Timer.PERIODIC, callback=lambda t: cycler.step())

        while True:
            time.sleep(1)

    if RGB_TEST:
        pixel.set_color(color=COLOR_RED)
        time.sleep(0.2)
        pixel.set_color(color=COLOR_GREEN)
        time.sleep(0.5)
        pixel.set_color(color=COLOR_BLUE)
        time.sleep(0.2)

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
