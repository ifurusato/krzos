# a convenient alias for testing that forgets its self

import sys
from logger import Logger, Level
from colorama import Fore, Style

def _blink():
    from pyb import LED
    import time
    _led = LED(1)
    for _ in range(7):
        _led.on()
        time.sleep_ms(10)
        _led.off()
        time.sleep_ms(275)

_blink()

# enable Emergency Exception Buffer
import micropython
micropython.alloc_emergency_exception_buf(256)

__log = Logger('mt', level=Level.INFO)

# Forget the modules you want to reload
for mod in ['free', 'cwd', 'main', 'motor_test']:
    __log.debug("forgetting '{}'".format(mod))
    sys.modules.pop(mod, None)

# Import in the desired order (they now re-execute)
import free
import cwd

# forget mt.py itself so it re-runs next time
sys.modules.pop(__name__, None)

__log.info(Style.BRIGHT + '┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈')
__log.info(Style.BRIGHT + 'executing Driver…')
#import motor_test

def reload_module(name):
    import sys
    if name in sys.modules:
        del sys.modules[name]
    return __import__(name)

main = reload_module('motor_test')
main.exec()

