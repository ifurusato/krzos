# a convenient alias for testing that forgets its self

import sys

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def _blink():
    from pyb import LED
    import time
    _led = LED(1)
    for _ in range(7):
        _led.on()
        time.sleep_ms(10)
        _led.off()
        time.sleep_ms(324)

_blink()

# enable Emergency Exception Buffer
import micropython
micropython.alloc_emergency_exception_buf(256)

def __reload_module(name):
    import sys
    if name in sys.modules:
        del sys.modules[name]
    return __import__(name)

def _load_main():

    # forget the modules you want to reload
    for mod in ['free', 'cwd', 'main', 'uart_slave_app']:
        print("forgetting '{}'".format(mod))
        sys.modules.pop(mod, None)

    # import in the desired order (they now re-execute)
    import free
    import cwd

    # forget app.py itself so it re-runs next time
    sys.modules.pop(__name__, None)

    #print('┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈')
    #pring('executing UartSlaveApp…')
    #import uart_slave_app

    __main = __reload_module('uart_slave_app')
    __main.exec()

#_load_main()
