# boot.py -- run on boot to configure USB and filesystem
# Put app code in main.py

import machine
import pyb
#pyb.main('main.py') # main script to run after this one
#pyb.usb_mode('VCP+MSC') # act as a serial and a storage device
#pyb.usb_mode('VCP+HID') # act as a serial device and a mouse
#import network
#network.country('US') # ISO 3166-1 Alpha-2 code, eg US, GB, DE, AU or XX for worldwide
#network.hostname('...') # DHCP/mDNS hostname

def _blink():
    from pyb import LED
    import time
    _led = LED(1)
    for _ in range(7):
        _led.on()
        time.sleep_ms(10)
        _led.off()
        time.sleep_ms(70)

_blink()
