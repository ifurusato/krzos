# boot.py -- run on boot-up

import pyb

pyb.country('US')        # ISO 3166-1 Alpha-2 code, eg US, GB, DE, AU
pyb.usb_mode('VCP')
#pyb.usb_mode('VCP+MSC') # act as a serial and a storage device
#pyb.usb_mode('CDC')
#pyb.usb_mode('VCP+HID') # act as a serial device and a mouse
#pyb.main('main.py')     # main script to run after this one
