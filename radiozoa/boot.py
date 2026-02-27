# boot.py -- run on boot-up

#from machine import usb_mode
#
#if usb_mode() != "CDC+MSC":
#    usb_mode("CDC+MSC")
#
#USE_NETWORK = False
#
#if USE_NETWORK:
#    import network
#
#    try:
#        wlan = network.WLAN(network.STA_IF)
#        wlan.active(True)
#        wlan.config(country='NZ')
#    except Exception:
#        pass
