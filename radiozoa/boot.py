# boot.py -- run on boot-up

USE_NETWORK = True

if USE_NETWORK:
    import network

    try:
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        wlan.config(country='NZ')
    except Exception:
        pass
