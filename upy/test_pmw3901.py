# Test script for PMW3901 / PAA5100 MicroPython driver.
# This variant shows how to enable Pimoroni vendor init ("secret sauce").

import sys
import machine, time
from pmw3901 import PAA5100

# auto-clear: remove cached modules to force reload
for mod in ['main', 'test_pmw3901', 'pmw3901']:
    if mod in sys.modules:
        del sys.modules[mod]

# defaults: SPI(1) and CS PA4. Adjust if your board uses different pins.
spi = machine.SPI(1, baudrate=500000, polarity=1, phase=1)
try:
    cs = machine.Pin('PA4', machine.Pin.OUT)
except Exception:
    cs = machine.Pin(4, machine.Pin.OUT)

# optional LED pin (set to the actual LED pin on your board if present)
led_pin = None
#try:
#    led_pin = machine.Pin('PA3', machine.Pin.OUT)
#except Exception:
#    led_pin = None

# instantiate the PAA5100 wrapper and enable Pimoroni secret_sauce init:
nofs = PAA5100(spi=spi, cs=cs, led_pin=led_pin, secret_sauce=True)

print("product ID: {}, revision: {}".format(hex(nofs.id), hex(nofs.revision)))
time.sleep_ms(50)

# for PAA5100, use the Pimoroni-style burst read get_motion loop (returns x,y)
try:
    for i in range(2000):
        try:
            x, y = nofs.get_motion(timeout=1.0)
            print("motion: ({}, {})".format(x, y))
            time.sleep_ms(200)
        except Exception as e:
            print("motion read error:", e)
except KeyboardInterrupt:
    pass
finally:
    print('finally.')
    if nofs:
        nofs.disable_sensor_led()

#EOF
