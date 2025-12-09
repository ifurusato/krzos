# Test script for PMW3901 / PAA5100 MicroPython driver.
# This variant shows how to enable Pimoroni vendor init ("secret sauce").

import sys
import machine, time
from pmw3901_rp2040 import create_paa5100

# auto-clear: remove cached modules to force reload
for mod in ['main', 'test_pmw3901', 'test_paa5100', 'pmw3901']:
    if mod in sys.modules:
        del sys.modules[mod]

# instantiate the PAA5100 wrapper and enable Pimoroni secret_sauce init
nofs, irq_pin = create_paa5100()

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
