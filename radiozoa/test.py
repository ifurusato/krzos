
import sys
import pyb

# auto-clear: remove cached modules to force reload
for mod in ['test']:
    if mod in sys.modules:
        del sys.modules[mod]

# initialize I2C1 as controller (master)
i2c = pyb.I2C(2, pyb.I2C.CONTROLLER)

# Scan for devices
devices = i2c.scan()

print("I2C devices found:", [hex(d) for d in devices])

if 0x38 in devices:
    print("✅ Device found at address 0x38")
else:
    print("❌ No device found at address 0x38")

