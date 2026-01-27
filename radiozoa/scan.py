
import sys
from i2c_scanner import I2CScanner

# force module reload
for mod in ['scan', 'i2c_scanner']:
    if mod in sys.modules:
        del sys.modules[mod]

scanner = I2CScanner()
scanner.i2cdetect()

