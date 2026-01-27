
import pyb

from colorama import Fore, Style

from logger import Logger, Level
from i2cscanner import I2CScanner
from device import *

RadiozoaConfig:

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

log = Logger('configure', level=Level.INFO)

for dev in Device.all():
    log.info("device '{}'; I2C address: 0x{:02X}; xshut: {}".format(dev.label, dev.i2c_address, dev.xshut))

# If 0x29 is present or any configured addresses are missing,
# attempts to configure sensors via RadiozoaConfig.
# Test passes only if 0x29 is absent and all eight addresses are present after configuration.

i2c_scanner = I2CScanner()
devices = i2c_scanner.scan()
for device in devices:
    log.info(Style.DIM + '  device: 0x{:02X}'.format(device))

_default_i2c_address = '0x29'

_sensor_addresses = [
    "0x{:02X}".format(dev.i2c_address)
    for dev in Device.all()
]

for _address in _sensor_addresses:
    log.info('address: {}'.format(_address))

_has_default = i2c_scanner.has_hex_address([_default_i2c_address])
_missing = [ addr for addr in _sensor_addresses if not i2c_scanner.has_hex_address([addr]) ]

if _has_default or _missing:
    log.info(Fore.GREEN + "found default 0x{:02X} device; reasssigning radiozoa addresses…")

else:
    log.info(Fore.GREEN + "radiozoa configured.")


log.info('complete.')
