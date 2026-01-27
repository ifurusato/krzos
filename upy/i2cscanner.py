
import pyb

class I2CScanner:
    def __init__(self, i2c_bus=1, led_num=1):
        # Initialize I2C as controller (master)
        self.i2c = pyb.I2C(i2c_bus, pyb.I2C.CONTROLLER)
        self.led = pyb.LED(led_num)
        self._devices = []

    @property
    def devices(self):
        '''
        Return the scanned list of devices.
        '''
        self._devices = []

    def scan(self):
        '''
        Scan all valid 7-bit I2C addresses, returning a list of found addresses.
        '''
        print("Starting I2C scan...")
        for addr in range(0x08, 0x78):
            try:
                # Send one byte to force the address phase
                self.i2c.send(b'\x00', addr)
                # If no exception, device responded
                self._devices.append(addr)
                self.led.toggle()      # LED flash per device
                print("Device found at", hex(addr))
                pyb.delay(150)         # brief pause to see LED toggle
            except OSError:
                # No device at this address
                pass

        if len(self._devices) == 0:
            print("no I2C devices found.")
        print("I2C scan complete: {} devices found.".format(len(self._devices)))
        return self._devices

    def has_hex_address(self, addr):
        '''
        Returns True if a given address is in the scanned device list.
        '''
        return addr in self._devices

# .....................................................................
#
#   scanner = I2CScanner()
#   devices = scanner.scan()
#   for device in devices:
#       print('  device: 0x{:02X}'.format(device))
#
#   print('-- has: 0x29 ? {}'.format(scanner.has_hex_address(0x29)))
#   print('-- has: 0x39 ? {}'.format(scanner.has_hex_address(0x39)))

