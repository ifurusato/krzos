
print('a.')
import time
from pyb import UART

print('b.')
uart = UART(3)
uart.init(baudrate=115200, bits=8, parity=None, stop=1)

print('c.')
try:
    print("UART3 listening...")
    while True:
        if uart.any():
            data = uart.readline()
            if data:
                print("Received:", data.decode().strip())
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Ctrl-C.")
finally:
    print("complete.")
