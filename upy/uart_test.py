
import pyb

# Use UART 3
uart = pyb.UART(3, 1_000_000)

print("STM32 UART echo test ready.")

while True:
    if uart.any():
        data = uart.read()
        if data:
            uart.write("echo: {}".format(data))
    else:
        print('no uart.')

print('done.')
