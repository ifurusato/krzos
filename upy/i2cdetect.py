#!/micropython
#
# from pins.csv for the STM32H562:  I2C3_SDA,PC1; I2C3_SCL,PC0

from pyb import I2C

# Try scanning all possible I2C bus IDs (depending on pins.csv definition)
for bus_id in range(6):
    try:
        print("Scanning I2C bus {}…".format(bus_id))
        i2c = I2C(bus_id)  # Rely on board defaults for SDA/SCL pins
        devices = i2c.scan()
        if devices:
            print("  Found {} device(s):".format(len(devices)))
            for d in devices:
                print("    - I2C address: 0x{:02X}".format(d))
        else:
            print("  No I2C devices found.")
    except Exception as e:
        print("  Error accessing I2C bus {}: {}".format(bus_id, str(e)))
