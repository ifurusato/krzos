
import sys
from machine import Pin, I2C
from vl53l0x import VL53L0X

# force module reload
for mod in ['main', 'tof_test', 'vl53l0x']:
    if mod in sys.modules:
        del sys.modules[mod]

i2c_id      = 1
i2c_address = 0x30

print("setting up i2c on I2C{}…".format(i2c_id))
i2c = I2C(i2c_id)

print(i2c.scan())

print("creating VL53L0X at 0x{:02X}…".format(i2c_address))
tof = VL53L0X(i2c, address=i2c_address)

# Pre: 12 to 18 (initialized to 14 by default)
# Final: 8 to 14 (initialized to 10 by default)

# the measuting_timing_budget is a value in ms, the longer the budget, the more accurate the reading. 
budget = tof.measurement_timing_budget_us
print("timing budget was: {}".format(budget))
tof.set_measurement_timing_budget(40000)

# Sets the VCSEL (vertical cavity surface emitting laser) pulse period for the
# given period type (VL53L0X::VcselPeriodPreRange or VL53L0X::VcselPeriodFinalRange) 
# to the given value (in PCLKs). Longer periods increase the potential range of
# the sensor. Valid values are (even numbers only):

# tof.set_Vcsel_pulse_period(tof.vcsel_period_type[0], 18)
tof.set_Vcsel_pulse_period(tof.vcsel_period_type[0], 12)

# tof.set_Vcsel_pulse_period(tof.vcsel_period_type[1], 14)
tof.set_Vcsel_pulse_period(tof.vcsel_period_type[1], 8)

print("start ranging…")
#while True:
#    print(tof.ping()-50, "mm")
offset = 50
try:
    tof.start()
    while True:
        distance = tof.read() - offset
        print('{}mm'.format(distance))
except KeyboardInterrupt:
    print("Ctrl-C caught; exiting…")
finally:
    print("stop ranging…")
    tof.stop()

#EOF
