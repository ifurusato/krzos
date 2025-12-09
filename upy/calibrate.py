#!/micropython
# -*- coding: utf-8 -*-
#
# Calibration script for Odometer with PAA5100
# Uses RP2040 wrapper with IRQ support

import sys
import time
from pmw3901_rp2040 import create_paa5100
from odometer import Odometer

# auto-clear: remove cached modules to force reload
for mod in ['main', 'calibrate', 'test_pmw3901', 'test_paa5100', 'pmw3901']:
    if mod in sys.modules:
        del sys.modules[mod]

#RESOLUTION_FACTOR = 0.009868
#RESOLUTION_FACTOR = 0.009871
RESOLUTION_FACTOR = 0.009537

# .............................................................

print("="*60)
print("OPTICAL FLOW ODOMETER CALIBRATION")
print("="*60)

# instantiate the PAA5100 wrapper and enable Pimoroni secret_sauce init
nofs, irq_pin = create_paa5100()
# configure orientation for 180° rotated mounting
nofs.set_orientation(invert_x=True, invert_y=True, swap_xy=False)
print("product ID: {}, revision: {}".format(hex(nofs.id), hex(nofs.revision)))
time.sleep_ms(50)

# Configure orientation for your 180° rotated mounting
print("Setting orientation (180° rotation, facing down)...")
nofs.set_orientation(invert_x=True, invert_y=True, swap_xy=False)

# Create odometer (IRQ mode if irq_pin is provided)
odom = Odometer(nofs, irq_pin=irq_pin, resolution_factor=RESOLUTION_FACTOR)

print("\n" + "="*60)
print("CALIBRATION PROCEDURE")
print("="*60)

print("\nStep 1: Press Enter to start")
input()

odom.reset()

print("\nStep 2: Starting update loop...")
print("   Move the robot 100cm FORWARD")
print("   Press Ctrl-C when done")
print("")

try:
    update_count = 0
    last_print = time.ticks_ms()

    while True:
        # In IRQ mode, update() is called automatically, but calling it again doesn't hurt
        # In polling mode, this is essential
        if not odom.irq_mode:
            odom.update()

        update_count += 1

        # Print status every 200ms
        current = time.ticks_ms()
        if time.ticks_diff(current, last_print) > 200:
            x, y = odom.position
            cumulative = odom.get_cumulative_distance()
            print("x={: 6.2f}cm, y={:6.2f}cm | dist={:6.2f}cm".format(x, y, cumulative))
            last_print = current

        time.sleep_ms(20)  # 50Hz in polling mode, less critical in IRQ mode

except KeyboardInterrupt:
    print("\n\nStep 3: Stopped by user")

x, y = odom.position
cumulative = odom.get_cumulative_distance()

print("\nFinal Results:")
print("  Position: x={:.2f}cm, y={:.2f}cm".format(x, y))
print("  Cumulative distance: {:.2f}cm".format(cumulative))
print("  Update count: {}".format(update_count))

if cumulative > 10:
    print("\nCalibrating based on Y-axis (forward) movement...")
    odom.calibrate(known_distance_cm=100.0, measured_distance_cm=abs(x))
    new_factor = odom.get_resolution_factor()

    print("\n" + "="*60)
    print("*** SAVE THIS TO YOUR CONFIG:  {:.6f} ***".format(new_factor))
    print("="*60)

    # Validation test
    print("\n\nWould you like to validate?  (y/n)")
    try:
        response = input()
        if response.lower() == 'y':
            print("\n" + "="*60)
            print("VALIDATION TEST")
            print("="*60)

            odom.reset()
            print("\nMove robot 50cm forward")
            print("Press Ctrl-C when done")

            try:
                while True:
                    if not odom.irq_mode:
                        odom.update()
                    time.sleep_ms(20)
            except KeyboardInterrupt:
                pass

            x, y = odom.position
            error = abs(abs(x) - 50.0)
            error_pct = error / 50.0 * 100.0

            print("\n\nValidation Results:")
            print("  Expected: 50cm")
            print("  Measured: {:.2f}cm".format(abs(x)))
            print("  Error:    {:.2f}cm ({:.1f}%)".format(error, error_pct))

            if error < 2.0:
                print("  Status: EXCELLENT ✓")
            elif error < 5.0:
                print("  Status: GOOD ✓")
            else:
                print("  Status:  Needs improvement")
    except:
        pass
else:
    print("\nERROR: Not enough movement detected!")
    print("  Cumulative was only {:.2f}cm".format(cumulative))
    print("\nTroubleshooting:")
    print("  1. Make sure update() was being called (check update_count)")
    print("  2. Check sensor is powered and facing ground")
    print("  3. Ensure surface has visible texture (not mirror-smooth)")
    print("  4. Check mounting height (should be 8-20mm typically)")

odom.close()
print("\nCalibration complete!")

#EOF
