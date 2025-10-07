#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-09-21
# modified: 2025-09-21

import time
import math
import pytest
from gpiozero import DigitalInputDevice
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

__log = Logger('test-extclock', level=Level.INFO)

pulse_count = 0

# function to be called on each rising edge
def count_pulse():
    global pulse_count
    pulse_count += 1

@pytest.mark.unit
def test_extclock():
    global pulse_count
    __log.info(Style.BRIGHT + "testing if external clock is generating a signal…")

    INPUT_PIN = 23

    # target frequency and tolerance
    TARGET_FREQUENCY   = 20.0 # Hz
    TOLERANCE_PERCENT  = 3.0
    RELATIVE_TOLERANCE = TOLERANCE_PERCENT / 100.0

    # test parameters
    NUM_ITERATIONS = 10
    DURATION = 0.25 # seconds per iteration

    # create a DigitalInputDevice instance
    device = DigitalInputDevice(INPUT_PIN, pull_up=False)

    # Initialize variables
    frequency_readings = []
    start_time = time.time()
    iteration_count = 0

    # when the pin becomes active (rising edge), call the count_pulse function
    device.when_activated = count_pulse

    __log.info("starting frequency test on GPIO {}…".format(INPUT_PIN))
    __log.info("target Frequency: {:.2f} Hz; tolerance: {:.2f}Hz".format(TARGET_FREQUENCY, TOLERANCE_PERCENT))
    __log.info(Style.DIM + "-" * 40)

    try:
        while iteration_count < NUM_ITERATIONS:
            # check if one second has passed for the current iteration
            if time.time() - start_time >= DURATION:
                # calculate the frequency for this iteration
                frequency = pulse_count / DURATION
                frequency_readings.append(frequency)
                __log.info(Style.DIM + "[{:02d}] detected frequency of {:.2f} Hz.".format(iteration_count + 1, frequency))
                # reset for the next iteration
                pulse_count = 0
                start_time = time.time()
                iteration_count += 1
            time.sleep(0.01)

    except KeyboardInterrupt:
        __log.info('Ctrl-C caught; exiting…')

    finally:
        # Stop event listener before processing results
        device.when_activated = None

        if frequency_readings:
            # Calculate the average frequency
            average_frequency = sum(frequency_readings) / len(frequency_readings)
            __log.info("-" * 40)
            __log.info("test complete.")
            __log.info("average frequency over {} iterations: ".format(NUM_ITERATIONS)  + Fore.GREEN + '{:.2f}Hz'.format(average_frequency))
            # compare the average frequency to the target using math.isclose()
            assert math.isclose(
                average_frequency,
                TARGET_FREQUENCY,
                rel_tol=RELATIVE_TOLERANCE
            ), (
                "the sampled average frequency of {:.2f} Hz is NOT within {:.2f}% of the target {} Hz."
                .format(average_frequency, TOLERANCE_PERCENT, TARGET_FREQUENCY)
            )
            __log.info("the sampled average frequency of {:.2f} Hz is within {:.2f}% of the target {} Hz.".format(
                    average_frequency, TOLERANCE_PERCENT, TARGET_FREQUENCY))
        else:
            pytest.fail("no frequency data was collected.")
        __log.info(Fore.GREEN + "external clock is functional.")

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def main():
    global __log
    """
    Runs only the marked unit tests within this file when executed directly.
    """
    try:
        pytest.main([__file__, "-m", "unit", "-s"])
        __log.info("test execution complete.")
    except Exception as e:
        __log.error("an unexpected error occurred: {}".format(e))
    finally:
        __log = None

if __name__ == "__main__":
    main()

#EOF
