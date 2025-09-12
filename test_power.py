#!/usr/bin/env python3

import time
import sys
import pytest

from ads1015 import ADS1015
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

__log = Logger('test-power', level=Level.INFO)

@pytest.mark.unit
def test_power():

    _config = ConfigLoader().configure('config.yaml')
    _cfg = _config['kros'].get('hardware').get('battery')

    _12v_battery_min = _cfg.get('low_12v_battery_threshold') # 11.5   e.g., in0/ref: 11.953v
    _3v3_min = _cfg.get('low_3v3_regulator_threshold')       #  3.25  e.g., in1/ref:  3.302v
    _5v_min  = _cfg.get('low_5v_regulator_threshold')        #  5.0   e.g., in2/ref:  5.028v

    __log.info("testing power supplies…")
    ads1015 = ADS1015()
    chip_type = ads1015.detect_chip_type()
    __log.info("found device:  " + Fore.GREEN + "{}".format(chip_type))

    ads1015.set_mode("single")
    ads1015.set_programmable_gain(2.048)
    if chip_type == "ADS1015":
        ads1015.set_sample_rate(1600)
    else:
        ads1015.set_sample_rate(860)
    reference = ads1015.get_reference_voltage()
    __log.info("reference:     " + Fore.GREEN + "{:6.3f}V".format(reference))

    try:

        # IN0 12V battery
        value_in0 = ads1015.get_compensated_voltage(channel='in0/ref', reference_voltage=reference)
        assert value_in0 > _12v_battery_min, 'measured in0 value less than threshold {}v'.format(_12v_battery_min)
        __log.info("battery (in0): " + Fore.GREEN + "{:6.3f}V".format(value_in0))

        value_in1 = ads1015.get_compensated_voltage(channel='in1/ref', reference_voltage=reference)
        assert value_in1 > _3v3_min, 'measured in1 value less than threshold {}v'.format(_3v3_min)
        __log.info("3V3 reg (in1): " + Fore.GREEN + "{:6.3f}V".format(value_in1))

        value_in2 = ads1015.get_compensated_voltage(channel='in2/ref', reference_voltage=reference)
        assert value_in2 > _5v_min, 'measured in2 value less than threshold {}v'.format(_5v_min)
        __log.info("5V ref (in2):  " + Fore.GREEN + "{:6.3f}V".format(value_in2))

    except Exception as e:
        __log.error('{} raised in power supply test: {}'.format(type(e), e))

def main():
    test_power()

if __name__ == "__main__":
    main()

#EOF
