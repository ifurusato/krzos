#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-12
# modified: 2025-07-06

import sys
from uart.uart_master import UARTMaster
from hardware.value_provider import RotaryEncoderCommandProvider, DigitalPotSpeedProvider
from colorama import init, Fore, Style
init()

from core.config_loader import ConfigLoader
from core.logger import Logger, Level

if __name__ == "__main__":

    _log = Logger('driver', Level.INFO)
    _delay_sec=1.0

    # if using closed loop, set digital pot max to motor max speed
    _loader = ConfigLoader(Level.INFO)
    _config = _loader.configure('upy/config.yaml')
    _cfg = _config['kros']['motor_controller']
    _use_closed_loop = _cfg['use_closed_loop']
    _max_motor_speed = _cfg['max_motor_speed']
    _log.info('maximum motor speed set to: {} RPM.'.format(_max_motor_speed))
    if _use_closed_loop:
        _multiplier = float(_max_motor_speed)
        _log.info(Fore.GREEN + 'using closed loop multiplier of {}'.format(_multiplier))
    else:
        _multiplier = 100.0
        _log.info(Fore.GREEN + 'using open loop multiplier of 100.0')
    _command_provider = RotaryEncoderCommandProvider()
    _speed_provider   = DigitalPotSpeedProvider(multiplier=_multiplier)

    # instantiate the UARTMaster and run in a loop
    master = UARTMaster(_config, port='/dev/ttyACM1')
    _log.info(Fore.GREEN + 'starting…')
    master.run(_command_provider, _speed_provider, delay_sec=_delay_sec)
    _log.info(Fore.GREEN + 'complete.')

#EOF
