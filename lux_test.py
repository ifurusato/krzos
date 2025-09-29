#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-09-29
# modified: 2025-09-29

import time
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.config_loader import ConfigLoader
from hardware.lux_sensor import LuxSensor
from hardware.tinyfx_controller import TinyFxController

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('main', Level.INFO)

CONTROL_HEADLIGHT = False
_luxSensor = None
_tinyfx = None
try:
    _config = ConfigLoader(Level.INFO).configure()
    _luxSensor = LuxSensor(_config)
    if CONTROL_HEADLIGHT:
        _tinyfx = TinyFxController(config=_config, level=Level.INFO)

    # rising and falling callbacks
    def on_lux_rise(lux_value):
        if CONTROL_HEADLIGHT:
            _tinyfx.send_data("off")
        _log.info(Fore.YELLOW + "Callback: lux rose above threshold! Value: {:06.2f}".format(lux_value) + Style.RESET_ALL)

    def on_lux_fall(lux_value):
        if CONTROL_HEADLIGHT:
            _tinyfx.send_data("fwd")
        _log.info(Fore.BLUE + "Callback: lux fell below threshold! Value: {:06.2f}".format(lux_value) + Style.RESET_ALL)

    _luxSensor.start_monitoring(rising_callback=on_lux_rise, falling_callback=on_lux_fall)

    while True:
        lux = _luxSensor.lux
        if lux < _luxSensor.get_lux_threshold():
            _log.info(Style.DIM + "lux: {:06.2f}".format(lux))
        else:
            _log.info("lux: {:06.2f}".format(lux))
        time.sleep(0.05)

except KeyboardInterrupt:
    pass
finally:
    if _luxSensor:
        _luxSensor.close()
    if _tinyfx:
        _tinyfx.close()

#EOF
