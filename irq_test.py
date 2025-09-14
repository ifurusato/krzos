#!/usr/bin/env python3

# -*- coding: utf-8 -*-
#
# Portions copyright 2020-2024 by Murray Altheim. All rights reserved. This file
# is part of the Robot Operating System project, released under the MIT License.
# Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-09-03
# modified: 2024-09-04
#

import sys, traceback
import time
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.irq_clock import IrqClock

def ping():
    print('ping.')

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    _em7180 = None
    _log = Logger('test', Level.INFO)
    _start_time = dt.now()

    try:

        # read YAML configuration
        _config = ConfigLoader(Level.INFO).configure()
        _irq_clock = IrqClock(_config, Level.INFO)
        _irq_clock.add_callback(ping)
        _irq_clock.enable()

        while True:

            time.sleep(1)

    except KeyboardInterrupt:
        _log.info('Ctrl-C caught; exiting…')
    except Exception as e:
        _log.error('error in test: {}'.format(e))
        traceback.print_exc(file=sys.stdout)
    finally:
        _elapsed_ms = round((dt.now() - _start_time).total_seconds() * 1000.0)
        _log.info(Fore.YELLOW + 'complete: elapsed: {:d}ms'.format(_elapsed_ms))

if __name__== "__main__":
    main()

#EOF
