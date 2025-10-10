#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-09-08
# modified: 2025-09-08
#

import signal
import time
from colorama import Fore, Style

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.radiozoa import Radiozoa

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

from colorama import Fore, Style

def main():
    '''
    Main function to demonstrate the Radiozoa class.
    '''
    enabled = True
    RANGE = 1001
    radiozoa = None
    log = Logger('main', level=Level.INFO)
    try: 
        log.info('loading configuration…')
        config = ConfigLoader(Level.INFO).configure()
        
        log.info('instantiating sensor array…')
        radiozoa = Radiozoa(config)
        radiozoa.enable()
        timeout_seconds = 5
        start_time = time.monotonic()
        while not radiozoa.enabled:
            if time.monotonic() - start_time > timeout_seconds:
                raise TimeoutError("radiozoa failed to enable within timeout")
            time.sleep(0.1)
        radiozoa.set_visualisation(True)
        radiozoa.start_ranging()
        log.info(Fore.GREEN + 'radiozoa enabled (Ctrl-C to exit)…')
        # keep alive with signal.pause() if available
        try:
            signal.pause()
        except AttributeError:
            # signal.pause() is not available on Windows; fallback to sleep loop
            while True:
                time.sleep(1)

    except KeyboardInterrupt:
        log.info('Ctrl-C caught, exiting…')
        enabled = False
    except Exception as e:
        import traceback
        log.error('{} raised during radiozoa processing: {}\n{}'.format(type(e).__name__, e, traceback.format_exc()))
        enabled = False
    finally:
        log.info('finally…')
        if radiozoa:
            radiozoa.close()
        log.info('complete.')
            
if __name__ == '__main__':
    main()

#EOF
