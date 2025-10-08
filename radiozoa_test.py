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

# thresholds in millimeters
CLOSE_THRESHOLD = 100
NEAR_THRESHOLD  = 200
MID_THRESHOLD   = 600
FAR_THRESHOLD   = 1000
# Out of range: >= FAR_THRESH or None/<=0

def color_for_distance(dist):
    match dist:
        case d if d is None or d <= 0:
            return Fore.BLACK + Style.DIM
        case d if d < CLOSE_THRESHOLD:
            return Fore.MAGENTA + Style.BRIGHT
        case d if d < NEAR_THRESHOLD:
            return Fore.RED
        case d if d < MID_THRESHOLD:
            return Fore.YELLOW
        case d if d < FAR_THRESHOLD:
            return Fore.GREEN
        case _:
            return Fore.BLACK

def print_distances(distances):
    '''
    Callback to print distance values from all eight sensors, with colorized output
    based on thresholds for "close", "near", "mid", "far" and "out of range".
    '''
    msg = "["
    for i, dist in enumerate(distances):
        color = color_for_distance(dist)
        if dist is not None and dist > 0:
            msg += f" {color}{dist:>4}mm{Style.RESET_ALL}"
        else:
            msg += f" {color}None{Style.RESET_ALL}"
        if i < len(distances)-1:
            msg += ","
    msg += " ]"
    print(msg)

def print_distances_simple(distances):
    '''
    Callback to print distance values from all eight sensors.
    '''
    msg = "["
    for i, dist in enumerate(distances):
        if dist is not None and dist > 0:
            msg += " {:>4}mm".format(dist)
        else:
            msg += "  None"
        if i < len(distances)-1:
            msg += ","
    msg += " ]"
    print(msg)

def main():
    '''
    Main function to demonstrate the Radiozoa class.
    '''
    enabled = True
    RANGE = 1001
    config_file = 'radiozoa_conf.yaml'
    radiozoa = None
    log = Logger('main', level=Level.INFO)
    try: 
        log.info('loading configuration…')
        loader = ConfigLoader()
        config = loader.configure(config_file)
        
        log.info('instantiating sensor array…')
        radiozoa = Radiozoa(config)
        radiozoa.enable()
        if radiozoa.enabled:
            log.info('start ranging…')
            radiozoa.start_ranging()
            radiozoa.set_callback(print_distances)
        else:
            raise Exception("unable to start ranging due to startup errors.")

        log.info(Fore.GREEN + 'Radiozoa enabled; waiting for sensor callbacks (Ctrl-C to exit)…')
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
