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

import time
from colorama import Fore, Style

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.radiozoa import Radiozoa

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():
    '''
    Main function to demonstrate the Radiozoa class.
    '''
    enabled = True
    RANGE = 1001
    config_file = 'radiozoa_conf.yaml'
    radiozoa = None
    _log = Logger('main', level=Level.INFO)
    try: 
        _log.info('loading configuration…')
        loader = ConfigLoader()
        config = loader.configure(config_file)
        
        _log.info('instantiating sensor array…')
        radiozoa = Radiozoa(config)
        radiozoa.enable()
        _log.info('setting up sensor array…')
        if radiozoa.enabled:
            _log.info('start ranging…')
            radiozoa.start_ranging()
        else:
            raise Exception("unable to start ranging due to startup errors.")
        if radiozoa._active_sensors:
            timing = radiozoa._active_sensors[0].tof.get_timing()
            if timing < 20000:
                timing = 20000
            _log.info("timing {} ms".format(timing/1000))
            for count in range(1, RANGE):
                distances = radiozoa.get_all_distances()
                for reading in distances:
                    sensor_id = reading.get('id')
                    label     = reading.get('label')
                    distance  = reading.get('distance')
                    if distance > 0:
                        _log.info("[{:04d}] sensor {} ({})\t {}mm, {}cm".format(count, label, sensor_id, distance, (distance / 10)))
                    else:
                        _log.warning("[{:04d}] sensor {} ({})\t error".format(count, label, sensor_id))
                    if not enabled:
                        break
                time.sleep(timing / 1000000.00)
                
    except KeyboardInterrupt:
        _log.info('Ctrl-C caught, exiting…')
        enabled = False
    except Exception as e:
        _log.info('{} raised during radiozoa processing: {}'.format(type(e).__name__, e))
        enabled = False
    finally:
        _log.info('finally…')
        if radiozoa:
            radiozoa.close()
        _log.info('complete.')
            
if __name__ == '__main__':
    main()

#EOF
