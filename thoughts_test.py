#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-17
# modified: 2025-11-17

import sys
from math import isclose
from colorama import init, Fore, Style
init()

from core.rate import Rate
from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.digital_pot import DigitalPotentiometer
from core.rate_limited import rate_limited
    
_log = Logger('test', Level.INFO)
    
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def key_callback(event):
    _log.info('callback on event: {}'.format(event))

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

@rate_limited(2000)
def play(name):
    print(Fore.WHITE + "play: '{}'".format(name) + Style.RESET_ALL)

def test_digital_potentiometer():

    MIN_RANGE = -1.0
    MAX_RANGE =  1.0

    names = ['blip',        'boink',    'chatter',     'chirp',  'dit_a',     'dit_b',     'dit_c',
             'dwerp',       'earpit',   'glince',      'glitch', 'gwolp',     'ippurt',    'itiz',
             'izit',        'muskogee', 'pizzle',      'sigh',   'skadoodle', 'skid-fzzt', 'tika-tika',
             'tsk-tsk-tsk', 'tweak',    'twiddle-pop', 'twit',   'wow',       'zzt']
    
    try:

        # read YAML configuration
        _level = Level.INFO
        _loader = ConfigLoader(_level)
        filename = 'config.yaml'
        _config = _loader.configure(filename)
        
        _log.info('using digital potentiometer…')
        # configure digital potentiometer for motor speed
        _pot = DigitalPotentiometer(_config, i2c_address=None, level=_level)
        _pot.set_output_range(-1.0, 1.0)
        _last_scaled_value = 0.0
        _log.info('starting test…')
        _hz = 5
        _rate = Rate(_hz, Level.ERROR)
        while True:
            _scaled_value = _pot.get_scaled_value(False)
            if _scaled_value != _last_scaled_value: # if not the same as last time
                if isclose(_scaled_value, 0.0, abs_tol=0.05):
                    _style = Style.DIM
                    _scaled_value = 0.0
                    _pot.set_black()
                else:
                    _style = Style.NORMAL
                    _pot.set_rgb(_pot.value)
                    play('name')
            _activity = abs(_scaled_value)
            _log.info(Fore.YELLOW + _style + 'output: {:7.4f}; activity: {:4.2f}'.format(_scaled_value, _activity))
            _last_scaled_value = _scaled_value
            _rate.wait()
    
    except KeyboardInterrupt:
        print('\n')
        _log.info('Ctrl-C caught; exiting…')
    except DeviceNotFound as e:
        _log.error('no potentiometer found, exiting.')
    except Exception as e:
        _log.error('{} encountered, exiting: {}'.format(type(e), e))
    finally:
        _log.info('complete.')
    
def main():
    try:
        test_digital_potentiometer()
    except Exception as e:
        print(Fore.RED + 'error in motor test: {}'.format(e) + Style.RESET_ALL)
    finally:
        pass
        
if __name__== "__main__":
    main()
    
#EOF
