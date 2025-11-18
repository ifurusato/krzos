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
import time
from math import isclose
from random import choice, expovariate
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.digital_pot import DigitalPotentiometer
from core.rate_limited import rate_limited
from hardware.tinyfx_controller import TinyFxController

MIN_RANGE = -1.0
MAX_RANGE =  1.0
SOUNDS = [
    'blat',        'blip',        'chatter',     'dwerp',       'dwoit',       'earpit',  
    'glitch',      'gwolp',       'ippurt',      'itiz',        'izit',        'muskogee',    
    'pizzle',      'purr',        'skadoodle',   'skid-fzzt',   'skiff',       'tititicha', 
    'tweak',       'twiddle-pop', 'twit',        'whistle',     'zzt'
]

_log = Logger('test', Level.INFO)

@rate_limited(1500) # ms
def play(controller, name):
    _log.info('play: ' + Fore.WHITE + Style.BRIGHT + "'{}'".format(name) + Style.RESET_ALL)
    response = controller.send('play {}'.format(name))
    _log.info('response: {}'.format(response))

def main():
    try:
        _level = Level.INFO
        _loader = ConfigLoader(_level)
        filename = 'config.yaml'
        _config = _loader.configure(filename)
        _controller = TinyFxController(_config, level=Level.INFO)
        _controller.enable()
        _pot = DigitalPotentiometer(_config, i2c_address=None, level=_level)
        _pot.set_output_range(-1.0, 1.0)
        _last_scaled_value = 0.0

        last_name      = None
        jitter_factor = 2.5
        max_frequency  = 0.8  # at full activity, 1.0 calls play() once per second
        next_play_time = time.monotonic()

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
            _activity = abs(_scaled_value)
            now = time.monotonic()
            frequency = max_frequency * _activity
            if frequency > 0:
                # If it's time to play a sound
                if now >= next_play_time:
#                   name = choice(SOUNDS)
                    name = choice([n for n in SOUNDS if n != last_name])
                    last_name = name
                    _log.info(Style.DIM + "calling play: '{}'".format(name))
                    play(_controller, name)
                    interval = expovariate(frequency / jitter_factor)
                    next_play_time = now + interval
            _log.info(Fore.BLUE + _style + 'output: {:7.4f}; activity: {:4.2f}'.format(_scaled_value, _activity))
            _last_scaled_value = _scaled_value
            time.sleep(0.1)
    except KeyboardInterrupt:
        _log.info('\nCtrl-C caught; exiting…')
    except Exception as e:
        _log.error('{} encountered, exiting: {}'.format(type(e), e))
    
if __name__== "__main__":
    main()
    
#EOF
