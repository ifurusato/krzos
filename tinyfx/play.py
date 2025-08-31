#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-28
# modified: 2025-04-27
#

import time
from tiny_fx import TinyFX
from colorama import Fore, Style

from sound_dictionary import _sounds

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

tiny = None

try:

    tiny = TinyFX(wav_root="/sounds") # Create a new TinyFX object and tell with where the wav files are located

    print(Fore.GREEN + Style.DIM + '-- press for boot button to exit play loop…' + Style.RESET_ALL)

    while True:
        for _name, _filename in _sounds:
            print(Fore.CYAN + '-- playing ' + Fore.YELLOW + Style.BRIGHT + '{}'.format(_name) + Fore.CYAN + Style.NORMAL
                    + ' from file ' + Style.BRIGHT + '{}'.format(_filename) + Style.NORMAL + '…' + Style.RESET_ALL)
            tiny.wav.play_wav(_filename)
            while tiny.wav.is_playing():
                time.sleep(0.05)
            time.sleep(2.0)
            if tiny.boot_pressed():
                while tiny.boot_pressed():
                    print(Fore.GREEN + Style.DIM + '-- let go of the button!' + Style.RESET_ALL)
                    # wait until button released
                    time.sleep(0.01)
                break
        print('')
        while not tiny.boot_pressed():
            print(Fore.GREEN + Style.DIM + '-- waiting for boot button…' + Style.RESET_ALL)
            time.sleep(1.0)

# turn off all the outputs and audio
finally:
    if tiny:
        tiny.shutdown()

