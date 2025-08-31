#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-25
# modified: 2025-05-26
#

import os, sys, gc
import utime
import _thread
from machine import Timer
import uasyncio as asyncio
from colorama import Fore, Style

from tiny_fx import TinyFX
from picofx import MonoPlayer
#from picofx import ColourPlayer
from picofx.mono import StaticFX
from triofx import TrioFX
from rgb_blink import RgbBlinkFX
from i2c_settable_blink import I2CSettableBlinkFX
from pir import PassiveInfrared

from colors import*
from core.logger import Level, Logger
from controller import Controller
from sound_dictionary import _sounds
from response import*

class TinyFxController(Controller):
    '''
    Extends Controller with TinyFX-specific commands.
    '''
    def __init__(self, tinyfx=None, display=None, level=Level.INFO):
        super().__init__(display=display, level=level)
        self._log = Logger('motor', level)
        self._tinyfx = tinyfx
        self.player = MonoPlayer(self._tinyfx.outputs) # create a new effect player to control TinyFX's mono outputs

        self.blink_fx     = I2CSettableBlinkFX(1, speed=0.5, phase=0.0, duty=0.015) # ch4
        self.stbd_trio_fx = TrioFX(2, brightness=0.8) # ch5
        self.port_trio_fx = TrioFX(3, brightness=0.8) # ch6

        self.pir_sensor = PassiveInfrared()

        # set up the effects to play
        self.player.effects = [
            None, #TrioFX(2, brightness=0.5),  # UNUSED
            None, #TrioFX(3, brightness=1.0),  # UNUSED
            None, # StaticFX(0.7),
            self.blink_fx,
            self.stbd_trio_fx,
            self.port_trio_fx
        ]

        self.pir_enabled = False # default disabled

        self._log.info("starting player…")
        self.player.start()

        self.play('arming-tone')
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def play(self, key):
        for name, filename in _sounds:
            if name == key:
#               self._log.debug("play key: '{}' from file: ".format(key) + Fore.YELLOW + "'{}'".format(filename))
                self._tinyfx.wav.play_wav(filename)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def handle_command(self, command):
        '''
        Extended async processor for motor-specific commands.
        '''
#       self._log.debug("handling command: '{}'".format(command))
        try:
            if command.startswith('help'):
                _thread.start_new_thread(self.help, ())
            elif command.startswith('play'):
                parts = command.strip().split()
                if len(parts) >= 2:
                    name = parts[1]
#                   self._log.debug("play sound: " + Fore.GREEN + "{}".format(name))
#                   _thread.start_new_thread(self.play, (name,))
                    self.play(name)
                else:
                    self._log.warning("no sound name provided.")

            # LED channels ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            elif command == 'ch1':
                pass
            elif command == 'ch2':
                pass
            elif command == 'ch3':
                pass
            elif command == 'ch4' or command == 'mast':
                self.blink_fx.on()
            elif command == 'ch5' or command == 'stbd':
                self.stbd_trio_fx.on()
            elif command == 'ch6' or command == 'port':
                self.port_trio_fx.on()
            elif command == 'on':
                self.blink_fx.on()
                self.port_trio_fx.on()
                self.stbd_trio_fx.on()
            elif command == 'off':
                self.blink_fx.off()
                self.port_trio_fx.off()
                self.stbd_trio_fx.off()

            # system info ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            elif command == 'flash':
                stat = os.statvfs('/') # get filesystem stats
                total_space = int(( stat[0] * stat[2] ) / 1000) # Block size * Total blocks
                free_space  = int(( stat[0] * stat[3] ) / 1000) # Block size * Free blocks
                print("total flash: {}KB".format(total_space))
                print("free flash:  {}KB".format(free_space))
            elif command == 'mem':
                gc.collect()
                ram_mb = gc.mem_free() / 1024
                print("free ram: {:.2f}KB".format(ram_mb))
            elif command == 'sounds':
                print('sounds:')
                for name in self._list_wav_files_without_extension():
                    print('  {}'.format(name))

            # pir ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            elif command == 'pir get':
                pass
                if self.pir_sensor.triggered:
#                   response = RESPONSE_PIR_ACTIVE
                    self.show_color(COLOR_ORANGE)
                else:
#                   response = RESPONSE_PIR_IDLE
                    self.show_color(COLOR_VIOLET)
            elif command == 'pir on':
                self.pir_enabled = True
                self.show_color(COLOR_GREEN)
            elif command == 'pir off':
                self.pir_enabled = False
                self.show_color(COLOR_DARK_GREEN)

            # set some colors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            elif command == 'red':
                self.show_color(COLOR_RED)
            elif command == 'green':
                self.show_color(COLOR_GREEN)
            elif command == 'blue':
                self.show_color(COLOR_BLUE)
            elif command == 'cyan':
                self.show_color(COLOR_CYAN)
            elif command == 'magenta':
                self.show_color(COLOR_MAGENTA)
            elif command == 'yellow':
                self.show_color(COLOR_YELLOW)
            elif command == 'black':
                self.show_color(COLOR_BLACK)

            # start and stop a timer ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            elif command == 'start-timer':
                self.startTimer()
            elif command == 'stop-timer':
                self.stopTimer()

            # asynchronous wait ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            elif command.startswith('wait'):
                self.show_color(COLOR_VIOLET)
                _duration = self._parse_duration(command, default=5)
                self._log.info("waiting for {:.2f} seconds.".format(_duration))
                await asyncio.sleep(_duration)
                self.show_color(COLOR_DARK_VIOLET)
            else:
                # delegate to base class if not processed ┈┈┈┈┈┈┈┈┈┈┈┈
                await super().handle_command(command)

        except Exception as e:
            self._log.error("TinyFxController error: {}".format(e))
            sys.print_exception(e)
            self.show_color(COLOR_RED)
            return RESPONSE_UNKNOWN_ERROR
        finally:
            self._processing_task = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _list_wav_files_without_extension(self, directory="sounds"):
        try:
            files = os.listdir(directory)
            wav_files = [f[:-4] for f in files if f.endswith('.wav')]
            return wav_files
        except OSError as e:
            print("Error accessing directory:", e)
            return []

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _parse_duration(self, arg, default=5):
        try:
            parts = arg.strip().split()
            if len(parts) >= 2:
                return int(parts[1])
        except ValueError:
            pass
        return default

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def validated(self):
#       self._log.debug("validated.")
        if self._timer:
            self.stopTimer()
        super().validated()

    # help ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def help(self):
        print(Fore.CYAN + '''
tinyfx controller commands:

    ch1               unused (channel 1)
    ch2               unused (channel 2)
    ch3               unused (channel 3)
    ch4 | mast        turn on mast LED (channel 4)
    ch5 | stbd        turn on starboard running lights (channel 5)
    ch6 | port        turn on port running lights (channel 6)
    help              prints this help
    enable            enable controller
    disable           disable and exit the controller
    play <name>       play a sound
    on                turn on all the LED channels
    off               turn off all the LED channels
    flash             display flash memory info to console
    mem               display free memory info
    sounds            display list of available sounds
    pir get           get the value of the PIR sensor
    pir on            enable the PIR sensor
    pir off           disable the PIR sensor
    red               set the RGB LED to red
    green             set the RGB LED to green
    blue              set the RGB LED to blue
    black             set the RGB LED to black (off)
    start-timer       start a timer that blinks the LED
    stop-timer        stop the timer
    wait [n]          asynchronously wait n seconds (default 5)

    ''' + Style.RESET_ALL)

    # start and stop a timer ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def startTimer(self):
        if not self._timer:
            self._log.info('timer: ' + Fore.GREEN + 'start')
            self._timer = Timer()
            self._timer.init(period=1000, mode=Timer.PERIODIC, callback=self._toggle_led)
        else:
            self._log.warning('timer already started.')

    def _toggle_led(self, arg):
        self._on = not self._on
        if self._on:
            self.show_color(COLOR_DARK_CYAN)
            utime.sleep_ms(50)
            self.show_color(COLOR_BLACK)
        else:
            pass

    def stopTimer(self):
        if self._timer:
            self._log.info('timer: ' + Fore.GREEN + 'stop')
            self._timer.deinit()
        else:
            self._log.warning('timer already stopped.')
        self._timer = None

#EOF
