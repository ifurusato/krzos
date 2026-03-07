#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License.
#
# author:   Ichiro Furusato
# created:  2026-02-09
# modified: 2026-03-08

import sys
import time
import math, random
from colorama import Fore, Style

from tiny_fx import TinyFX
from manual_player import ManualPlayer
from settable import SettableFX
from settable_blink import SettableBlinkFX
from pir import PassiveInfrared

from logger import Logger, Level
from colors import *
from controller import Controller
from timestamp import TimeStamp
from wav_util import wav_duration

class TinyFxController(Controller):
    '''
    A TinyFX controller for command strings received from the I2CSlave.

    Commands include:
      play [sound-name]     play a sound
      ch[1-6] on|off        control channels
      all on|off            turn all channels on or off (including RGB LED)
      heartbeat on|off      blinking RGB LED
      color [name]          set RGB LED to color name (see colors.py)

    Setting the heartbeat or color will disable the other.
    PIR sensor functionality currently has not been tested.
    '''
    def __init__(self, config=None, level=Level.INFO):
        self._log = Logger('tinyfx-ctrl', level=level)
        super().__init__(config)
#       self._slave = None
        self._tinyfx  = TinyFX(init_wav=True, wav_root='/sounds')
        self._rgbled  = self._tinyfx.rgb
        self._intra_play_delay_ms = 200 # ms: additional time between plays
        self._playing = False
        # channel definitions
        blink_channels = [True, False, False, True, False, False] # channel 1 and 4 blinks
        self._channel1_fx = self._get_channel(1, blink_channels[0])
        self._channel2_fx = self._get_channel(2, blink_channels[1])
        self._channel3_fx = self._get_channel(3, blink_channels[2])
        self._channel4_fx = self._get_channel(4, blink_channels[3])
        self._channel5_fx = self._get_channel(5, blink_channels[4])
        self._channel6_fx = self._get_channel(6, blink_channels[5])
        # set up the effects to play
        self._player = ManualPlayer(self._tinyfx.outputs)
        self._player.effects = [
            self._channel1_fx,
            self._channel2_fx,
            self._channel3_fx,
            self._channel4_fx,
            self._channel5_fx,
            self._channel6_fx
        ]
        # name map of channels (you can add aliases here)
        self._channel_map = {
            'ch1': self._channel1_fx,
            'ch2': self._channel2_fx,
            'ch3': self._channel3_fx,
            'ch4': self._channel4_fx,
            'ch5': self._channel5_fx,
            'ch6': self._channel6_fx,
            # aliases
            'back': self._channel1_fx,
            'head': self._channel2_fx,
            'dome': self._channel3_fx,
            'mast': self._channel4_fx,
            'stbd': self._channel5_fx,
            'port': self._channel6_fx
        }
        # heartbeat blink feature
        self._heartbeat_enabled     = True
        self._heartbeat_on_time_ms  = 50
        self._heartbeat_off_time_ms = 2950
        self._heartbeat_timer = 0
        self._heartbeat_state = False
        # PIR sensor and polling timer
        self._timestamp       = TimeStamp()
        self._pir_sensor      = PassiveInfrared()
        # we'll rely on the heartbeat instead
#       self._pir_timer = Timer()
#       self._pir_timer.init(period=1000, mode=Timer.PERIODIC, callback=self._poll_pir)
        self._play('arming-tone')
        self._log.info('ready.')
        # ready

    def _start_services(self):
        '''
        This method is called upon startup, after a preset delay. It can be overridden
        as necessary to start any application-level services.
        '''
        super()._start_services()

    def _led_off(self, timer=None):
        super()._led_off(timer)

    def tick(self, delta_ms):
        self._player.update(delta_ms)
        super().tick(delta_ms)

    def _poll_pir(self):
        if self._pir_sensor.triggered:
            self._timestamp.mark()
            print('triggered.')

    def _get_channel(self, channel, blinking=False):
        '''
        The channel argument is included in case you want to customise what
        is returned per channel, e.g., channel 1 below.
        '''
        if blinking:
            if channel == 1:
                # we use a more 'irrational' speed so that the two blinking channels almost never synchronise
                return SettableBlinkFX(speed=0.66723, phase=0.0, duty=0.25)
            else:
                return SettableBlinkFX(speed=0.5, phase=0.0, duty=0.015)
        else:
            return SettableFX(brightness=0.8)

    def _get_pir(self):
        '''
        Returns True if the PIR sensor has been triggered.
        '''
        if self._timestamp.marked:
            print("raw: '{}'".format(self._timestamp.raw()))             # 8-tuple
            print("iso: '{}'".format(self._timestamp.iso()))             # ISO string like "2025-01-01T12:56:55"
            print("'{}'sec elapsed.".format(self._timestamp.elapsed()))  # seconds since last trigger
            return str(int(self._timestamp.elapsed()))
        else:
            return "-1"

    def _parse_repeat(self, value):
        '''
        Parse a third argument of the form '10x'.
        Returns an int if valid, otherwise 1.
        '''
        if value and value.endswith('x'):
            try:
                return int(value[:-1])
            except ValueError:
                return 1
        return 1

    def _play(self, cmd, repeat=1):
        print("playing sound for command: {}".format(cmd))
        try:
            self._playing = True
            parts = cmd.split()
            if len(parts) < 2:
                sound_name = cmd
            else:
                sound_name = parts[1]
            file_name = '{}.wav'.format(sound_name)
            # compute duration once per play() invocation
            duration_ms = int(wav_duration("sounds/{}".format(file_name)) * 1000)
            print('duration: {}ms'.format(duration_ms))
            for i in range(repeat):
                if repeat == 1:
                    print('playing: {}…'.format(sound_name))
                else:
                    print('playing: {}… ({}/{})'.format(sound_name, (i+1), repeat))
                self._tinyfx.wav.play_wav(file_name)
                if repeat > 1:
                    print('waiting {}ms…'.format(duration_ms + self._intra_play_delay_ms))
                    time.sleep_ms(duration_ms)
                    time.sleep_ms(self._intra_play_delay_ms)
        finally:
            self._playing = False

    def _show_color(self, cmd):
        parts = cmd.split()
        if len(parts) < 2:
            print("ERROR: show color command missing color name.")
            return
        color_name = parts[1]
        color = Color.get(color_name)
        if color:
            print('showing color: {}…'.format(color.name))
            self._rgbled.set_rgb(*color)
        else:
            print("ERROR: unknown color name: {}".format(color_name))

    def pre_process(self, cmd, arg0, arg1, arg2, arg3, arg4):
        '''
        Pre-process the arguments, returning a response and color if a match occurs.
        Such a match precludes further processing.
        '''
#       self._log.info("ring: pre-process command '{}' with arg0: '{}'; arg1: '{}'; arg2: '{}'; arg3: '{}'; arg4: '{}'".format(cmd, arg0, arg1, arg2, arg3, arg4))

        if arg0 == 'all' or arg0 in self._channel_map and arg1 is not None:
            print('action: {}'.format(arg1))
            if arg0 == 'all':
                if arg1 == 'on':
                    for fx in self._player.effects:
                        fx.set(True)
                    self._heartbeat_enabled = True
                elif arg1 == 'off':
                    for fx in self._player.effects:
                        fx.set(False)
                    self._heartbeat_enabled = False
                    self._show_color('color black')
                else:
                    return Controller._PACKED_ERR, COLOR_RED
            else:
                fx = self._channel_map[arg0]
                if arg1 == 'on':
                    fx.set(True)
                elif arg1 == 'off':
                    fx.set(False)
                else:
                    return Controller._PACKED_ERR, COLOR_RED
            return Controller._PACKED_ACK, COLOR_DARK_GREEN

        elif arg0 == "run":
            fxs = [ self._channel_map['ch4'], self._channel_map['ch5'], self._channel_map['ch6'] ]
            for fx in fxs:
                if arg1 == 'on':
                    fx.set(True)
                elif arg1 == 'off':
                    fx.set(False)
                else:
                    return Controller._PACKED_ERR, COLOR_RED
            return Controller._PACKED_ACK, COLOR_DARK_GREEN

        elif arg0 == "color":
            self._heartbeat_enabled = False
            self._show_color(cmd)
            return Controller._PACKED_ACK, COLOR_DARK_GREEN

        elif arg0 == "play":
            _repeat  = self._parse_repeat(arg2)
            self._play(arg1, _repeat)
            return Controller._PACKED_ACK, COLOR_DARK_GREEN

        elif arg0 == "pir":
            return self._get_pir(), COLOR_DARK_GREEN

        else:
            return super().pre_process(cmd, arg0, arg1, arg2, arg3, arg4)

    def process(self, cmd):
        '''
        Processes the callback from the I2C slave.
        '''
        return super().process(cmd)

#EOF
