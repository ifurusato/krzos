#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-04-30
# modified: 2025-05-27
#
# Generates the 'hardware/sound.py' file and './sounds.json metadata' file using
# the './sounds-source.json' JSON file and 'hardware/sounds-template.py' as a
# template. The sounds.json or sound_dictionary.py is then copied to the ESP32 or
# Tiny FX (resp) so that its 'main.py' can use it to determine which sound to play.
#
# See: https://github.com/pimoroni/ioe-python/blob/master/REFERENCE.md#function-reference
#

import time
import datetime as dt
import json

from core.stringbuilder import StringBuilder
from colorama import init, Fore, Style
init(autoreset=True)

from core.util import Util

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Sound:
    def __init__(self, index, name, mnemonic, duration, filename, description):
        self._index    = index
        self._name     = name
        self._mnemonic = mnemonic
        self._duration = duration
        self._filename = filename
        self._description = description

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def index(self):
        return self._index

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def mnemonic(self):
        return self._mnemonic

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def duration(self):
        return self._duration

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def filename(self):
        return self._filename

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def description(self):
        return self._description

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def as_enum(self):
        _offset = Util.repeat(' ', 12 - len(self._mnemonic))
        return "    {}{}  = ( {:2d}, '{}',{} '{}', {}{:2.1f}, '{}',{} '{}')".format(
                self._mnemonic, _offset, self._index, self._name, _offset, self._mnemonic, _offset, self._duration, self._filename, _offset, self._description)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __str__(self):
        return "Sound(index={:d}, name={}, mnemonic={}, duration={:d}, filename: {}, description: '{}')".format(
                self._index, self._name, self._mnemonic, self._duration, self._filename, self._description)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def _get_sound_config(filepath):
        with open(filepath) as json_file:
            _config = json.load(json_file)
            return _config

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def load_sounds(filename):
        '''
        A static method that loads a JSON configuration file, returning a
        list of Sound objects.
        '''
        _sounds = []
        for _item in Sound._get_sound_config(filename):
            _sounds.append(Sound(
                    int(_item['index']),
                    _item['name'],
                    _item['mnemonic'],
                    int(_item['duration']),
                    _item['filename'],
                    _item['description']))
        return _sounds

# generator ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_enum_defs = StringBuilder()
_order = StringBuilder()
_sounds = Sound.load_sounds('sounds-source.json')

for _sound in _sounds:
    _enum = _sound.as_enum()
    _enum_defs.append(_enum + '\n')
    _order.append(_sound.mnemonic)
    _order.append(' ')

def generate_dictionary():
    '''
    Generate the dictionary used in MicroPython to map between
    sound names and their corresponding filenames.
    '''
    _sounds = Sound.load_sounds('sounds-source.json')
    _sb = StringBuilder()
    _sb.append('# dictionary of sounds ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈\n\n')
    _sb.append('_sounds = [\n')
    for _sound in _sounds:
        _name = _sound.name
        _filename = _sound.filename
        _sb.append('    (\'')
        _sb.append(_sound.name)
        _sb.append('\',')
        _sb.append(' ' * (16 - len(_name)))
        _sb.append('\'')
        _sb.append(_sound.filename)
        _sb.append('\'),\n')
    _sb.append(']\n')
    return _sb.to_string()

print('generating: sound_dictionary.py')
_output = generate_dictionary()
with open('tinyfx/sound_dictionary.py', 'w') as _output_file:
    _output_file.write(_output)
    print('wrote file: tinyfx/sound_dictionary.py')

with open('hardware/sound-template.py', 'r') as _template_file:
    _template = _template_file.read()
    _timestamp = dt.datetime.now().strftime('%Y-%m-%d')
    _template = _template.replace('%TIMESTAMP',_timestamp)
    _template = _template.replace('%ORDER',_order.to_string())
    _template = _template.replace('%ENUM',_enum_defs.to_string())

with open('hardware/sound.py', 'w') as _output_file:
    _output_file.write(_template)
    print('wrote file: hardware/sound.py')

print('generating: sounds.json')
from hardware.sound import Sound
Sound.export()

print("export complete, now copy 'sounds.json' to the ESP32, or 'sound_dictionary.py' to the Tiny FX.")

#EOF
