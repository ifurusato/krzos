#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-14
# modified: 2025-12-30

class Color:
    _registry = []

    def __init__(self, name, rgb):
        self.name = name
        self.rgb = rgb
        Color._registry.append(self)

    def __getitem__(self, index):
        return self.rgb[index]

    def __iter__(self):
        return iter(self.rgb)

    def __len__(self):
        return len(self.rgb)

    def __eq__(self, other):
        if isinstance(other, Color):
            return self.rgb == other.rgb
        return self.rgb == other

    def __hash__(self):
        return hash(self.rgb)

    def __repr__(self):
        return '{} {}'.format(self.name, self.rgb)

    @classmethod
    def all_colors(cls):
        return cls._registry

    @classmethod
    def get(cls, name: str):
        '''
        Return a Color whose name matches the key.
        '''
        key = name.lower().replace("_", " ")
        for c in cls._registry:
            # normalize stored name: strip leading "COLOR_" and lowercase
            norm = c.name.lower().replace("color_", "").replace("_", " ")
            if norm == key:
                return c
        return None

COLOR_BLACK         = Color("COLOR_BLACK",        (  0,   0,   0))
COLOR_WHITE         = Color("COLOR_WHITE",        (255, 255, 255))
COLOR_RED           = Color("COLOR_RED",          (255,   0,   0))
COLOR_GREEN         = Color("COLOR_GREEN",        (  0, 255,   0))
COLOR_BLUE          = Color("COLOR_BLUE",         (  0,   0, 255))
COLOR_CYAN          = Color("COLOR_CYAN",         (  0, 255, 255))
COLOR_MAGENTA       = Color("COLOR_MAGENTA",      (255,   0, 255))
COLOR_YELLOW        = Color("COLOR_YELLOW",       (250, 150,   0))

COLOR_DARK_RED      = Color("COLOR_DARK_RED",     ( 32,   0,   0))
COLOR_DARK_GREEN    = Color("COLOR_DARK_GREEN",   (  0,  32,   0))
COLOR_DARK_BLUE     = Color("COLOR_DARK_BLUE",    (  0,   0,  32))
COLOR_DARK_CYAN     = Color("COLOR_DARK_CYAN",    (  0,  32,  32))
COLOR_DARK_MAGENTA  = Color("COLOR_DARK_MAGENTA", ( 32,   0,  32))
COLOR_DARK_YELLOW   = Color("COLOR_DARK_YELLOW",  ( 64,  32,   0))
COLOR_DARK_GREY     = Color("COLOR_DARK_GREY",    ( 10,  10,  10))

COLOR_APPLE         = Color("COLOR_APPLE",        ( 70, 100,   0))
COLOR_ORANGE        = Color("COLOR_ORANGE",       (240,  53,   0))
COLOR_TANGERINE     = Color("COLOR_TANGERINE",    (100,  11,   0))
COLOR_CORNFLOWER    = Color("COLOR_CORNFLOWER",   ( 60,  90, 142))
COLOR_INDIGO        = Color("COLOR_INDIGO",       (  0,  75, 130))
COLOR_VIOLET        = Color("COLOR_VIOLET",       (138,  43, 226))
COLOR_DEEP_CYAN     = Color("COLOR_DARK_CYAN",    (  0,  11,  11))
COLOR_PINK          = Color("COLOR_PINK",         (255,  50,  40))
COLOR_FUCHSIA       = Color("COLOR_FUCHSIA",      (158,  16,  99))
COLOR_PURPLE        = Color("COLOR_PURPLE",       ( 14,   0,  56))
COLOR_AMBER         = Color("COLOR_AMBER",        (255,  90,   0))
COLOR_SKY_BLUE      = Color("COLOR_SKY_BLUE",     (  9,  25, 190))

#EOF
