#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2024-08-14
# modified: 2026-02-08

class Color:
    _registry = []

    def __init__(self, name, rgb):
        self._name = name
        # normalize stored name: strip leading "COLOR_" and lowercase
        self._norm = name.lower().replace("color_", "").replace("_", " ")
        self._rgb = rgb
        Color._registry.append(self)

    @property
    def name(self):
        '''
        Return the normalised, lowercase color name.
        '''
        return self._norm

    @property
    def rgb(self):
        return self._rgb

    def __getitem__(self, index):
        return self._rgb[index]

    def __iter__(self):
        return iter(self._rgb)

    def __len__(self):
        return len(self._rgb)

    def __eq__(self, other):
        if isinstance(other, Color):
            return self._rgb == other._rgb
        return self._rgb == other

    def __hash__(self):
        return hash(self._rgb)

    def __repr__(self):
        return '{} {}'.format(self._name, self._rgb)

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
            norm = c._name.lower().replace("color_", "").replace("_", " ")
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
COLOR_GREY          = Color("COLOR_GREY",         ( 40,  40,  40))

COLOR_DARK_RED      = Color("COLOR_DARK_RED",     ( 22,   0,   0))
COLOR_DARK_GREEN    = Color("COLOR_DARK_GREEN",   (  0,  24,   0))
COLOR_DARK_BLUE     = Color("COLOR_DARK_BLUE",    (  0,   0,  32))
COLOR_DARK_CYAN     = Color("COLOR_DARK_CYAN",    (  0,  28,  28))
COLOR_DARK_MAGENTA  = Color("COLOR_DARK_MAGENTA", ( 28,   0,  28))
COLOR_DARK_YELLOW   = Color("COLOR_DARK_YELLOW",  ( 50,  20,   0))
COLOR_DARK_GREY     = Color("COLOR_DARK_GREY",    ( 10,  10,  10))

COLOR_AMBER         = Color("COLOR_AMBER",        (255,  90,   0))
COLOR_ORANGE        = Color("COLOR_ORANGE",       (240,  53,   0))
COLOR_TANGERINE     = Color("COLOR_TANGERINE",    (100,  11,   0))
COLOR_FUCHSIA       = Color("COLOR_FUCHSIA",      (158,  16,  99))
COLOR_APPLE         = Color("COLOR_APPLE",        ( 70, 100,   0))
COLOR_EMERALD       = Color("COLOR_EMERALD",      (  0,  90,  10))
COLOR_MEDIUM_CYAN   = Color("COLOR_MEDIUM_CYAN",  (  0, 128, 128))
COLOR_DEEP_CYAN     = Color("COLOR_DEEP_CYAN",    (  0,  11,  11))
COLOR_CORNFLOWER    = Color("COLOR_CORNFLOWER",   ( 60,  90, 142))
COLOR_SKY_BLUE      = Color("COLOR_SKY_BLUE",     (  9,  25, 190))
COLOR_INDIGO        = Color("COLOR_INDIGO",       (  0,  16,  50))
COLOR_LAVENDER      = Color("COLOR_LAVENDER",     ( 24,  11, 130))
COLOR_VIOLET        = Color("COLOR_VIOLET",       (138,  43, 226))
COLOR_PURPLE        = Color("COLOR_PURPLE",       ( 14,   0,  56))
COLOR_PINK          = Color("COLOR_PINK",         (255,  50,  40))

COLOR_GREY_0        = Color("COLOR_GREY_0",       ( 11,  11,  10))
COLOR_GREY_1        = Color("COLOR_GREY_1",       ( 15,  15,  10))
COLOR_GREY_2        = Color("COLOR_GREY_2",       ( 20,  20,  15))
COLOR_GREY_3        = Color("COLOR_GREY_3",       ( 30,  30,  20))
COLOR_GREY_4        = Color("COLOR_GREY_4",       ( 40,  40,  30))
COLOR_GREY_5        = Color("COLOR_GREY_5",       ( 50,  50,  40))
COLOR_GREY_6        = Color("COLOR_GREY_6",       ( 70,  70,  60))
COLOR_GREY_7        = Color("COLOR_GREY_7",       (130, 130, 110))

#EOF
