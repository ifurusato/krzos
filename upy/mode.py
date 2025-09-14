#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-07-04
# modified: 2025-07-14

from colorama import Fore, Style
try:
    from colorama import init
    init()
except Exception:
    pass # not in MicroPython

class Mode:
    _instances = []

    def __init__(self, index, name, code, speeds, description):
        self._index = index
        self._name = name
        self._code = code
        self._speeds = speeds
        self._description = description
        Mode._instances.append(self)

    @property
    def index(self):
        return self._index

    @property
    def name(self):
        return self._name

    @property
    def code(self):
        return self._code

    @property
    def has_speeds(self):
        return self._speeds is not None

    @property
    def speeds(self):
        return self._speeds

    @property
    def description(self):
        return self._description

    def __str__(self):
        return self._name

    def __repr__(self):
        return "Mode({}, {}, {})".format(self._name, self._code, self._description)

    @classmethod
    def all(cls):
        return cls._instances

    @classmethod
    def from_index(cls, index):
        for inst in cls._instances:
            if inst.index == index:
                return inst
        raise ValueError("no Mode with index '{}'".format(index))

    @classmethod
    def from_code(cls, code):
        for inst in cls._instances:
            if inst.code == code:
                return inst
        raise ValueError("no Mode with code '{}'".format(code))

# non-movement modes
Mode.COLOR       = Mode( 0, "COLOR",     "CO", None,              "show color")
# stopped
Mode.STOP        = Mode( 1, "STOP",      "ST", ( 0,  0,  0,  0),  "stop")
# all wheels forward/backward
Mode.GO          = Mode( 2, "GO",        "GO", ( 1,  1 , 1 , 1),  "go forward or reverse")
# rotation (spin in place)
Mode.ROT_CW      = Mode( 3, "ROT_CW",    "RO", (-1,  1, -1,  1),  "rotate clockwise")
Mode.ROT_CCW     = Mode( 4, "ROT_CCW",   "RC", ( 1, -1,  1, -1),  "rotate counter-clockwise")
# crab movement (lateral strafe)
Mode.CRAB_PORT   = Mode( 5, "CRAB_PORT", "CP", (-1,  1,  1, -1),  "crab to port")
Mode.CRAB_STBD   = Mode( 6, "CRAB_STBD", "CS", ( 1, -1, -1,  1),  "crab to starboard")
# diagonal movement
Mode.DIA_PFWD    = Mode( 7, "DIA_PFWD",  "DP", ( 0,  1,  1,  0),  "diagonal forward to port")
Mode.DIA_SFWD    = Mode( 8, "DIA_SFWD",  "DS", ( 1,  0,  0,  1),  "diagonal forward to starboard")
Mode.DIA_PREV    = Mode( 9, "DIA_PREV",  "DQ", (-1,  0,  0, -1),  "diagonal reverse to port")
Mode.DIA_SREV    = Mode(10, "DIA_SREV",  "DT", ( 0, -1, -1,  0),  "diagonal reverse to starboard")
# miscellany
Mode.PING        = Mode(11, "PING",      "PN", None,              "ping")
Mode.IP_ADDRESS  = Mode(12, "IP",        "IP", None,              "ip address")
Mode.REQUEST     = Mode(13, "REQUEST",   "RS", None,              "request status")
Mode.ACK         = Mode(14, "ACK",       "AK", None,              "acknowledge")
Mode.ENABLE      = Mode(15, "ENABLE",    "EN", None,              "enable")
Mode.DISABLE     = Mode(16, "DISABLE",   "DI", None,              "disable")
Mode.ERROR       = Mode(17, "ERROR",     "ER", None,              "error")

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():
    # Print a specific mode's fields
    print(Fore.CYAN + "CRAB_PORT:")
    print(Fore.CYAN + "  Name        : " + Fore.WHITE + "{}".format(CRAB_PORT.name) + Style.RESET_ALL)
    print(Fore.CYAN + "  Code        : " + Fore.WHITE + "{}".format(CRAB_PORT.code) + Style.RESET_ALL)
    print(Fore.CYAN + "  Description : " + Fore.WHITE + "{}".format(CRAB_PORT.description) + Style.RESET_ALL)

    # Lookup by code
    code = "RS"
    mode = Mode.from_code(code)
    print(Fore.CYAN + "\nFrom code '{}':".format(code) + Style.RESET_ALL)
    print(Fore.WHITE + "    →  {}".format(mode) + Style.RESET_ALL)

    print()

    # List all available mode names
    print(Fore.CYAN + "All mode names:" + Style.RESET_ALL)
    for m in Mode.all():
        pad = 30 - len(m.description)
        if m.speeds is None:
            print(Fore.CYAN  + " {:>10} ".format(m.name)
                + Fore.WHITE + "    {}".format(m.description)
                + Fore.GREEN + "    {} NA".format(' ' * pad)
                + Style.RESET_ALL)
        else:
            print(Fore.CYAN  + " {:>10} ".format(m.name)
                + Fore.WHITE + "    {}".format(m.description)
                + Fore.GREEN + "    {}{:5.1f}{:5.1f}{:5.1f}{:5.1f}".format(' ' * pad, *m.speeds)
                + Style.RESET_ALL)

if __name__ == "__main__":
    main()

#EOF
