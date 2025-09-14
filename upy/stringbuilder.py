#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-10-15
# modified: 2025-06-26

from io import StringIO

class StringBuilder():
    '''
    Mimics Java's StringBuilder class, because we miss it.
    '''
    def __init__(self, init_obj=None, indent=0, delim=None):
        self._buffer = StringIO()
        self._delim = delim
        self._indent = None
        if init_obj:
            self.append(init_obj)
        if indent > 0:
            self._indent = ' ' * indent

    def append(self, obj, indent=None, delim=None):
        '''
        Append the object to the buffer. This accepts any object.
        '''
        if obj == None:
            raise TypeError('null argument')
        if indent:
            self._buffer.write(' ' * indent)
        elif self._indent:
            self._buffer.write(self._indent)
        if isinstance(obj, str):
            self._buffer.write(obj)
        else:
            self._buffer.write(str(obj))
        if delim or delim == '':
            self._buffer.write(delim)
        elif self._delim:
            self._buffer.write(self._delim)

    def length(self):
        return len(self._buffer.getvalue())

    def __str__(self):
        return self._buffer.getvalue()

    def to_string(self):
        return self._buffer.getvalue()

#EOF
