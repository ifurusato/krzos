#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2021-09-03
# modified: 2025-11-14

import sys, re
#import logging
from logging import FileHandler

class AnsiFilteringFileHandler(FileHandler):
    '''
    Extends FileHandler to filter out ANSI character sequences from
    the emitted output.
    '''
    def __init__(self, filename, mode='w', encoding=None, delay=False):
        FileHandler.__init__(self, filename=filename, mode=mode, encoding=encoding, delay=delay)

    def emit(self, record):
        try:
            if self.stream is None:
                self.stream = self._open()
            msg = self.format(record)
            filtered_msg = self._escape_ansi(msg)
            self.stream.write(filtered_msg + self.terminator)
            self.flush()
        except Exception:
            self.handleError(record)

    def _escape_ansi(self, line):
        ansi_escape = re.compile(r'(\x9B|\x1B\[)[0-?]*[ -\/]*[@-~]')
        return ansi_escape.sub('', line)

#EOF
