#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-07-16
# modified: 2025-07-16

from logger import Logger, Level

class Display:
    
    def __init__(self):
        self._log = Logger('mock_display', Level.INFO)
        self._log.info('ready.')
    
    def hello(self):
        pass
    
    def console(self, args):
        pass

    def ip_address(self, args):
        pass

    def ready(self):
        pass

    def close(self):
        pass

#EOF
