#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2020-04-15
# modified: 2024-06-19

from core.logger import Level, Logger

class ConfigurationError(Exception):
    '''
    This exception is thrown when any error due to configuration occurs.
    '''
    def __init__(self, message):
        super().__init__(message)

#EOF
