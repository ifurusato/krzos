#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-08-04
# modified: 2025-08-04

class TooManyErrors(Exception):
    '''
    An exception thrown when too many errors have occurred.
    '''
    pass

#EOF
