#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2024 by Murray Altheim. All rights reserved. This file is part of
# the Robot Operating System project and is released under the "Apache Licence,
# Version 2.0". Please see the LICENSE file included as part of this package.
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
