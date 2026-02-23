#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License.
#
# author:   Ichiro Furusato
# created:  2026-02-17
# modified: 2026-02-18
#
# A collection of exceptions.

class TimeoutError(RuntimeError):
    pass

class IllegalStateError(RuntimeError):
    pass

#EOF
