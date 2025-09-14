#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-30
# modified: 2025-06-30
#
# Changes the current working directory to /flash (necessary when an SD card is present)

try:
    import os

    if os.getcwd() != '/flash':
        os.chdir('/flash')
except Exception as e:
    print("WARNING: Failed to change directory to /flash:", e)

