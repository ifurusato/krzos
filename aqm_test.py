#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the K-Series Robot Operating System (KROS) project, released under the MIT
# License. Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2026-02-02
# modified: 2026-02-04

from core.logger import Logger, Level
from hardware.aqm import AirQualityMonitor

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():
    _log = Logger('test', level=Level.INFO)
    try:
        _aqm = AirQualityMonitor()
        _aqm.enable()
        _aqm.calibrate()
        _aqm.monitor()
    except KeyboardInterrupt:
        _log.info('Ctrl-C caught; exiting…')
    except Exception as e:
        _log.error('{} raised in monitor loop: {}.'.format(type(e), e))

if __name__== "__main__":
    main()

#EOF
