#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-07-14
# modified: 2025-07-14

import socket

def get_ip_address(as_tuple=False):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80)) # Google's DNS (no data sent)
        ip = s.getsockname()[0]
        s.close()
        if as_tuple:
            return tuple(int(part) for part in ip.split('.'))
        else:
            return ip
    except Exception as e:
        return "{} raised getting IP address: {}".format(type(e), e)

#EOF
