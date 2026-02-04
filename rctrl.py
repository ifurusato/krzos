#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-02-04

from hardware.radiozoa_controller import RadiozoaController

I2C_ADDRESS = 0x47
ALWAYS_DATA_REQUEST = False

def main():
    try:
        master = RadiozoaController(i2c_address=I2C_ADDRESS)
        master.enable()
        while True:
            user_msg = input('Enter command string to send ("quit" to exit): ')
            if user_msg.strip().lower() == 'quit':
                break
            print('user msg: {}'.format(user_msg))
            if len(user_msg) == 0:
                continue
            response = master.send_request(user_msg)
            print('response: {}'.format(response))

    except KeyboardInterrupt:
        print('Ctrl-C caught, exiting…')
    except Exception as e:
        print('error: {}'.format(e))

if __name__ == '__main__':
    main()

#EOF
