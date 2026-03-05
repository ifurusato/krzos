#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from i2c_master import I2CMaster

# ..........................................

retries     = 0
max_retries = 5
DISTANCES_COMMAND = "distances"

master = I2CMaster(timeset=False)
master.enable()
print('sending distances…')
response = master.send_request(DISTANCES_COMMAND)
while response is None \
        or response == DISTANCES_COMMAND \
        or response == 'ACK' \
        or response == 'ERR':
    if retries >= max_retries:
        print('max retries ({}) exceeded: no valid distances response.'.format(max_retries))
        break
#   print('invalid response ({}), retrying…'.format(response))
    time.sleep(0.008)
    response = master.send_request(DISTANCES_COMMAND)
    retries += 1

print('final response: {!r} after {} tries.'.format(response, retries + 1))
master.disable()
