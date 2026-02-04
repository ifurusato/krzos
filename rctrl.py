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

import time
from threading import Event, Lock, Thread
from colorama import init, Fore, Style
init()

from hardware.radiozoa_controller import RadiozoaController

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

I2C_ADDRESS = 0x47
ALWAYS_DATA_REQUEST = False

REQUEST = "distances"

def worker_loop(master, stop_event, lock):
    """
    Runs until stop_event is set.
    Repeatedly sends a request to the slave.
    """
    print("Worker thread started")

    try:
        while not stop_event.is_set():
            with lock:
                response = master.send_request(REQUEST)
            if response == REQUEST:
                print(Fore.BLACK + "[worker] response: {}".format(response) + Style.RESET_ALL)
            else:
                print(Fore.GREEN + "[worker] response: {}".format(response) + Style.RESET_ALL)
            time.sleep(0.5)
    finally:
        print("worker thread stopping")

def main():

    worker_thread = None
    stop_event = Event()
    i2c_lock = Lock()

    try:
        master = RadiozoaController(i2c_address=I2C_ADDRESS)
        master.enable()
        while True:
            user_msg = input('Enter command string to send ("quit" to exit): ')
            if user_msg.strip().lower() == 'quit':
                break
            print('user msg: {}'.format(user_msg))

            if user_msg.lower() == 'go':
                if worker_thread and worker_thread.is_alive():
                    print("worker already running")
                else:
                    stop_event.clear()
                    worker_thread = Thread(target=worker_loop, args=(master, stop_event, i2c_lock), daemon=True,)
                    worker_thread.start()
                continue

            if user_msg.lower() == 'stop':
                if worker_thread and worker_thread.is_alive():
                    stop_event.set()
                    worker_thread.join()
                else:
                    print("worker not running")
                continue

            if not user_msg:
                continue

            with i2c_lock:
                response = master.send_request(user_msg)
            print('response: {}'.format(response))

    except KeyboardInterrupt:
        print('Ctrl-C caught, exiting…')
    except Exception as e:
        print('error: {}'.format(e))
    finally:
        # clean shutdown
        if worker_thread and worker_thread.is_alive():
            stop_event.set()
            worker_thread.join()

if __name__ == '__main__':
    main()

#EOF
