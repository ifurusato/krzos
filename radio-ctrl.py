#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-03-04
#
# CLI for the RadiozoaController. Supports manual command sending, button-triggered
# scans, a polling loop ("go"/"stop"), and occupancy map load/save/viz.

import os
import sys
import select
import argparse
from threading import Lock
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.radiozoa_controller import RadiozoaController

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

USE_BUTTON = False

def read_xyz():
    try:
        print(Fore.WHITE + "Enter x,y,heading: " + Style.RESET_ALL, end="", flush=True)
        raw = input().strip()
        parts = raw.split(",")
        if len(parts) == 2:
            # backwards compatible: x,y without heading
            x_str, y_str = parts
            return int(x_str.strip()), int(y_str.strip()), 0
        elif len(parts) == 3:
            x_str, y_str, h_str = parts
            return int(x_str.strip()), int(y_str.strip()), int(h_str.strip())
        else:
            return None
    except (ValueError, TypeError):
        return None

def write_map_data(map_data, map_data_path):
    with open(map_data_path, "w") as file:
        for row in map_data:
            first_part = "{} {} {}".format(row[0], row[1], row[2])
            first_part = "{:<16}".format(first_part) # 16 chars wide, matches sample
            second_part = " ".join("{:04d}".format(n) for n in row[3:])
            file.write(first_part + second_part + "\n")
            file.flush()
    print(Fore.CYAN + "saved map to file: {}".format(map_data_path) + Style.RESET_ALL)

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def main():

    button         = None
    button_pressed = False
    map_data       = []

    cwd            = os.getcwd()
    map_data_path  = os.path.join(cwd, "map_data.txt")
    i2c_lock       = Lock()
    prompt         = Fore.WHITE + Style.BRIGHT + "► " + Style.RESET_ALL
    controller     = None

    try:

        parser = argparse.ArgumentParser(description='RadiozoaController CLI')
        parser.add_argument('--address',
                type=lambda x: int(x, 0), default=None,
                help='I2C device address (default from config)')
        args = parser.parse_args()

        _config = ConfigLoader(Level.INFO).configure()
        controller = RadiozoaController(config=_config)
        controller.enable()

        def button_handler():
            nonlocal button_pressed
            print(Fore.GREEN + "button released…" + Style.RESET_ALL)
            button_pressed = True

        if USE_BUTTON:
            from hardware.button import Button
            button = Button(config=_config, callback=button_handler)

        print('\nEnter command string to send (Ctrl-C or "exit" to exit):')
        last_user_msg = None
        print(prompt, end='', flush=True)

        while True:
            if button_pressed:
                button_pressed = False
                print() # newline after prompt
                xyz = read_xyz()
                if xyz is None:
                    xyz = 0, 0, 0
                distances = controller.perform_scan(xyz)
                if distances is not None:
                    row = (*xyz, *distances)
                    map_data.append(row)
                    print("map: '{}'".format(map_data))
                print(prompt, end='', flush=True)
                continue

            # check if input is available with timeout
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue

            user_msg = sys.stdin.readline().strip()

            if not user_msg:
                print(prompt, end='', flush=True)
                continue

            elif last_user_msg is not None and user_msg == 'r':
                # repeat last command
                user_msg = last_user_msg

            elif user_msg == 'exit' or user_msg == 'quit':
                break

            elif user_msg == 'go':
                print(Fore.WHITE + '\nStarting poll loop (type "stop" to exit)…\n' + Style.RESET_ALL)
                controller.start_polling()
                print(prompt, end='', flush=True)
                continue

            elif user_msg == 'stop':
                controller.stop_polling()
                print(prompt, end='', flush=True)
                continue

            elif user_msg == 'save':
                if len(map_data) > 0:
                    write_map_data(map_data, map_data_path)
                else:
                    print("empty map.")
                print(prompt, end='', flush=True)
                continue

            elif user_msg == 'viz':
                controller.generate_svg()
                print(prompt, end='', flush=True)
                continue

            elif user_msg.startswith('load '):
                filename = user_msg[5:].strip()
                if not filename:
                    print("usage: load <filename>")
                else:
                    controller.load_scan_data(filename)
                print(prompt, end='', flush=True)
                continue

            print(Fore.CYAN + 'user msg: ' + Fore.WHITE + "'{}'".format(user_msg))
            with i2c_lock:
                response = controller.send_request(user_msg)
                if response == user_msg:
                    print(Fore.YELLOW + 'response matches input.' + Style.RESET_ALL)

            if response is None:
                print(Fore.CYAN + Style.DIM + "response: " + Fore.BLACK + 'None' + Style.RESET_ALL)
            elif response == user_msg:
                print(Fore.CYAN + Style.DIM + "response: " + Fore.BLACK + '{}'.format(response) + Style.RESET_ALL)
            else:
                print(Fore.CYAN + 'response: ' + Fore.GREEN + '{}'.format(response) + Style.RESET_ALL)
            last_user_msg = user_msg
            print(prompt, end='', flush=True)

    except OSError:
        print('unable to connect with device.')
    except KeyboardInterrupt:
        print('Ctrl-C caught, exiting…')
    except Exception as e:
        print('error: {}'.format(e))
    finally:
        if controller is not None:
            controller.close()


if __name__ == '__main__':
    main()

#EOF
