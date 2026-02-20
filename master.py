#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-02-13
#
# I2C master controller, with CLI option to set I2C address. Permits repeat
# sending of a command using a worker thread loop initiated by "go" and halted
# by "stop".

import time
import sys
import select
import argparse
from threading import Event, Lock, Thread
from colorama import init, Fore, Style
init()

from i2c_master import I2CMaster
from hardware.tinyfx_controller import TinyFxController
from adaptive_occupancy_map import AdaptiveOccupancyMap, MapVisualizer

# pushbutton support ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

import RPi.GPIO as GPIO

class InterruptButton:
    def __init__(self, pin=21, debounce_ms=50, callback=None):
        self._pin = pin
        self._callback = callback
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(
            self._pin,
            GPIO.BOTH,
            callback=self._handle_edge,
            bouncetime=debounce_ms
        )

    def _handle_edge(self, channel):
        state = GPIO.input(self._pin)
        pressed = (state == GPIO.LOW)
        if self._callback:
            self._callback(pressed)

    def cleanup(self):
        GPIO.remove_event_detect(self._pin)
        GPIO.cleanup(self._pin)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

I2C_ID            = 0            # I2C bus identifier
I2C_ADDRESS       = 0x47         # default I2C device address
WORKER_DELAY_SEC  = 1.0          # time between automatic polls
WORKER_REQUEST    = "distances"  # poll command
BUTTON_DELAY_SEC  = 5            # how many seconds after pushbutton to do scan
TICK_COMMAND      = 'play tick'
BEEP_COMMAND      = 'play beep'
DISTANCES_COMMAND = 'distances'
TRIM_OCCUPANCY_MAP_TO_DATA = True

def worker_loop(master, stop_event, lock):
    '''
    Runs until stop_event is set.
    Repeatedly sends a request to the slave.
    '''
    print("worker thread started")

    try:
        while not stop_event.is_set():
            with lock:
                response = master.send_request(WORKER_REQUEST)
            if response == WORKER_REQUEST:
                print("response: {}".format(response))
            else:
                print("response: {}".format(response))
            time.sleep(WORKER_DELAY_SEC)
    finally:
        print("worker thread stopping")

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

def read_xy():
    try:
        print(Fore.WHITE + "Enter x,y: " + Style.RESET_ALL, end="", flush=True)
        raw = input().strip()
        x_str, y_str = raw.split(",", 1)
        return int(x_str.strip()), int(y_str.strip())
    except (ValueError, TypeError):
        return None

def load_scan_data(filename):
    '''
    Load and process scan data from a text file.
    each line: x y heading distance1 distance2 ... distance8
    '''
    global occupancy_map
    
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
        scan_count = 0
        for line in lines:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) != 11:
                print('skipping invalid line: {}'.format(line))
                continue
            x = int(parts[0])
            y = int(parts[1])
            heading = int(parts[2])
            distances = ' '.join(parts[3:11])
            # initialize map on first scan
            if occupancy_map is None:
                occupancy_map = AdaptiveOccupancyMap(min_cell_size=50, initial_size=8000)
                print(Fore.MAGENTA + 'adaptive occupancy map initialized' + Style.RESET_ALL)
            # add sensor reading to map
            occupancy_map.process_sensor_reading(distances, x, y, heading)
            scan_count += 1
            print(Fore.CYAN + 'loaded scan {}: '.format(scan_count) + Fore.YELLOW + '({},{},{}°) '.format(x, y, heading) + Fore.GREEN + '{}'.format(distances) + Style.RESET_ALL)
        print(Fore.MAGENTA + '{} scans loaded from {}'.format(scan_count, filename) + Style.RESET_ALL)
        return True
        
    except FileNotFoundError:
        print(Fore.RED + 'file not found: {}'.format(filename) + Style.RESET_ALL)
        return False
    except Exception as e:
        print(Fore.RED + 'ERROR loading scan data: {}'.format(e) + Style.RESET_ALL)
        return False

def perform_scan(master, tfxc, xyz):
    global occupancy_map
    try:
        print(Fore.CYAN + 'xyz: ' + Fore.GREEN + '{}\n'.format(xyz) + Style.RESET_ALL)
        print(Fore.WHITE + 'continuing…' + Style.RESET_ALL)

        for _ in range(BUTTON_DELAY_SEC):
            response = tfxc.send_request(TICK_COMMAND)
            print('response: {}'.format(response))
            time.sleep(1)
        response = tfxc.send_request(BEEP_COMMAND)
        print('response: {}'.format(response))
        time.sleep(1)
        
        x, y, heading = xyz
        response = master.send_request(DISTANCES_COMMAND)
        while response is None or response == DISTANCES_COMMAND:
            print(Style.DIM + 'invalid response: {}'.format(response) + Style.RESET_ALL)
            response = master.send_request(DISTANCES_COMMAND)
        print(Fore.CYAN + 'response: ' + Fore.YELLOW + '({},{},{}°) '.format(x, y, heading) + Fore.GREEN + '{}'.format(response) + Style.RESET_ALL)

        # initialize map on first scan
        if occupancy_map is None:
            occupancy_map = AdaptiveOccupancyMap(min_cell_size=50, initial_size=8000)
            print(Fore.MAGENTA + 'adaptive occupancy map initialized' + Style.RESET_ALL)

        # add sensor reading to map
        occupancy_map.process_sensor_reading(response, x, y, heading)
        print(Fore.MAGENTA + 'sensor reading added to map' + Style.RESET_ALL)

    except Exception as e:
        print(Fore.RED + 'ERROR: {} raised in perform_scan: {}'.format(type(e),e) + Style.RESET_ALL)


# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

button = None
button_pressed = False
occupancy_map = None

def main():

    global button_pressed
    worker_thread = None
    stop_event    = Event()
    i2c_lock      = Lock()
    i2c_address = I2C_ADDRESS
    prompt = "► "

    try:

        tfxc = TinyFxController()
        tfxc.enable()

        parser = argparse.ArgumentParser(description='I2C master remote control')
        parser.add_argument('--address',
                type=lambda x: int(x, 0), default=I2C_ADDRESS,
                help='I2C device address (default: 0x{:02x})'.format(I2C_ADDRESS))
        args = parser.parse_args()
        i2c_address = args.address
        print('connecting to device at 0x{:02X}…'.format(i2c_address))
        master = I2CMaster(i2c_id=I2C_ID, i2c_address=i2c_address)
        master.enable()
        print('\nEnter command string to send (Ctrl-C or "exit" to exit):')

        def button_handler(pressed):
            global button_pressed
            if pressed:
                print("pressed.")
            else:
                print("released…")
                button_pressed = True

        button = InterruptButton(pin=21, debounce_ms=50, callback=button_handler)

        last_user_msg = None
        print(prompt, end='', flush=True)
        
        while True:
            if button_pressed:
                button_pressed = False
                print() # newline after prompt
                xyz = read_xyz()
                if xyz is None:
                    xyz = 0, 0, 0
                perform_scan(master, tfxc, xyz)
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

            elif user_msg.strip() == 'exit' or user_msg.strip() == 'quit':
                break

            elif user_msg == 'go':
                if worker_thread and worker_thread.is_alive():
                    print("worker already running")
                else:
                    stop_event.clear()
                    worker_thread = Thread(target=worker_loop, args=(master, stop_event, i2c_lock), daemon=True,)
                    worker_thread.start()
                print(prompt, end='', flush=True)
                continue

            elif user_msg == 'stop':
                if worker_thread and worker_thread.is_alive():
                    stop_event.set()
                    worker_thread.join()
                else:
                    print("worker not running")
                print(prompt, end='', flush=True)
                continue

            elif user_msg == 'viz':
                if occupancy_map is None:
                    print("no map data available - press button to scan first")
                else:
                    if TRIM_OCCUPANCY_MAP_TO_DATA:
                        occupancy_map.trim_to_data(margin_mm=100)
                    visualizer = MapVisualizer(occupancy_map)
                    visualizer.generate_svg("occupancy_map.svg")
                print(prompt, end='', flush=True)
                continue

            elif user_msg.startswith('load '):
                filename = user_msg[5:].strip()
                if not filename:
                    print("usage: load <filename>")
                else:
                    load_scan_data(filename)
                print(prompt, end='', flush=True)
                continue

            print('user msg: {}'.format(user_msg))
            with i2c_lock:
                response = master.send_request(user_msg)
                if response == user_msg:
                    print(Fore.YELLOW + 'response matches input.' + Style.RESET_ALL)

            print('response: {}'.format(response))
            last_user_msg = user_msg
            print(prompt, end='', flush=True)

    except OSError as e:
        print('unable to connect with device at 0x{:02X}'.format(i2c_address))
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
