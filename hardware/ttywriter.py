#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-03-10
# modified: 2026-03-10

import sys
import re
import subprocess
import argparse
import time
from colorama import init, Fore, Style

class TtyWriter:
    # mapping of keywords to colorama Fore colors
    COLOR_MAP = {
        "BLACK":   Fore.BLACK,
        "RED":     Fore.RED,
        "GREEN":   Fore.GREEN,
        "YELLOW":  Fore.YELLOW,
        "BLUE":    Fore.BLUE,
        "MAGENTA": Fore.MAGENTA,
        "CYAN":    Fore.CYAN,
        "WHITE":   Fore.WHITE,
        "DIM":     Style.DIM,
        "NORMAL":  Style.NORMAL,
        "BRIGHT":  Style.BRIGHT,
        "RESET":   Style.RESET_ALL
    }
    _colorama_initialized = False

    def __init__(self, tty='/dev/tty1', append=False):
        '''
        Initialize the TtyWriter with the console device.
        '''
        self.tty = tty
        self.append = append
        if not TtyWriter._colorama_initialized:
            init() # initialize colorama
            TtyWriter._colorama_initialized = True

    def write(self, text, colorise=False):
        '''
        Write text to the console. Interprets literal '\\n' as newlines.
        Optionally colorise keywords.
        '''
        if not isinstance(text, str):
            text = str(text)
        text = text.replace('\\n', '\n')
        if colorise:
            # pattern: color keyword followed by optional space
            pattern = r'\b(' + '|'.join(self.COLOR_MAP.keys()) + r')\b\s?'
            # replace keyword + optional space with ANSI code only
            def repl(m):
                return self.COLOR_MAP[m.group(1)]
            text = re.sub(pattern, repl, text)
            # each line must end with a reset
            text = '\n'.join(line + Style.RESET_ALL for line in text.splitlines())
        mode = 'a' if self.append else 'w'
        with open(self.tty, mode) as f:
            f.write(text + '\n')

    def exec(self, cmd):
        '''
        Execute a shell command and send its output to the console.
        '''
        mode = 'a' if self.append else 'w'
        with open(self.tty, mode) as f:
            try:
                subprocess.run(cmd, shell=True, stdout=f, stderr=f, check=True)
            except subprocess.CalledProcessError as e:
                f.write("command failed with exit code {}\n".format(e.returncode))

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def main():
    parser = argparse.ArgumentParser(
        description="write text or execute commands on a console (/dev/tty1)."
    )
    parser.add_argument(
        '-a', '--append', action='store_true',
        help="append to the console instead of overwriting"
    )
    parser.add_argument(
        '-c', '--color', action='store_true',
        help="replace uppercase color keywords in text with colorama codes"
    )
    parser.add_argument(
        '-C', '--clear', action='store_true',
        help="clear the console before writing or executing"
    )
    parser.add_argument(
        '-e', '--exec', action='store_true',
        help="execute the given command instead of writing text"
    )
    parser.add_argument(
        '-i', '--ip', action='store_true',
        help="prints the IP address of the host"
    )
    parser.add_argument(
        '-l', '--loop', action='store_true',
        help="loop and print data to the console"
    )
    parser.add_argument(
        'content', nargs=argparse.REMAINDER,
        help="text to write or command to execute"
    )

    try:

        args = parser.parse_args()
        ttywriter = TtyWriter(append=args.append)
        # clear console if requested
        if args.clear:
            ttywriter.exec('clear')
        if args.ip:
            cmd = "ip -4 addr show wlan0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}'"
            ttywriter.exec(cmd)
        elif not args.content:
            parser.print_help()
            sys.exit(1)
        # join content into a single string
        payload = ' '.join(args.content)
        if args.exec:
            ttywriter.exec(payload)
        elif args.loop:
            while True:
                ttywriter.write(payload, colorise=args.color)
                time.sleep(1)
        else:
            ttywriter.write(payload, colorise=args.color)

    except KeyboardInterrupt:
        print('\nCtrl-C caught; exiting…')
    except Exception as e:
        print('error in test: {}'.format(e))

if __name__ == "__main__":
    main()

#EOF
