#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-25
# modified: 2025-05-27
#

import argparse
import sys, traceback
import time
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from hardware.controller import Controller
from hardware.payload import Payload
from hardware.response import*

def parse_args():
    parser = argparse.ArgumentParser(description="Process command and optional speeds/duration.")
    # first positional argument: the base command
    parser.add_argument("command", type=str, help="The command to execute.")
    # optional positional arguments (zero or more float arguments)
    parser.add_argument("args", nargs="*", help="Optional arguments for the command (e.g., speeds, duration).")
    return parser.parse_args()

def main():

    _log = Logger('main', Level.INFO)
    _controller = None

    try:

        _log.info('controller begin…')

        _controller = Controller('tinyfx', i2c_bus=1, i2c_address=0x44)

        start_time = dt.now()

        # parse the arguments
        _args = parse_args()
        # combine into a single command string
        _command_string = ' '.join([_args.command] + _args.args)
#       _response = _controller.send_payload(_args.command)
        _response = _controller.send_payload(_command_string)
        elapsed_ms = (dt.now() - start_time).total_seconds() * 1000.0
        if _response is None:
            raise ValueError('null response.')
        elif isinstance(_response, Response):
            if _response == RESPONSE_OKAY:
                _log.info("response: "
                        + Fore.GREEN + "'{}'".format(_response.description)
                        + Fore.CYAN + "; {:5.2f}ms elapsed.".format(elapsed_ms))
            else:
                _log.warning("response: "
                        + Fore.RED + "'{}'".format(_response.description)
                        + Fore.WHITE + "; {:5.2f}ms elapsed.".format(elapsed_ms))
        elif not isinstance(_response, Response):
            raise ValueError('expected Response, not {}.'.format(type(_response)))
        else:
            _log.error("error response: {}; {:5.2f}ms elapsed.".format(_response.description, elapsed_ms))

    except KeyboardInterrupt:
        print("Program interrupted by user (Ctrl+C). Exiting gracefully.")
    except ValueError as e:
        # handle any CLI validation errors
        _log.error("parsing command line: {}".format(e))
    except TimeoutError as te:
        _log.error('transfer timeout: {}'.format(te))
    except Exception as e:
        _log.error('{} encountered: {}\n{}'.format(type(e), e, traceback.format_exc()))
    finally:
        _log.info('complete.')
        if _controller:
            _controller.close()

if __name__ == "__main__":
    main()

#EOF
