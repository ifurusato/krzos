#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-25
# modified: 2025-09-12
#

import argparse
import sys, traceback
import time
import datetime as dt
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.controller import Controller
from hardware.payload import Payload
from hardware.response import*

class TinyFxController(Component):
    NAME = 'tinyfx'
    '''
    Connects with a Tiny FX over I2C.
    '''
    def __init__(self, config=None, level=Level.INFO):
        self._log = Logger(TinyFxController.NAME, level)
        self._log.info('instantiating TinyFxController…')
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        _cfg = config['kros'].get('hardware').get('tinyfx-controller')
        self._i2c_address        = _cfg.get('i2c_address')
        self._bus_number         = _cfg.get('bus_number')
        self._i2cbus             = None
        self._config_register    = 1
        self._max_payload_length = 32
        self._controller = Controller('tinyfx', i2c_bus=1, i2c_address=0x44)
        self._last_send_time = None  # timestamp of last send
        self._min_send_interval = dt.timedelta(milliseconds=100)  # 100ms minimum send interval
        self._log.info('ready at 0x{:02X} on I2C bus {}.'.format(self._i2c_address, self._bus_number))

    def send_data(self, data):
        '''
        Sends a string to the TinyFX.
        '''
        start_time = dt.datetime.now()
        if self._last_send_time:
            elapsed = start_time - self._last_send_time
            if elapsed < self._min_send_interval:
                self._log.warning(
                    "write_payload skipped: only {:.1f}ms since last send (minimum is {:.1f}ms)".format(
                        elapsed.total_seconds() * 1000,
                        self._min_send_interval.total_seconds() * 1000
                    )   
                )   
                return RESPONSE_SKIPPED
        self._log.info("sending data: '{}'…".format(data))
        _response = self._controller.send_payload(data)
        elapsed_ms = (dt.datetime.now() - start_time).total_seconds() * 1000.0
        if _response is None:
            raise ValueError('null response.')
        elif isinstance(_response, Response):
            if _response == RESPONSE_OKAY:
                self._log.info("response: "
                        + Fore.GREEN + "'{}'".format(_response.description)
                        + Fore.CYAN + "; {:5.2f}ms elapsed.".format(elapsed_ms))
            else:
                self._log.warning("response: "
                        + Fore.RED + "'{}'".format(_response.description)
                        + Fore.WHITE + "; {:5.2f}ms elapsed.".format(elapsed_ms))
        elif not isinstance(_response, Response):
            raise ValueError('expected Response, not {}.'.format(type(_response)))
        else:
            self._log.error("error response: {}; {:5.2f}ms elapsed.".format(_response.description, elapsed_ms))

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

        _log.info('creating TinyFX controller…')
        _config = ConfigLoader(Level.INFO).configure()
        _tinyfx = TinyFxController(config=_config, level=Level.INFO)

        # parse the arguments and combine into a single command string
        _args = parse_args()
        _command_string = ' '.join([_args.command] + _args.args)
        _response = _tinyfx.send_data(_command_string)

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
