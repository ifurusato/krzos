#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2025-12-16
'''
    odo pos             # robot position (x,y)
        vel             # robot velocity (vx,vy)
        ts              # timestamp of last odometer update
        reset           # resets odometer
        led on | off
        rf get
        rf set <value> | None
'''

import time
from datetime import datetime as dt, timezone
from zoneinfo import ZoneInfo
import traceback
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.numbers import Numbers
from hardware.i2c_master import I2CMaster

class NofsOdometer(Component):
    NAME = 'nofs-odometer'

    def __init__(self, config, suppressed=False, enabled=False, level=Level.INFO):
        self._config = config
        self._log = Logger(NofsOdometer.NAME, level)
        Component.__init__(self, self._log, suppressed=suppressed, enabled=enabled)
        # obtain geometry from config (use 'kros.geometry')
        _cfg = config['kros'].get('hardware').get('nofs_odometer')
        _i2c_address = _cfg.get('i2c_address') # 0x41
        self._master = I2CMaster('pico', i2c_address=_i2c_address, timeset=True)
        self._repeats = 4
        self._repeat_delay_sec = 0.5
        # internal state
        self._last_time = None
        self._seconds = 0   # how many seconds since last update?
        self._x     = 0.0   # forward position (cm)
        self._y     = 0.0   # lateral position (cm)
        self._theta = 0.0   # heading (radians)
        self._vx    = 0.0   # forward velocity (cm/s)
        self._vy    = 0.0   # lateral velocity (cm/s)
        self._omega = 0.0   # yaw rate (rad/s)
        self._timezone = ZoneInfo("Pacific/Auckland")
        self._log.info('ready.')

    def _update_ts(self, _ts):
        _mcu_dt = dt.fromtimestamp(_ts, tz=timezone.utc)
        self._last_time = dt.fromtimestamp(_ts, tz=self._timezone)
        _now_dt = dt.now(tz=timezone.utc)
        elapsed = _now_dt - _mcu_dt  # timedelta object
        seconds_elapsed = elapsed.total_seconds()
        hours, remainder = divmod(seconds_elapsed, 3600)
        minutes, self._seconds = divmod(remainder, 60)
        self._log.info("elapsed: " + Fore.YELLOW + "{:.2f}s".format(self._seconds))

    def get_timestamp(self):
        '''
        timestamp = self._odometer.ts
        '''
        try:
            response = self._master.send_data_request('odo ts')
            if response is None:
                self._log.warning('None: data response on position.')
            elif response == 'ACK':
                self._log.info('ACK: data response on position: {}'.format(response))
            elif response == 'ERR':
                self._log.error('ERR: data response on position: {}'.format(response))
            else:
                self._log.info(Style.DIM + 'data response on position: {}'.format(response))
                timestamp = int(response)
                self._update_ts(timestamp)
                self._log.info('response on timestamp: ' + Fore.YELLOW + "'{}'".format(timestamp)
                        + Fore.CYAN + "; last time: " + Fore.YELLOW + "'{}'".format(self._last_time))
        except Exception as e:
            self._log.error('{} raised getting position: {}\n{}'.format(type(e), e, traceback.format_exc()))

    def get_position(self):
        '''
        Tries 3 times to get the position.
        '''
        self._log.info('get position called  -------------------------- ')
        for i in range(self._repeats):
            ordinal = Numbers.from_number(i+1, prefer_ordinal=True)
            x, y = self._get_position()
            if x != None:
                self._log.info('> get_position: ({}, {}) on {} try.'.format(x, y, ordinal))
                self._x = x
                self._y = y
                self._log.info(Fore.WHITE + 'A. 🏁 _get_position() response on position: ' + Fore.YELLOW + '({}, {})'.format(self._x, self._y))
                return self._x, self._y
            else:
                self._log.warning('get_position: RETURNED ({},{}) on {} try.'.format(x, y, ordinal))
            time.sleep(self._repeat_delay_sec)
        return None, None

    def _get_position(self):
        '''
        x, y = self._odometer.position
        '''
        try:
            response = self._master.send_data_request('odo pos')
            if response is None:
                self._log.warning('_get_position() None: data response on position.')
            elif response == 'ACK':
                self._log.info('_get_position() ACK: data RESPONSE on position: {}'.format(response))
            elif response == 'ERR':
                self._log.error('_get_position() ERR: data RESPONSE on position: {}'.format(response))
            else:
                self._log.info(Style.DIM + '_get_position() data RESPONSE on position: {}'.format(response))
                parts = response.split()
                if len(parts) == 2:
                    x = int(parts[0])
                    y = int(parts[1])
                    self._log.info(Fore.WHITE + 'B. 🏁 _get_position() RETURNING response on position: ' + Fore.YELLOW + '({}, {})'.format(x, y))
                    return x, y
                else:
                    self._log.error('ERR: could not parse response on position: {}'.format(response))
        except Exception as e:
            self._log.error('{} raised getting position: {}\n{}'.format(type(e), e, traceback.format_exc()))
        return None, None

    def get_velocity(self):
        '''
        Tries three times to return the velocity.
        '''
        for i in range(self._repeats):
            ordinal = Numbers.from_number(i+1, prefer_ordinal=True)
            vx, vy = self._get_velocity()
            if vx != None:
                self._log.info('> get_position: ({}, {}) on {} try.'.format(vx, vy, ordinal))
                self._vx = vx
                self._vy = vy
                self._log.info(Fore.WHITE + 'A. 🏁 get_velocity() response on velocity: ' + Fore.YELLOW + '({}, {})'.format(self._vx, self._vy))
                return self._x, self._y
            else:
                self._log.warning('get_position: RETURNED {},{}'.format(vx, vy))
            time.sleep(self._repeat_delay_sec)
        return None, None
       

    def _get_velocity(self):
        '''
        vx, vy = self._odometer.velocity
        '''
        try:
            response = self._master.send_data_request('odo vel')
            if response is None:
                self._log.warning('None: data response on velocity.')
            elif response == 'ACK':
                self._log.info('ACK: data response on velocity: {}'.format(response))
            elif response == 'ERR':
                self._log.error('ERR: data response on velocity: {}'.format(response))
            else:
                self._log.info(Style.DIM + 'data response on velocity: {}'.format(response))
                parts = response.split()
                if len(parts) == 2:
                    vx = int(parts[0])
                    vy = int(parts[1])
                    self._log.info("response on velocity: " + Fore.YELLOW + "({}, {})".format(vx, vy))
                    return vx, vy
                else:
                    self._log.error('ERR: could not parse response on velocity: {}'.format(response))
        except Exception as e:
            self._log.error('{} raised getting velocity: {}\n{}'.format(type(e), e, traceback.format_exc()))
        return None, None

    def reset(self):
        '''
        Tries three times to reset the odometer.
        '''
        for i in range(self._repeats):
            ordinal = Numbers.from_number(i+1, prefer_ordinal=True)
            was_reset = self._reset()
            x, y = self.get_position()
            self._log.info('RESET: ({},{}) was reset? {}'.format(x, y, was_reset))
            if x == None or y == None:
                self._log.warning('RESET failed on {} try with None.'.format(ordinal))
            if x == 0 and y == 0:
                self._log.info('RESET worked on {} try.'.format(ordinal))
                return
            else:
                self._log.warning('RESET failed on {} try with ({}, {}).'.format(ordinal, x, y))
            time.sleep(self._repeat_delay_sec)

    def _reset(self):
        '''
        self._odometer.reset
        '''
        try:
            response = self._master.send_data_request('odo reset')
            if response is None:
                self._log.warning(Fore.MAGENTA + 'None: data response on reset.')
            elif response == 'ACK':
                self._log.info(Fore.MAGENTA + 'ACK: data response on reset: {}'.format(response))
                return True
            elif response == 'ERR':
                self._log.error(Fore.MAGENTA + 'ERR: data response on reset: {}'.format(response))
            else:
                self._log.info(Fore.MAGENTA + Style.DIM + 'data response on reset: {}'.format(response))
        except Exception as e:
            self._log.error('{} raised getting reset: {}\n{}'.format(type(e), e, traceback.format_exc()))
        return False

    def enable(self):
        if not self.enabled:
            Component.enable(self)
            self._master.enable()
            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def disable(self):
        if self.enabled:
            Component.disable(self)
            self._master.disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

#EOF
