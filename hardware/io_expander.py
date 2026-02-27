#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-28
# modified: 2026-02-26

import traceback
from colorama import init, Fore, Style
init()

import ioexpander as io

from core.logger import Logger, Level
from core.component import Component, MissingComponentError

class IoExpander(Component):
    NAME = 'io-expander'
    '''
    A wrapper class around the Pimoroni IO Expander as a Component.
    '''
    def __init__(self, config, level=Level.INFO):
        self._log = Logger(IoExpander.NAME, level=level)
        if config is None:
            raise ValueError('no configuration provided.')
        self._config = config
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._level = level
        # config ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _cfg = config.get('kros').get('hardware').get('io_expander')
        self._ioe = None
        # try to get the existing IOE from registry, otherwise create our own
        _component_registry = Component.get_registry()
        _ioe = _component_registry.get(IoExpander.NAME)
        if _ioe:
            self._log.info('obtaining IO Expander from registry…')
            self._ioe = _ioe
        else:
            self._log.info('creating IO Expander…')
            _i2c_address = _cfg.get('i2c_address')
            _i2c_bus_number = _cfg.get('i2c_bus_number')
            try:
                self._log.info('opening IO Expander at {} on I2C{}…'.format(_i2c_address, _i2c_bus_number))
                self._ioe = io.IOE(i2c_addr=_i2c_address, smbus_id=_i2c_bus_number)
                self._log.info('found IO Expander at {} on I2C{}.'.format(_i2c_address, _i2c_bus_number))
            except Exception as e:
                self._log.error('{} raised setting up IO Expander: {}\n{}'.format(type(e), e, traceback.format_exc()))
                raise MissingComponentError('could not find IO Expander at {} on I2C{}.'.format(_i2c_address, _i2c_bus_number))
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def ioe(self):
        '''
        Return the instance of the IO Expander.
        '''
        return self._ioe

    def enable(self):
        if not self.enabled:
            super().enable()
            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def disable(self):
        if not self.disabled:
            super().disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

    def close(self):
        if not self.closed:
            super().close()
            self._log.info('closed.')
        else:
            self._log.warning('already closed.')

#EOF
