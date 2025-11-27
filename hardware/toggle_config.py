#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-28
# modified: 2025-11-28

from colorama import init, Fore, Style
init()

import ioexpander as io

from core.logger import Logger, Level
from core.component import Component, MissingComponentError
#from hardware.radiozoa_sensor import RadiozoaSensor

class ToggleConfig(Component):
    NAME = 'toggle-config'
    '''
    Reqads four pins from an IOExpander connected to four toggle switches.
    '''
    def __init__(self, config, level=Level.INFO):
        '''
        This uses the Radiozoa configuration since we're using the same
        IOExpander.

        Args:
            config (dict): The configuration dictionary for the sensors.
            level (Level): The logging level.
        '''
        self._log = Logger(ToggleConfig.NAME, level=level)
        if config is None:
            raise ValueError('no configuration provided.')
        self._config = config
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._level = level
        # config ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._ioe = None
        # try to get the existing IOE from RadiozoaSensor, otherwise create our own
        _component_registry = Component.get_registry()
        _radiozoa_sensor = _component_registry.get('radiozoa-sensor')
        if _radiozoa_sensor:
            self._log.info(Fore.MAGENTA + 'obtaining IOExpander from RadiozoaSensor…')
            self._ioe = _radiozoa_sensor.ioe
        else:
            self._log.info(Fore.MAGENTA + 'creating IOExpander…')
            import pkg_resources
            SMBUS='smbus2'
            for dist in pkg_resources.working_set:
                if dist.project_name == 'smbus':
                    break
                if dist.project_name == 'smbus2':
                    SMBUS='smbus2'
                    break
            if SMBUS == 'smbus':
                import smbus
            elif SMBUS == 'smbus2':
                import smbus2 as smbus

            _cfg_radiozoa    = config.get('kros').get('hardware').get('radiozoa')
            _ioe_i2c_address = '0x{:02X}'.format(_cfg_radiozoa.get('ioe_i2c_address'))
            self._i2c_bus_number = _cfg_radiozoa.get('i2c_bus_number')
            if not isinstance(self._i2c_bus_number, int):
                raise ValueError('expected an int for an I2C bus number, not a {}.'.format(type(self._i2c_bus_number)))
            self._i2c_bus = smbus.SMBus()
            self._i2c_bus.open(bus=self._i2c_bus_number)
            self._log.debug('I2C{} open.'.format(self._i2c_bus_number))
            # set up IO Expander ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            try:
                self._ioe = io.IOE(i2c_addr=0x18, smbus_id=self._i2c_bus_number)
                self._log.info('found IO Expander at {} on I2C{}.'.format(_ioe_i2c_address, self._i2c_bus_number))
            except Exception as e:
                self._log.error('{} raised setting up IO Expander: {}'.format(type(e), e))
                raise MissingComponentError('could not find IO Expander at {} on I2C{}.'.format(_ioe_i2c_address, self._i2c_bus_number))
        # yes we could configure this from YAML but… why?
        self._pin1 = 2
        self._pin2 = 4
        self._pin3 = 3
        self._pin4 = 1
        self._ioe.set_mode(self._pin1, io.IN_PU)
        self._ioe.set_mode(self._pin2, io.IN_PU)
        self._ioe.set_mode(self._pin3, io.IN_PU)
        self._ioe.set_mode(self._pin4, io.IN_PU)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def pin1(self):
        '''
        Returns True if toggle 1 is active (low).
        '''
        return not self._ioe.input(self._pin1)

    @property
    def pin2(self):
        '''
        Returns True if toggle 2 is active (low).
        '''
        return not self._ioe.input(self._pin2)

    @property
    def pin3(self):
        '''
        Returns True if toggle 3 is active (low).
        '''
        return not self._ioe.input(self._pin3)

    @property
    def pin4(self):
        '''
        Returns True if toggle 4 is active (low).
        '''
        return not self._ioe.input(self._pin4)

    @property
    def ioe(self):
        '''
        Return the instance of the IOExpander.
        '''
        return self._ioe

    def enable(self):
        if not self.enabled:
            Component.enable(self)
            self._log.info('enabled.')
        else:
            self._log.debug('already enabled.')

    def disable(self):
        if not self.disabled:
            Component.disable(self)
            self._log.info('disabled.')
        else:
            self._log.debug('already disabled.')

    def close(self):
        if not self.closed:
            Component.close(self)
            self._log.info('closed.')
        else:
            self._log.debug('already closed.')

#EOF
