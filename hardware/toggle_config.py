#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-28
# modified: 2025-11-29

from colorama import init, Fore, Style
init()

import ioexpander as io

from core.logger import Logger, Level
from core.component import Component, MissingComponentError
#from hardware.radiozoa_sensor import RadiozoaSensor

class ToggleConfig(Component):
    NAME = 'toggle-config'
    '''
    Reads four pins from an IOExpander connected to four toggle switches.

    This provides a configurable API by toggle switch number and one or
    more assigned names for each toggle, e.g.:

        toggle1:      'radiozoa'
        toggle2:          'roam'
        toggle3:  'avoid swerve'
        toggle4:    'scout scan'
    '''
    def __init__(self, config, level=Level.INFO):
        self._log = Logger(ToggleConfig.NAME, level=level)
        if config is None:
            raise ValueError('no configuration provided.')
        self._config = config
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._level = level
        # config ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _cfg = config.get('kros').get('hardware').get('toggle_config')
        self._ioe = None
        # try to get the existing IOE from RadiozoaSensor, otherwise create our own
        _component_registry = Component.get_registry()
        # uses the Radiozoa configuration since we're using the same IOExpander
        _radiozoa_sensor = _component_registry.get('radiozoa-sensor') # hard-coded name
        if _radiozoa_sensor:
            self._log.info('obtaining IOExpander from RadiozoaSensor…')
            self._ioe = _radiozoa_sensor.ioe
        else:
            self._log.info('creating IOExpander…')
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
            # set up I2C ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            _cfg_radiozoa    = config.get('kros').get('hardware').get('radiozoa')
            _ioe_i2c_address = '0x{:02X}'.format(_cfg_radiozoa.get('ioe_i2c_address'))
            self._i2c_bus_number = _cfg_radiozoa.get('i2c_bus_number')
            if not isinstance(self._i2c_bus_number, int):
                raise ValueError('expected an int for an I2C bus number, not a {}.'.format(type(self._i2c_bus_number)))
            self._i2c_bus = smbus.SMBus()
            self._i2c_bus.open(bus=self._i2c_bus_number)
            self._log.debug('I2C{} open.'.format(self._i2c_bus_number))
            # set up IO Expander ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            try:
                self._ioe = io.IOE(i2c_addr=0x18, smbus_id=self._i2c_bus_number)
                self._log.info('found IO Expander at {} on I2C{}.'.format(_ioe_i2c_address, self._i2c_bus_number))
            except Exception as e:
                self._log.error('{} raised setting up IO Expander: {}'.format(type(e), e))
                raise MissingComponentError('could not find IO Expander at {} on I2C{}.'.format(_ioe_i2c_address, self._i2c_bus_number))
        # configuration by pin
        self._pin1 = _cfg.get('pin1')
        self._pin2 = _cfg.get('pin2')
        self._pin3 = _cfg.get('pin3')
        self._pin4 = _cfg.get('pin4')
        # set up pins
        self._ioe.set_mode(self._pin1, io.IN_PU)
        self._ioe.set_mode(self._pin2, io.IN_PU)
        self._ioe.set_mode(self._pin3, io.IN_PU)
        self._ioe.set_mode(self._pin4, io.IN_PU)
        # assignments by name
        old_way = '''
        self._toggle1 = _cfg.get('toggle1')
        self._toggle2 = _cfg.get('toggle2')
        self._toggle3 = _cfg.get('toggle3')
        self._toggle4 = _cfg.get('toggle4')
        self._assignments = {
                self._toggle1: lambda: self.toggle(1),
                self._toggle2: lambda: self.toggle(2),
                self._toggle3: lambda: self.toggle(3),
                self._toggle4: lambda: self.toggle(4)
            }
        '''
        self._toggle1 = self._parse_names(_cfg.get('toggle1'))
        self._toggle2 = self._parse_names(_cfg.get('toggle2'))
        self._toggle3 = self._parse_names(_cfg.get('toggle3'))
        self._toggle4 = self._parse_names(_cfg.get('toggle4'))
        toggles = [
            (1, self._toggle1),
            (2, self._toggle2),
            (3, self._toggle3),
            (4, self._toggle4),
        ]
        self._assignments = {}
        for num, names in toggles:
            for name in names:
                self._assignments[name] = lambda n=num: self.toggle(n)
        self.print_config()
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _parse_names(self, value):
        if not value:
            return []
        return value.split() if isinstance(value, str) else list(value)

    def print_config(self):
        self._log.info("toggle assignments:")
        grouped = {1: [], 2: [], 3: [], 4: []}
        for name, fn in self._assignments.items():
            n = fn.__defaults__[0] # toggle number that the lambda was assigned to
            grouped[n].append(name)
        for n in range(1, 5):
            names = grouped[n]
            txt = " ".join(names) if names else "(none)"
            self._log.info("  {}: {}{}{}".format(n, Fore.GREEN, txt, Style.RESET_ALL))

    def has_assignment(self, name):
        return name in self._assignments

    def is_enabled(self, name):
        '''
        Returns True if name is found amongst the assignments and it is
        configured as enabled (statically).
        '''
        return self._assignments.get(name, lambda: False)()

    def toggle(self, pin):
        '''
        Returns True if the specified toggle switch is active (low).
        '''
        match(pin):
            case 1:
                return not self._ioe.input(self._pin1)
            case 2:
                return not self._ioe.input(self._pin2)
            case 3:
                return not self._ioe.input(self._pin3)
            case 4:
                return not self._ioe.input(self._pin4)
            case _:
                raise Exception('no such pin.')

    @property
    def toggle1(self):
        '''
        Returns True if toggle switch 1 is active (low).
        '''
        return not self._ioe.input(self._pin1)

    @property
    def toggle2(self):
        '''
        Returns True if toggle switch 2 is active (low).
        '''
        return not self._ioe.input(self._pin2)

    @property
    def toggle3(self):
        '''
        Returns True if toggle switch 3 is active (low).
        '''
        return not self._ioe.input(self._pin3)

    @property
    def toggle4(self):
        '''
        Returns True if toggle switch 4 is active (low).
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
