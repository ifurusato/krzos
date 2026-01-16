#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2020-10-05
# modified: 2025-10-12

import sys, traceback
from fractions import Fraction
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level
from core.orientation import Orientation
from hardware.i2c_scanner import I2CScanner
from hardware.motor import Motor
from hardware.decoder import Decoder
from hardware.system import System
from hardware.thunderborg import ThunderBorg

class MotorConfigurer(Component):
    NAME = 'motor-config'
    '''
    Configures either a ThunderBorg motor controller for a pair of motors.

    :param config:          the application configuration
    :param i2c_scanner:     the I²C bus scanner
    :param motors_enabled:  an optional flag to enable motors (default false)
    :param level:           the logging level
    '''
    def __init__(self, config, i2c_scanner, motors_enabled=False, level=Level.INFO):
        self._log = Logger(MotorConfigurer.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        if config is None:
            raise ValueError('null configuration argument.')
        self._config = config
        if not isinstance(i2c_scanner, I2CScanner):
            raise ValueError('expected I2CScanner, not {}.'.format(type(i2c_scanner)))
        self._i2c_scanner = i2c_scanner
        self._system = Component.get_registry().get('system')
        if self._system is None:
            self._system = System(config)
        self._log.debug('getting battery reading…')
        # configure from command line argument properties
        _args = self._config['kros'].get('arguments')
        self._motors_enabled = _args.get('motors_enabled') or motors_enabled
        self._log.info('motors enabled? {}'.format(self._motors_enabled))
        # import the ThunderBorg library, then configure and return the motors
        self._max_power_ratio = None
        self._enable_tb_scan = False
        self._port_tb = self._import_thunderborg(Orientation.PORT)
        self._log.info('configured PORT ThunderBorg at I2C address: 0x{:02X}'.format(self._port_tb.I2cAddress))
        self._stbd_tb  = self._import_thunderborg(Orientation.STBD)
        self._log.info('configured STBD ThunderBorg at I2C address: 0x{:02X}'.format(self._stbd_tb.I2cAddress))
        if self._max_power_ratio is None: # this should have been set by the ThunderBorg code.
            raise ValueError('max_power_ratio not set.')
        try:
            # now import motors
            self._log.info('configuring motors…')
            # GPIO pins configured for A and B channels for each encoder, reversed on starboard side
            _odo_cfg = self._config['kros'].get('motor').get('odometry')
            # PFWD "port-forward"
            self._pfwd_motor = Motor(self._config, self._port_tb, Orientation.PFWD, level)
            self._pfwd_motor.max_power_ratio = self._max_power_ratio
            _motor_encoder_pfwd_a    = _odo_cfg.get('motor_encoder_pfwd_a')
            _motor_encoder_pfwd_b    = _odo_cfg.get('motor_encoder_pfwd_b')
            _reverse_encoder_pfwd    = _odo_cfg.get('reverse_encoder_pfwd')
            self._pfwd_motor.decoder = Decoder(
                orientation = Orientation.PFWD,
                gpio_a  = _motor_encoder_pfwd_a, 
                gpio_b  = _motor_encoder_pfwd_b, 
                reverse = _reverse_encoder_pfwd
            )
            self._log.info('configured {} motor encoder on pin {} and {} (reversed? {}).'.format(
                Orientation.PFWD, _motor_encoder_pfwd_a, _motor_encoder_pfwd_b, _reverse_encoder_pfwd))

            # SFWD "starboard-forward"
            self._sfwd_motor = Motor(self._config, self._stbd_tb, Orientation.SFWD, level)
            self._sfwd_motor.max_power_ratio = self._max_power_ratio
            _motor_encoder_sfwd_a    = _odo_cfg.get('motor_encoder_sfwd_a')
            _motor_encoder_sfwd_b    = _odo_cfg.get('motor_encoder_sfwd_b')
            _reverse_encoder_sfwd    = _odo_cfg.get('reverse_encoder_sfwd')
            self._sfwd_motor.decoder = Decoder(
                orientation = Orientation.SFWD,
                gpio_a  = _motor_encoder_sfwd_a,
                gpio_b  = _motor_encoder_sfwd_b,
                reverse = _reverse_encoder_sfwd
            )
            self._log.info('configured {} motor encoder on pin {} and {} (reversed? {}).'.format(
                Orientation.SFWD, _motor_encoder_sfwd_a, _motor_encoder_sfwd_b, _reverse_encoder_sfwd))

            # PAFT "port-aft"
            self._paft_motor = Motor(self._config, self._port_tb, Orientation.PAFT, level)
            self._paft_motor.max_power_ratio = self._max_power_ratio
            _motor_encoder_paft_a    = _odo_cfg.get('motor_encoder_paft_a')
            _motor_encoder_paft_b    = _odo_cfg.get('motor_encoder_paft_b')
            _reverse_encoder_paft    = _odo_cfg.get('reverse_encoder_paft')
            self._paft_motor.decoder = Decoder(
                orientation = Orientation.PAFT,
                gpio_a  = _motor_encoder_paft_a,
                gpio_b  = _motor_encoder_paft_b,
                reverse = _reverse_encoder_paft
            )
            self._log.info('configured {} motor encoder on pin {} and {} (reversed? {}).'.format(
                Orientation.PAFT, _motor_encoder_paft_a, _motor_encoder_paft_b, _reverse_encoder_paft))

            # SAFT "starboard-aft"
            self._saft_motor = Motor(self._config, self._stbd_tb, Orientation.SAFT, level)
            self._saft_motor.max_power_ratio = self._max_power_ratio
            _motor_encoder_saft_a    = _odo_cfg.get('motor_encoder_saft_a')
            _motor_encoder_saft_b    = _odo_cfg.get('motor_encoder_saft_b')
            _reverse_encoder_saft    = _odo_cfg.get('reverse_encoder_saft')
            self._saft_motor.decoder = Decoder(
                orientation = Orientation.SAFT,
                gpio_a  = _motor_encoder_saft_a,
                gpio_b  = _motor_encoder_saft_b,
                reverse = _reverse_encoder_saft
            )
            self._log.info('configured {} motor encoder on pin {} and {} (reversed? {}).'.format(
                Orientation.SAFT, _motor_encoder_saft_a, _motor_encoder_saft_b, _reverse_encoder_saft))

        except Exception as e:
            self._log.error('{} raised configuring motors: {}'.format(type(e), e))
            self._pfwd_motor = None
            self._sfwd_motor = None
            self._paft_motor = None
            self._saft_motor = None
            raise
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _import_thunderborg(self, orientation):
        if self._motors_enabled:
            if orientation is Orientation.PORT:
                self._log.info('configure thunderborg & motors for {} orientation…')
                _thunderborg_address = self._config['kros'].get('motor').get('thunderborg_port_address')
            elif orientation is Orientation.STBD:
                _thunderborg_address = self._config['kros'].get('motor').get('thunderborg_stbd_address')
            else:
                raise Exception('expected PORT or STBD orientation.')
            self._log.debug('importing ThunderBorg for {} orientation at address 0x{:02X}…'.format(orientation.name, _thunderborg_address))
            _tb = None
            if self._enable_tb_scan:
                try:
                    if self._i2c_scanner.has_address([_thunderborg_address]):
                        self._log.debug('importing ThunderBorg at address 0x{:02X}…'.format(_thunderborg_address))
                        _tb = ThunderBorg(Level.INFO)  # create a new ThunderBorg object
                        _tb.i2cAddress = _thunderborg_address
                        self._log.debug('instantiated thunderborg.')
                    else:
                        raise Exception('unable to instantiate ThunderBorg [1].')
                    _tb.Init() # set the board up (checks the board is connected)
                    self._log.info('successfully instantiated ThunderBorg for orientation {} at address 0x{:02X}.'.format(
                            orientation.name, _thunderborg_address))
                    if not _tb.foundChip:
                        boards = ThunderBorg.ScanForThunderBorg()
                        if len(boards) == 0:
                            self._log.error('No ThunderBorg found, check you are attached :)')
                        else:
                            self._log.error('No ThunderBorg at address %02X, but we did find boards:' % (_tb.i2cAddress))
                            for board in boards:
                                self._log.info('    %02X (%d)' % (board, board))
                            self._log.error('If you need to change the I²C address change the setup line so it is correct, e.g. TB.i2cAddress = 0x{}'.format(
                                    boards[0]))
                        raise Exception('unable to instantiate ThunderBorg [2].')
                except OSError as e:
                    raise Exception('unable to instantiate ThunderBorg [3].\n{}', traceback.format_exc())
                except Exception as e:
                    raise Exception('{} error instantiating ThunderBorg [4]: {}\n{}'.format(type(e), e, traceback.format_exc()))
            else:
                try:
                    _tb = ThunderBorg(Level.INFO)  # create a new ThunderBorg object
                    _tb.i2cAddress = _thunderborg_address
                    _tb.Init() # set the board up (checks the board is connected)
                    self._log.info('successfully instantiated ThunderBorg for orientation {} at address 0x{:02X}.'.format(
                            orientation.name, _thunderborg_address))
                except Exception as e:
                    raise Exception('{} error instantiating ThunderBorg [5]: {}\n{}'.format(type(e), e, traceback.format_exc()))
            # initialise ThunderBorg
            _tb.SetLedShowBattery(True)
            self._log.info('getting battery reading…')
            voltage_in = 0
            try:
                # get battery voltage to determine max motor power
                # could be: Makita 12V or 18V power tool battery, 12V line supply
                voltage_in = _tb.GetBatteryReading()
                self._log.info('battery reading: {}'.format(voltage_in))
                if voltage_in is None:
                    raise OSError('cannot read battery voltage from ThunderBorg.')
                self._log.info('voltage in: {:>5.2f}V'.format(voltage_in))
            except OSError as e:
                voltage_in = self._system.get_battery_12v()
                self._log.info('voltage in: {:>5.2f}V'.format(voltage_in) + Fore.YELLOW + '(from ADS1015)')
            # maximum motor voltage
            _motor_voltage = self._config['kros'].get('motor').get('motor_voltage')
            self._log.info('voltage out: {:>5.2f}V'.format(_motor_voltage))
            if voltage_in < _motor_voltage:
                self._log.warning('battery voltage low ({:>5.2f}V).'.format(voltage_in))
            # set the power limits
            if _motor_voltage > voltage_in:
                self._max_power_ratio = 1.0
            else:
                self._max_power_ratio = _motor_voltage / float(voltage_in)
                self._log.info('voltage in: {:.2f}; motor voltage: {:.2f}; max_power_ratio: {:.2f}'.format(
                        voltage_in, _motor_voltage, self._max_power_ratio))
            # convert float to ratio format
            self._log.info('battery level: {:>5.2f}V; motor voltage: {:>5.2f}V; maximum power ratio: {}'.format(voltage_in, _motor_voltage, \
                    str(Fraction(self._max_power_ratio).limit_denominator(max_denominator=20)).replace('/',':')))
            return _tb

    def set_thunderborg_leds(self, enable):
        '''
        Turns the motor controller LEDs on or off.
        '''
        if self._port_tb:
            self._port_tb.SetLedShowBattery(enable)
            if not enable:
                self._port_tb.SetLeds(0.0, 0.0, 0.0) # black
        if self._stbd_tb:
            self._stbd_tb.SetLedShowBattery(enable)
            if not enable:
                self._stbd_tb.SetLeds(0.0, 0.0, 0.0) # black

    def get_thunderborg(self, orientation):
        '''
        Temporary: do not use this brain.
        '''
        if orientation is Orientation.PORT:
            return self._port_tb
        elif orientation is Orientation.STBD:
            return self._stbd_tb
        else:
            raise ValueError('expected a PORT or STBD orientation.')

    def get_motor(self, orientation):
        if orientation is Orientation.PFWD:
            return self._pfwd_motor
        elif orientation is Orientation.SFWD:
            return self._sfwd_motor
        elif orientation is Orientation.PAFT:
            return self._paft_motor
        elif orientation is Orientation.SAFT:
            return self._saft_motor

    def close(self):
        self.set_thunderborg_leds(True)
        # anything else?

#EOF
