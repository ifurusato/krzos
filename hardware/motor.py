#!/usr/bin/env python3 # -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-01-18
# modified: 2025-11-19

import sys, itertools, time
from math import isclose
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.component import Component
from core.orientation import Orientation
from hardware.pid_controller import PIDController

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Motor(Component):
    '''
    Controls a motor that uses a Hall Effect encoder to determine the robot's
    velocity and distance traveled.

    This Motor class takes an input as speed (-1.0 to 1.0).

    This uses the kros:motor: section of the configuration.

    There are three reported speed properties of the motor:

        1. speed:           the scaled (0.0-1.0) current target speed of
                            the motor
        2. target speed:    the un-scaled target speed of the motor
        3. modified speed:  the target speed of the motor as modified by
                            any lambdas

    Args:
        config:        application configuration
        tb:            reference to the ThunderBorg motor controller
        orientation:   motor orientation
        level:         the logging level
    "
    :param level:       log level
    '''
    def __init__(self, config, tb, orientation, level=Level.INFO):
        if config is None:
            raise ValueError('null config argument.')
        if tb is None:
            raise ValueError('null thunderborg argument.')
        self._tb = tb
        self._orientation = orientation
        self._log = Logger('motor:{}'.format(orientation.label), level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._log.info('initialising {} motor with {} at address 0x{:02X} as motor controller…'.format(
                orientation.name, type(self._tb).__name__, self._tb.I2cAddress))
        # configuration
        _cfg = config['kros'].get('motor')
        self._scale_factor_closed = _cfg.get('scale_factor_closed') # constant converts target speed to motor speed (closed loop)
        self._scale_factor_open   = _cfg.get('scale_factor_open')   # constant converts target speed to motor speed (open loop)
        self._max_observed_speed = 0.0                              # the observed max forward speed
        _max_speed = 1.0
        self._motor_power_limit = _cfg.get('motor_power_limit') # power limit to motor
        self._log.info('motor power limit: {:<5.2f}'.format(self._motor_power_limit))
        self._power_clip = lambda n: ( -1.0 * self._motor_power_limit ) if n <= ( -1.0 * self._motor_power_limit ) \
                else self._motor_power_limit if n >= self._motor_power_limit \
                else n
        self._reverse_motor = _cfg.get('reverse_motor_{}'.format(orientation.label))
        self._log.info('reverse motor: {}'.format(self._reverse_motor))
        self._counter            = itertools.count()
        self.__callbacks         = []
        self.__max_applied_power = 0.0      # capture maximum power applied
        self.__max_power_ratio   = 0.0      # will be set by MotorConfigurer
        self.__target_speed      = 0.0      # the current target speed of the motor
        self.__modified_target_speed = 0.0  # ...as modified by any lambdas
        self._last_driving_power = 0.0      # last power setting for motor
        self._decoder            = None     # motor encoder
        self.__speed_lambdas     = {}
        self._verbose            = False
        self._allow_speed_multipliers = False
        self._pid_controller     = PIDController(config, self, level=level)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def velocity(self):
        '''
        Returns the current velocity of this motor.
        '''
        return self._pid_controller.velocity

    def get_velocity(self):
        '''
        DEPRECATED: Returns the current velocity of this motor.
        '''
        self._log.warning('get_velocity is DEPRECATED.')
        return self._pid_controller.velocity

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def current_power(self):
        '''
        Return the current power set on the motor.
        '''
        return self.get_current_power(settle_to_zero=False)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def add_callback(self, callback):
        '''
        Used by the Velocity class to obtain a callback on the motor loop.
        '''
#       self.__callbacks.append(callback)
        raise NotImplementedError('add_callback unsupported in Motor.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def orientation(self):
        '''
        Returns the orientation of this motor.
        '''
        return self._orientation

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def max_observed_speed(self):
        '''
        Returns the maximum observed forward speed.
        '''
        return self._max_observed_speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def add_speed_multiplier(self, name, lambda_function, exclusive=True):
        '''
        Adds a named speed multiplier to the dict of lambda functions. This
        replaces any existing lambda under the same name. The default with
        exclusive True is to clear any existing lambdas upon adding a new one.

        This is a function that alters the target speed as a multiplier.

        :param optional exclusive if True, clear all existing multipliers before adding
        '''
        if not self._allow_speed_multipliers:
            self._log.warning('speed multipliers are disabled.')
            return
        if exclusive:
            self.__speed_lambdas.clear()
        if name in self.__speed_lambdas:
            self._log.warning('motor already contains a \'{}\' lambda.'.format(name))
        else:
            self._log.info('adding \'{}\' lambda to motor {}…'.format(name, self.orientation.name))
            self.__speed_lambdas[name] = lambda_function

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def remove_speed_multiplier(self, name):
        '''
        Removes a named (or partial-named) speed multiplier from the dict of
        lambda functions.
        '''
        if not self._allow_speed_multipliers:
            self._log.warning('speed multipliers are disabled.')
            return
        for _name, _lambda in self.__speed_lambdas.copy().items():
            if name == _name or name in _name:
#               self._log.info('removing \'{}\' lambda from motor {}…'.format(_name, self.orientation.name))
                del self.__speed_lambdas[_name]

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def has_speed_multiplier(self, name):
        '''
        Returns true if a named speed multiplier exists in the dict of lambda functions.
        '''
        if not self._allow_speed_multipliers:
            self._log.warning('speed multipliers are disabled.')
            return False
        return name in self.__speed_lambdas

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def clear_speed_multipliers(self):
        '''
        Resets the speed multipliers to None, i.e., no function.
        '''
        if not self._allow_speed_multipliers:
            self._log.warning('speed multipliers are disabled.')
            return
        _count = len(self.__speed_lambdas)
        if _count > 0:
            self._log.info('clearing {:d} speed multipliers…'.format(_count))
            self.__speed_lambdas.clear()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def speed_multiplier_count(self):
        '''
        Resets the number of speed multipliers.
        '''
        if not self._allow_speed_multipliers:
            self._log.warning('speed multipliers are disabled.')
            return 0
        return len(self.__speed_lambdas)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def list_speed_multipliers(self):
        '''
        Lists the current speed multipliers to the log.
        '''
        if not self._allow_speed_multipliers:
            self._log.warning('speed multipliers are disabled.')
            return
        _count = len(self.__speed_lambdas)
        if _count == 0:
            self._log.info(Fore.GREEN + '  motor {} contains no lambdas.')
        else:
            self._log.info(Fore.GREEN + '  motor {} contains {:d} lambdas.'.format(self.orientation.name, _count))
            for _lambda in self.__speed_lambdas:
                self._log.info(Fore.GREEN + '    speed multiplier: {}'.format(_lambda))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def pid_controller(self):
        '''
        Returns the PID controller used by this motor.
        '''
        return self._pid_controller

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def decoder(self):
        return self._decoder

    @decoder.setter
    def decoder(self, decoder):
        self._decoder = decoder

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def modified_speed(self):
        '''
        Return the current lambda-modified target speed of the Motor.
        '''
        return self.__modified_target_speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def target_speed(self):
        '''
        Return the current target speed of the Motor.
        '''
        return self.__target_speed

    @target_speed.setter
    def target_speed(self, target_speed):
        '''
        Set the target speed of the Motor.
        '''
        if not isinstance(target_speed, float):
            raise ValueError('expected float, not {}'.format(type(target_speed)))
        self.__target_speed = target_speed
        if self._verbose:
            if ( target_speed < 0.0 ) or ( self.__target_speed < 0 ):
                self._log.info(Fore.RED + 'target speed: {:5.2f}'.format(self.__target_speed))
            else:
                self._log.info('target speed: {:5.2f}'.format(self.__target_speed))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def steps(self):
        '''
        Return the value of the step count from the motor encoder.
        '''
        return self._decoder.steps

    # max power rate ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def max_power_ratio(self):
        return self.__max_power_ratio

    @max_power_ratio.setter
    def max_power_ratio(self, max_power_ratio):
        self.__max_power_ratio = max_power_ratio
        self._log.info('set maximum power ratio: {:<5.2f}'.format(self.__max_power_ratio))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_stopped(self):
        '''
        Returns True if the motor is entirely stopped, or very nearly stopped.
        '''
        _current_power = self.get_current_power(settle_to_zero=False)
        if _current_power:
            value = isclose(_current_power, 0.0, abs_tol=1e-2)
            print('current: {:4.2f}; value: {}'.format(_current_power, value))
            return value
#           return isclose(_current_power, 0.0, abs_tol=1e-2)
        else:
            return True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_in_motion(self):
        '''
        Returns True if the motor is moving, i.e., if the current power
        setting of the motor is not equal to zero. Note that this returns
        False if the value is very close to zero.
        '''
#       return self.get_current_power() != 0.0
        return not isclose(self.get_current_power(settle_to_zero=False), 0.0, abs_tol=1e-2)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_moving_ahead(self):
        '''
        Returns True if the motor is moving ahead (forward), i.e., if the
        current power setting of the motor is greater than zero.
        '''
        return self.get_current_power() > 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_moving_astern(self):
        '''
        Returns True if the motor is moving astern (reverse), i.e., if the
        current power setting of the motor is less than zero.
        '''
        return self.get_current_power() < 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def update_target_speed(self):
        '''
        Align the current speed and the target speed and motor power. This
        is meant to be called regularly, in a loop.

        Returns the calculated motor speed after setting motor power to the value.
        '''
        if self.enabled:
            for callback in self.__callbacks:
                callback()
            # we start with the current value of the target speed
            self.__modified_target_speed = self.__target_speed
            _returned_value = 0.0
            if len(self.__speed_lambdas) > 0:
                for _name, _lambda in self.__speed_lambdas.items():
                    _returned_value = _lambda(self.__modified_target_speed)
                    if isinstance(_returned_value, str):
                        # lambda has returned indicator that it's finished
                        break
                    else:
                        _before_lambda_speed = self.__modified_target_speed
                        self.__modified_target_speed = _returned_value
                        if 'accum' in _name: # update stored target speed if lambda name includes "accum"
                            self.__target_speed = self.__modified_target_speed
                if isinstance(_returned_value, str):
                    # the lambda name was returned; this only should apply to brake, halt and stop.
                    _lambda_name = _returned_value
                    self.remove_speed_multiplier(_lambda_name)
                    if self._pid_controller.is_active:
                        self._pid_controller.set_speed(0.0)
                    return None
            if self._pid_controller.is_active:
                _motor_speed = self.__modified_target_speed * self._scale_factor_closed
                return self._pid_controller.set_speed(_motor_speed)
            else:
                _motor_speed = self.__modified_target_speed * self._scale_factor_open
                self.set_motor_power(_motor_speed)
                return _motor_speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_motor_power(self, target_power):
        '''
        Direct-drive the motor via a target power argument, whose value must be
        between -1.0 and 1.0, with the actual limits set by the max_power_ratio,
        which alters the value to match the power/motor voltage ratio.

        :param target_power:  the target motor power
        '''
        if target_power is None:
            raise ValueError('null target_power argument.')
        elif not self.enabled and target_power > 0.0: # though we'll let the power be set to zero
            raise Exception('motor {} not enabled.'.format(self.orientation.name))

#       if abs(target_power) > 1.0:
#           print(Fore.YELLOW + '⚠️  EXCESSIVE TARGET POWER for {} motor: {:.3f}'.format(self.orientation.name, target_power) + Style.RESET_ALL)

        _driving_power = round(self._power_clip(float(target_power * self.max_power_ratio)), 4) # round to 4 decimal

#       if abs(_driving_power) > 0.5:
#           print(Fore.YELLOW + '⚠️  EXCESSIVE DRIVING POWER for {} motor: {:.3f}'.format(self.orientation.name, target_power) + Style.RESET_ALL)

        _is_zero = isclose(_driving_power, 0.0, abs_tol=0.05) # deadband
        if self._reverse_motor:
            _driving_power *= -1.0
        if self._orientation is Orientation.PFWD:
            if _is_zero:
                self._log.debug(Fore.RED + Style.DIM + 'target power {:5.2f} converted to driving power {:<5.2f} for PFWD motor.'.format(target_power, _driving_power))
                self._tb.SetMotor2(0.0)
            else:
                self._log.debug(Fore.RED   + 'target power {:5.2f} converted to driving power {:<5.2f} for PFWD motor.'.format(target_power, _driving_power))
                self._tb.SetMotor2(_driving_power)
        elif self._orientation is Orientation.SFWD:
            if _is_zero:
                self._log.debug(Fore.GREEN + Style.DIM + 'target power {:5.2f} converted to driving power {:<5.2f} for SFWD motor.'.format(target_power, _driving_power))
                self._tb.SetMotor2(0.0)
            else:
                self._log.debug(Fore.GREEN + 'target power {:5.2f} converted to driving power {:<5.2f} for SFWD motor.'.format(target_power, _driving_power))
                self._tb.SetMotor2(_driving_power)
        elif self._orientation is Orientation.PAFT:
            if _is_zero:
                self._log.debug(Fore.RED + Style.DIM + 'target power {:5.2f} converted to driving power {:<5.2f} for PAFT motor.'.format(target_power, _driving_power))
                self._tb.SetMotor1(0.0)
            else:
                self._log.debug(Fore.RED   + 'target power {:5.2f} converted to driving power {:<5.2f} for PAFT motor.'.format(target_power, _driving_power))
                self._tb.SetMotor1(_driving_power)
        elif self._orientation is Orientation.SAFT:
            if _is_zero:
                self._log.debug(Fore.GREEN + Style.DIM + 'target power {:5.2f} converted to driving power {:<5.2f} for SAFT motor.'.format(target_power, _driving_power))
                self._tb.SetMotor1(0.0)
            else:
                self._log.debug(Fore.GREEN + 'target power {:5.2f} converted to driving power {:<5.2f} for SAFT motor.'.format(target_power, _driving_power))
                self._tb.SetMotor1(_driving_power)
        else:
            raise ValueError('cannot set speed via side.')

        self._last_driving_power = _driving_power
        # keep track of highest-applied target power
        self.__max_applied_power = max(abs(target_power), self.__max_applied_power)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def tb(self):
        '''
        For diagnostics only; not to be used directly.
        '''
        return self._tb

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_current_power(self, settle_to_zero=True):
        '''
        Makes a best attempt at getting the current power value from the motors.
        Note that the motor controller does not report absolute zero when the
        motors are not moving, but a very small positive or negative value. In
        this case we report 0.0.

        Note: a side effect of calling this method is that if it is determined
        that the motor power is close to zero we will therefore set the power
        level to zero. We could have done this elsewhere but it would have been
        more complicated. Because the motors can't move with so little power
        it's just a waste of battery power to do so.

        If you want to avoid the side effect, call this method with the
        'settle_to_zero' argument as False (default is True).
        '''
        _value = None
        count = 0
        if self._orientation is Orientation.PAFT or self._orientation is Orientation.SAFT:
            while _value == None and count < 20:
                count += 1
                _value = self._tb.GetMotor1()
            if settle_to_zero and (_value == None or isclose(_value, 0.0, abs_tol=1e-1)):
                _value = 0.0
                self._tb.SetMotor1(_value)
        elif self._orientation is Orientation.PFWD or self._orientation is Orientation.SFWD:
            while _value == None and count < 20:
                count += 1
                _value = self._tb.GetMotor2()
            if settle_to_zero and (_value == None or isclose(_value, 0.0, abs_tol=1e-2)):
                _value = 0.0
                self._tb.SetMotor2(_value)
        return _value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def last_power(self):
        '''
        Returns the last power setting for motor.
        '''
        return self._last_driving_power

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        '''
        Stops the motor immediately.
        '''
        self.target_speed = 0.0

        if self._orientation is Orientation.PAFT or self._orientation is Orientation.SAFT:
            self._tb.SetMotor1(0.0)
        elif self._orientation is Orientation.PFWD or self._orientation is Orientation.SFWD:
            self._tb.SetMotor2(0.0)
        else:
            raise ValueError('unrecognised orientation.')
        self._log.info('{} motor stopped.'.format(self._orientation.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def off(self):
        '''
        Stops the motor entirely.
        '''
        if self._orientation is Orientation.PFWD or self._orientation is Orientation.PAFT:
            self._tb.SetMotor1Off()
        elif self._orientation is Orientation.SFWD or self._orientation is Orientation.SAFT:
            self._tb.SetMotor2Off()
        else:
            raise ValueError('unrecognised orientation.')
        self._log.info('{} motor off.'.format(self._orientation.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def is_enabled(self):
        return self.enabled

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        if not self.enabled:
            Component.enable(self)
        self._log.info('enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        if self.enabled:
            Component.disable(self)
            self.pid_controller.disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')
        self.off() # in any case

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        # just do it anyway
        self.stop()
        if self.enabled:
            self.disable()
        if self.__max_applied_power > 0.0:
            self._log.info('on closing, maximum applied power: {:>5.2f}'.format(self.__max_applied_power))
        self._log.info('closed.')

#EOF
