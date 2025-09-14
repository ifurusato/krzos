#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-07-04
# modified: 2025-07-20

import itertools
from math import isclose
from colorama import Fore, Style
from logger import Logger, Level
from util import Util

class PID:
    '''
    A relatively generic PID controller.

    Args:
        id:      the identifier of the motor, used for the logger.
        config:  the application configuration
        level:   the log level
    '''
    def __init__(self, id, config, level=Level.INFO):
        self._log = Logger('pid-{}'.format(id), level=level)
        _cfg = config['kros']['pid']
        self._verbose     = _cfg['verbose']
        if self._verbose:
            self._log.info('initialising PID controller…')
        self._kp          = _cfg['kp']
        self._ki          = _cfg['ki']
        self._kd          = _cfg['kd']
        self._en_deadband = _cfg['enable_deadband']
        self._deadband    = _cfg['deadband']
        self._stop_threshold = _cfg['stop_threshold'] # below this we consider motor is stopped
        max_motor_speed   = config['kros']['motor_controller']['max_motor_speed']
        self._output_min  = -max_motor_speed
        self._output_max  = max_motor_speed
        self._setpoint    = 0.0
        self._last_error  = 0.0
        self._int_error   = 0.0
        self._p_term      = 0.0
        self._i_term      = 0.0
        self._d_term      = 0.0
        self._output      = 0.0
        self._log_freq    = 100
        self._counter     = itertools.count()
        if self._verbose:
            self._log.info(Fore.MAGENTA + "kp={:>5.3f}; ki={:>5.3f}; kd={:>5.3f}; limits: {}🠊 {}".format(
                    self._kp, self._ki, self._kd, self._output_min, self._output_max))
            self._log.info('ready.')

    @property
    def info(self):
        '''
        Returns a tuple containing the dynamic info: p_term, i_term, d_term, setpoint, output.
        '''
        return ( self._p_term, self._i_term, self._d_term, self._setpoint, self._output )

    @property
    def setpoint(self):
        return self._setpoint

    @setpoint.setter
    def setpoint(self, value):
        self._setpoint = float(value)

    @property
    def deadband_enabled(self):
        return self._en_deadband

    @property
    def deadband(self):
        return self._deadband

    def update(self, value, dt_us):
        dt_seconds = dt_us / 1_000_000.0 if dt_us > 0 else 0.000001
        error = self._setpoint - value
        # deadband: zero output and integral if error is within deadband
        if self._en_deadband and abs(error) < self._deadband:
            error = 0
            self._int_error = 0.0 # reset integral term inside PID logic
        # Proportional term
        self._p_term = self._kp * error
        # Derivative term
        self._d_term = self._kd * ((error - self._last_error) / dt_seconds) if dt_seconds > 0 else 0.0
        self._last_error = error
        # conditional integration
        output_estimate = self._p_term + self._ki * self._int_error + self._d_term
        output_will_saturate = not (self._output_min < output_estimate < self._output_max)
        if not output_will_saturate:
            if self._setpoint == 0.0:
                # deadband: logic for zero setpoint
                if isclose(value, 0.0, abs_tol=self._stop_threshold):
                    self._int_error = 0.0
                elif value > 0: # motor moving forward (positive RPM)
                    self._int_error = min(0.0, self._int_error + error * dt_seconds)
                else:
                    self._int_error = max(0.0, self._int_error + error * dt_seconds)
            else:
                self._int_error += error * dt_seconds
        # Integral term
        self._i_term = self._ki * self._int_error
        # Output (with normal clipping)
        self._output = self._p_term + self._i_term + self._d_term
        self._output = Util.clip(self._output, self._output_min, self._output_max)
        if self._verbose:
            if next(self._counter) % self._log_freq == 0:
                self._log.info(Style.DIM + "p={:.2f}, i={:.2f}, d={:.2f}, sp={:.2f}; o={:.2f}".format(
                        self._p_term, self._i_term, self._d_term, self._setpoint, self._output))
        return self._output

    def reset_derivative_state(self, current_value):
        '''
        Resets the _last_error to the current error.
        Call this when the controller resumes after a known, external pause
        to prevent an incorrect derivative term calculation.
        '''
        self._last_error = self._setpoint - current_value
        # Optionally, you could also reset the derivative term itself if desired:
        # self._d_term = 0.0

    def reset(self):
        self._last_error = 0.0
        self._int_error = 0.0

#EOF
