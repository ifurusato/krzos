#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-05
# modified: 2025-10-13
#

import sys, traceback
import time
import statistics
import itertools
from math import isclose
from threading import Thread
from colorama import init, Fore, Style
init()

from core.component import Component
from core.direction import Direction
from core.orientation import Orientation
from core.rate import Rate
from core.rotation import Rotation
from core.steering_mode import SteeringMode
from core.logger import Logger, Level
from hardware.i2c_scanner import I2CScanner
from hardware.irq_clock import IrqClock
from hardware.motor_configurer import MotorConfigurer
from hardware.slew_rate import SlewRate

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MotorController(Component):
    NAME = 'motor-ctrl'
    STOP_LAMBDA_NAME  = "__stop_accum"  # TEMP moved to StopHandler
    HALT_LAMBDA_NAME  = "__halt_accum"  # TEMP moved to StopHandler
    BRAKE_LAMBDA_NAME = "__brake_accum" # TEMP moved to StopHandler
    PORT_CW_ROTATE_LAMBDA_NAME   = "__port_rotate_cw_steering"
    STBD_CW_ROTATE_LAMBDA_NAME   = "__stbd_rotate_cw_steering"

    PORT_CCW_ROTATE_LAMBDA_NAME  = "__port_rotate_ccw_steering"
    STBD_CCW_ROTATE_LAMBDA_NAME  = "__stbd_ccw_rotate_steering"

    FWD_REPOSITION_ROTATE_LAMBDA_NAME = "__fwd_reposition_rotate_steering"
    REV_REPOSITION_ROTATE_LAMBDA_NAME = "__aft_reposition_rotate_steering"

    FWD_REPOSITION_RETURN_LAMBDA_NAME = "__fwd_reposition_return_steering"
    AFT_REPOSITION_RETURN_LAMBDA_NAME = "__aft_reposition_return_steering"

    '''
    The controller for 4 motors:

        pfwd: Port-Forward        sfwd: Starboard-Forward
        paft: Port-Aft            saft: Starboard-Aft

    This permits speed change lambda functions to be added to the motors
    to alter their behaviour, such as coming to a halt.

    The contract that speed change lambdas have is that they pass a lambda
    function and the target speed as arguments, and return an altered target
    speed OR the name of the originating lambda, in the case where the lambda
    has completed and should no longer be processed (i.e., it should be removed).
    The passed lambda returns True only when all motors are clear of speed
    change lambdas.

    By default, lambdas alter the target speed only for that 20Hz cycle, unless
    the lambda has 'accum' in its name, which causes changes to accumulate. The
    latter are used for stopping modes.

    :param config:            the YAML based application configuration
    :param external_clock     the optional external clock (in lieu of a thread loop)
    :param suppressed         if True the controller is suppressed
    :param enabled            if True the controller is enabled upon instantiation
    :param level:             the logging Level
    '''
    def __init__(self, config, external_clock=None, suppressed=False, enabled=False, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._log = Logger(MotorController.NAME, level)
        Component.__init__(self, self._log, suppressed, enabled)
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['kros'].get('motor_controller')
        _i2c_scanner = I2CScanner(config=config, i2c_bus_number=1, i2c_bus=None, level=level)
        # config ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._verbose        = _cfg.get('verbose')
        self._loop_freq_hz   = _cfg.get('loop_freq_hz') # main loop frequency
        self._loop_delay_sec = 1 / self._loop_freq_hz
        self._accel_step     = _cfg.get('accel_step', 0.02)
        self._accel_step_delay_ms = _cfg.get('accel_step_delay_ms', 20)
        self._decel_step     = _cfg.get('decel_step', 0.02)
        self._decel_step_delay_ms = _cfg.get('decel_step_delay_ms', 20)
        self._halt_slew_rate = SlewRate.from_string(_cfg.get('halt_rate'))
        self._log.info('halt rate: {}'.format(self._halt_slew_rate.name))
        # slew limiters are on motors, not here
        self._slew_limiter_enabled = config['kros'].get('motor').get('enable_slew_limiter')
        _create_ext_clock    = _cfg.get('create_external_clock')
        if external_clock:
            self._external_clock = external_clock
        elif _create_ext_clock:
            self._log.info('creating IRQ clock…')
            self._external_clock = IrqClock(config, level=Level.INFO)
            self._external_clock.enable()
        if self._external_clock is None:
            self._rate = Rate(self._loop_freq_hz, Level.ERROR)
        else:
            self._rate = None
        self._log.info('loop frequency: {}Hz ({:4.2f}s)'.format(self._loop_freq_hz, self._loop_delay_sec))
        # motor controller ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._all_motors     = []
        _motor_configurer = MotorConfigurer(config, _i2c_scanner, motors_enabled=True, level=level)
        self._pfwd_motor     = _motor_configurer.get_motor(Orientation.PFWD)
        self._sfwd_motor     = _motor_configurer.get_motor(Orientation.SFWD)
        self._paft_motor     = _motor_configurer.get_motor(Orientation.PAFT)
        self._saft_motor     = _motor_configurer.get_motor(Orientation.SAFT)
        self._all_motors     = self._get_motors()
        self._is_daemon      = True
        self._loop_thread    = None
        self._loop_enabled   = False
        self._event_counter  = itertools.count()
        self._motor_loop_callback = None
        self._graceful_stop  = True
        # speed and changes to speed ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        # --- Begin new intent vector registry ---
        self._intent_vectors = {}  # {name: lambda}
        # --- End new intent vector registry ---
        self._vector_functions = {
            Orientation.PFWD: {},
            Orientation.SFWD: {},
            Orientation.PAFT: {},
            Orientation.SAFT: {},
        }
        self.__callback      = None
        self._is_stopped     = True # used to capture state transitions
        self.__state_change_callbacks = [] # anyone who wants to be informed if the robot is moving or stopped
        self._rotation_speed_multiplier = 1.0
        # lambdas to alter direction to comply with rotation
        self._port_cw_rotate_lambda  = lambda speed: speed * self._rotation_speed_multiplier
        self._stbd_cw_rotate_lambda  = lambda speed: -1.0 * speed * self._rotation_speed_multiplier
        self._port_ccw_rotate_lambda = lambda speed: -1.0 * speed * self._rotation_speed_multiplier
        self._stbd_ccw_rotate_lambda = lambda speed: speed * self._rotation_speed_multiplier
        # lambdas for rotating motors
        self._fwd_reposition_rotate_lambda = lambda speed: speed * self._rotation_speed_multiplier
        self._rev_reposition_rotate_lambda = lambda speed: -1.0 * speed * self._rotation_speed_multiplier
        self._theta          = 0.0
        self._stbd_speed     = 0.0
        self._port_speed     = 0.0
        self._differential_drive_mode      = False
        _max_speed           = _cfg.get('max_speed') # max speed of motors (0-100)
        _min_speed           = -1 * _max_speed
        self._log.info('motor speed clamped at {} to {}.'.format(_min_speed, _max_speed))
        self._clamp          = lambda n: max(min(_max_speed, n), _min_speed)
        self._print_info_done = False
        _closed_loop         = _cfg.get('closed_loop')
        self.set_closed_loop(_closed_loop)
        # finish up…
        self._log.info('ready with {} motors.'.format(len(self._all_motors)))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def add_intent_vector(self, name, vector_lambda):
        '''
        Register a behaviour's intent vector lambda.
        The lambda should return (vx, vy) or (vx, vy, omega).
        '''
        self._intent_vectors[name] = vector_lambda
        self._log.info('added intent vector: {}'.format(name))

    def remove_intent_vector(self, name):
        '''
        Remove a behaviour's intent vector lambda.
        '''
        if name in self._intent_vectors:
            del self._intent_vectors[name]
            self._log.info('removed intent vector: {}'.format(name))

    def blend_intent_vectors(self):
        '''
        Blends all registered intent vectors, returning a single (vx, vy[, omega]) tuple.
        '''
        vectors = [fn() for fn in self._intent_vectors.values()]
        if not vectors:
            return (0.0, 0.0)
        dim = len(vectors[0])
        sum_vector = [0.0] * dim
        for v in vectors:
            for i in range(dim):
                sum_vector[i] += v[i]
        avg_vector = tuple(s / len(vectors) for s in sum_vector)
        return avg_vector

    def set_differential_drive(self, enabled):
        '''
        In differential drive mode, the aft wheels are set to the same speed as the fore.
        '''
        self._differential_drive_mode = enabled

    def is_closed_loop(self):
        return self._closed_loop

    def set_closed_loop(self, closed_loop):
        '''
        Set the motor controller to open or closed loop.
        '''
        self._closed_loop = closed_loop
        if self._closed_loop:
            for _motor in self._all_motors:
                _motor.pid_controller.enable()
            self._log.info('motor control: ' + Fore.GREEN + 'closed loop')
        else:
            for _motor in self._all_motors:
                _motor.pid_controller.disable()
            self._log.info('motor control: ' + Fore.GREEN + 'open loop')

    def get_mean_speed(self, orientation):
        '''
        Return the mean speed of the motors as characterised by the
        orientation argument, PORT, STBD, or CNTR as a proxy for all
        motors.
        '''
        _speeds = []
        if orientation is Orientation.CNTR:
            for _motor in self._get_motors():
                if len(_speeds) < 2:
                    _mean = 0.0
                else:
                    _mean = statistics.fmean(_speeds)
                self._log.info('averaging with speed of {} motor: modified={:.2f}; speed={:.2f}; target={:.2f}; mean: {:4.2f}'.format(_motor.orientation.name,
                        _motor.modified_speed, _motor.target_speed, _motor.target_speed, _mean))
                _speeds.append(_motor.target_speed)
            return statistics.fmean(_speeds)
        elif orientation is Orientation.PORT:
            return statistics.mean([self._pfwd_motor.modified_speed, self._paft_motor.modified_speed])
        elif orientation is Orientation.STBD:
            return statistics.mean([self._sfwd_motor.modified_speed, self._saft_motor.modified_speed])

    def get_motors(self):
        '''
        Returns a list containing all instantiated motors.
        This includes only instantiated motors.
        '''
        return self._all_motors

    def _get_motors(self):
        '''
        Returns a list of all extant motors.
        '''
        _list = []
        if self._pfwd_motor:
            _list.append(self._pfwd_motor)
        if self._sfwd_motor:
            _list.append(self._sfwd_motor)
        if self._paft_motor:
            _list.append(self._paft_motor)
        if self._saft_motor:
            _list.append(self._saft_motor)
        return _list

    def get_motor(self, orientation):
        '''
        Returns the motor corresponding to the orientation.
        '''
        if orientation is Orientation.PFWD:
            return self._pfwd_motor
        elif orientation is Orientation.SFWD:
            return self._sfwd_motor
        elif orientation is Orientation.PAFT:
            return self._paft_motor
        elif orientation is Orientation.SAFT:
            return self._saft_motor
        else:
            raise Exception('unsupported orientation.')

    def enable(self):
        '''
        Enables the motors. This issues a warning if already enabled, but
        no harm is done in calling it repeatedly.
        '''
        if self.enabled:
            self._log.debug('already enabled.')
        else:
            self._log.info(Style.BRIGHT + 'enabling motor controller…')
            Component.enable(self)
            if self._external_clock:
                self._external_clock.add_callback(self._external_callback_method)
                for _motor in self._all_motors:
                    _motor.enable()
            elif not self.loop_is_running:
                self._start_loop()
            self._log.info('enabled.')

    def _start_loop(self):
        '''
        Start the loop Thread.

        If we're using an external clock, calling this method throws an exception.
        '''
        self._log.info('start motor control loop…')
        if not self.enabled:
            raise Exception('not enabled.')
        if self.loop_is_running:
            self._log.warning('loop already running.')
        elif self._loop_thread is None:
            if self._external_clock:
                raise Exception('cannot use thread-based loop: external clock enabled.')
            self._loop_enabled = True
            self._loop_thread = Thread(name='motor_loop', target=MotorController._motor_loop, args=[self, lambda: self._loop_enabled], daemon=self._is_daemon)
            self._loop_thread.start()
            self._log.info('loop enabled.')
        else:
            raise Exception('cannot enable loop: thread already exists.')

    def _stop_loop(self):
        '''
        Stop the motor control loop.
        '''
        if self.loop_is_running:
            self._loop_enabled = False
            self._loop_thread  = None
            self._log.info('stopped motor control loop.')
        else:
            self._log.warning('motor control loop already disabled.')

    @property
    def loop_is_running(self):
        '''
        Returns true if using an external clock or if the loop thread is alive.
        '''
        return self._loop_enabled and self._loop_thread != None and self._loop_thread.is_alive()

    def set_motor_loop_callback(self, callback):
        '''
        Sets the optional callback executed upon each motor loop. This should
        return relatively quickly as to not affect the motor loop performance.
        '''
        self._motor_loop_callback = callback

    def _motor_tick(self):
        '''
        The shared logic for a single motor control loop iteration, used by
        both _motor_loop() and _external_callback_method(). Incorporates intent vector 
        blending and full Mecanum wheel kinematic mapping and normalization.
        '''
        intent = self.blend_intent_vectors()
        if len(intent) == 2:
            vx, vy = intent
            omega = 0.0
        elif len(intent) == 3:
            vx, vy, omega = intent
        else:
            vx = vy = omega = 0.0
        # Mecanum equations
        pfwd = vy + vx + omega
        sfwd = vy - vx - omega
        paft = vy - vx + omega
        saft = vy + vx - omega
        speeds = [pfwd, sfwd, paft, saft]
        max_abs = max(abs(s) for s in speeds)
        if max_abs > 1.0:
            speeds = [s / max_abs for s in speeds]
        self.set_motor_speed(Orientation.PFWD, speeds[0])
        self.set_motor_speed(Orientation.SFWD, speeds[1])
        self.set_motor_speed(Orientation.PAFT, speeds[2])
        self.set_motor_speed(Orientation.SAFT, speeds[3])
        if self._differential_drive_mode:
            _port_motor_power = self._pfwd_motor.update_target_speed()
            _stbd_motor_power = self._sfwd_motor.update_target_speed()
            self._paft_motor.set_motor_power(_port_motor_power)
            self._saft_motor.set_motor_power(_stbd_motor_power)
        else:
            for _motor in self._all_motors:
                _motor.update_target_speed()
        if self._verbose: # print stats
            _count = next(self._event_counter)
            self.print_info(_count)
        self._state_change_check()
        # execute any callback here…
        if self._motor_loop_callback is not None:
            self._motor_loop_callback()

    def _motor_loop(self, f_is_enabled):
        '''
        The motors loop, which executes while the flag argument lambda is True.
        '''
        self._log.info('loop start.')
        try:
            while f_is_enabled():
                self._motor_tick()
                self._rate.wait()
        except Exception as e:
            self._log.error('error in loop: {}\n{}'.format(e, traceback.format_exc()))
        finally:
            self._log.info(Fore.GREEN + 'exited motor control loop.')

    def _external_callback_method(self):
        '''
        The callback called by the external clock as an alternative to the
        asyncio _loop() method.
        '''
        if self.enabled:
            self._motor_tick()
        else:
            pass

    def accelerate(self, target_speed, enabled=None):
        '''
        Gradually accelerate from the current speed to the target speed
        using the provided slew rate (step size and delay).

        Args:
            target_speed (float):  Desired lower speed.
            enabled (callable):    A function or lambda returning a bool that
                                   determines whether deceleration should continue.
        '''
        if not callable(enabled):
            raise TypeError("argument 'enabled' must be a lambda function.")
        port_speed = self.get_mean_speed(Orientation.PORT)
        stbd_speed = self.get_mean_speed(Orientation.STBD)
        if port_speed < 0 or stbd_speed < 0:
            self._log.warning('Cannot accelerate: robot not stopped or moving forward.')
            return
        elif port_speed >= target_speed or stbd_speed >= target_speed:
            self._log.warning('Cannot accelerate: already at or above target speed.')
            return
        while enabled() and (port_speed < target_speed or stbd_speed < target_speed):
            self.set_differential_speeds(
                min(port_speed + self._accel_step, target_speed),
                min(stbd_speed + self._accel_step, target_speed))
            time.sleep(self._accel_step_delay_ms / 1000.0)
            port_speed = self.get_mean_speed(Orientation.PORT)
            stbd_speed = self.get_mean_speed(Orientation.STBD)
            if isclose(port_speed, target_speed, rel_tol=1e-3, abs_tol=1e-3) and \
               isclose(stbd_speed, target_speed, rel_tol=1e-3, abs_tol=1e-3):
                break

    def decelerate(self, target_speed=0.0, step=0.02, step_delay_ms=20, enabled=None):
        '''
        Gradually decelerate from the current speed to the target speed
        using the provided slew rate (step size and delay).

        Args:
            target_speed (float):  Desired lower speed.
            step (int):            How much to decrease the motor speed upon each step.
            step_delay_ms (int):   The delay in milliseconds for each step.
            enabled (callable):    A function or lambda returning a bool that
                                   determines whether deceleration should continue.
        '''
        if not callable(enabled):
            raise TypeError("Argument 'enabled' must be a callable (e.g., a lambda returning a bool).")
        port_speed = self.get_mean_speed(Orientation.PORT)
        stbd_speed = self.get_mean_speed(Orientation.STBD)
        if port_speed < 0 or stbd_speed < 0:
            self._log.warning('Cannot decelerate: robot not stopped or traveling backward.')
            return
        if port_speed <= target_speed or stbd_speed <= target_speed:
            self._log.warning('Cannot decelerate: already at or below target speed.')
            return
        while enabled() and (port_speed > target_speed or stbd_speed > target_speed):
            self.set_differential_speeds(
                max(port_speed - self._decel_step, target_speed),
                max(stbd_speed - self._decel_step, target_speed))
            time.sleep(self._decel_step_delay_ms / 1000.0)
            port_speed = self.get_mean_speed(Orientation.PORT)
            stbd_speed = self.get_mean_speed(Orientation.STBD)
            if isclose(port_speed, target_speed, rel_tol=1e-3, abs_tol=1e-3) and \
               isclose(stbd_speed, target_speed, rel_tol=1e-3, abs_tol=1e-3):
                break

    def add_state_change_callback(self, callback):
        '''
        Add the callback to the list, for changes to stop/moving state.
        '''
        self.__state_change_callbacks.append(callback)

    def _state_change_check(self):
        '''
        Check if the stopped/moving state has changed since the last call.
        '''
        _currently_is_stopped = self.is_stopped
        if _currently_is_stopped != self._is_stopped:
            for _callback in self.__state_change_callbacks:
                _callback()
        self._is_stopped = _currently_is_stopped

    def is_stopping(self):
        _lambda_count = 0
        for _motor in self._all_motors:
            _lambda_count += _motor.speed_multiplier_count
        return _lambda_count > 0

    @property
    def is_stopped(self):
        '''
        Returns True if the speed of all motors is zero, False if any are
        moving, i.e., if the motor power of any motor is greater than zero.
        '''
        for _motor in self._all_motors:
            if not _motor.is_stopped:
                return False
        return True

    def set_speeds(self, pfwd_speed, sfwd_speed, paft_speed, saft_speed):
        '''
        Set all four motors speeds.
        '''
        if self._verbose:
            self._log.info(Fore.MAGENTA + '🎀 set speeds: {:4.2f} | {:4.2f} | {:4.2f} | {:4.2f}'.format(pfwd_speed, sfwd_speed, paft_speed, saft_speed))
        self.set_motor_speed(Orientation.PFWD, pfwd_speed)
        self.set_motor_speed(Orientation.SFWD, sfwd_speed)
        self.set_motor_speed(Orientation.PAFT, paft_speed)
        self.set_motor_speed(Orientation.SAFT, saft_speed)

    def set_differential_speeds(self, port_speed, stbd_speed):
        if self._verbose:
            self._log.info(Fore.MAGENTA + '🎀 set diff speeds: {:4.2f} | {:4.2f}'.format(port_speed, stbd_speed))
        self.set_speeds(port_speed, stbd_speed, port_speed, stbd_speed)

    def get_differential_speeds(self):
        '''
        Returns a tuple containing the port and starboard motors speeds.
        '''
        return self._port_speed, self._stbd_speed

    def set_speed(self, orientation, value):
        '''
        Sets the speed of all motors associated with the port or starboard
        orientation.

        For testing we permit a single SAFT motor, which otherwise is not a
        particularly good idea.
        '''
        _speed = self._clamp(value)
        if self._verbose:
            self._log.info(Fore.MAGENTA + '🎀 set {} speed: {:4.2f} clamped to {:4.2f}'.format(orientation.name, value, _speed))
        if orientation is Orientation.ALL:
            self._port_speed = _speed
            self.set_motor_speed(Orientation.PFWD, _speed)
            self.set_motor_speed(Orientation.PAFT, _speed)
            self._stbd_speed = _speed
            self.set_motor_speed(Orientation.SFWD, _speed)
            self.set_motor_speed(Orientation.SAFT, _speed)
        elif orientation is Orientation.PORT:
            self._port_speed = _speed
            self.set_motor_speed(Orientation.PFWD, _speed)
            self.set_motor_speed(Orientation.PAFT, _speed)
        elif orientation is Orientation.STBD:
            self._stbd_speed = _speed
            self.set_motor_speed(Orientation.SFWD, _speed)
            self.set_motor_speed(Orientation.SAFT, _speed)
        elif orientation is Orientation.PFWD:
            self.set_motor_speed(Orientation.PFWD, _speed)
        elif orientation is Orientation.SFWD:
            self.set_motor_speed(Orientation.SFWD, _speed)
        elif orientation is Orientation.PAFT:
            self.set_motor_speed(Orientation.PAFT, _speed)
        elif orientation is Orientation.SAFT:
            self.set_motor_speed(Orientation.SAFT, _speed)
        else:
            raise Exception('unsupported orientation {}'.format(orientation.name))

    def set_motor_speed(self, orientation, target_speed):
        '''
        A convenience method that sets the target speed and motor power of
        the specified motor, as identified by Orientation. Accepts either
        ints or floats between -1.0 and 1.0.

        This precedes setting the target speed by reseting the Velocity
        step count, otherwise it would be polluted by previous data.

        When the motor controller is disabled any calls to this method will
        override the target speed argument and set it to zero.

        The values are scaled on the motor side, not here.
        '''
        if not self.enabled:
            self._log.error('motor controller not enabled.')
            target_speed = 0.0
        if isinstance(target_speed, int):
            raise ValueError('expected target speed as float not int: {:d}'.format(target_speed))
        if not isinstance(target_speed, float):
            raise ValueError('expected float, not {}'.format(type(target_speed)))
        if self._verbose:
            self._log.info(Fore.MAGENTA + '🎀 set {} motor speed: {:5.2f}'.format(orientation.name, target_speed))
        if orientation is Orientation.PFWD and self._pfwd_motor.enabled:
            self._pfwd_motor.target_speed = target_speed
        elif orientation is Orientation.SFWD and self._sfwd_motor.enabled:
            self._sfwd_motor.target_speed = target_speed
        elif orientation is Orientation.PAFT and self._paft_motor.enabled:
            self._paft_motor.target_speed = target_speed
        elif orientation is Orientation.SAFT and self._saft_motor.enabled:
            self._saft_motor.target_speed = target_speed
        else:
            raise TypeError('expected a motor orientation, not {}'.format(orientation))

    def add_lambda(self, orientation, lambda_name, lambda_function, exclusive=True):
        '''
        Adds the named lambda to the specified motor, replacing any others if exclusive.
        '''
        if orientation is Orientation.PFWD:
            self._pfwd_motor.add_speed_multiplier(lambda_name, lambda_function, exclusive)
        elif orientation is Orientation.SFWD:
            self._sfwd_motor.add_speed_multiplier(lambda_name, lambda_function, exclusive)
        elif orientation is Orientation.PAFT:
            self._paft_motor.add_speed_multiplier(lambda_name, lambda_function, exclusive)
        elif orientation is Orientation.SAFT:
            self._saft_motor.add_speed_multiplier(lambda_name, lambda_function, exclusive)

    def add_vector(self, orientation, vector_name, vector_function, exclusive=True):
        '''
        Adds a named vector function to the specified motor, replacing any others if
        exclusive. Behaviours should use this method to register their movement vector
        functions for blending.

        Only functions that return a tuple or list (i.e., representing a vector such
        as (x, y)) are permitted. This is enforced at registration time.

        :param orientation:     the motor orientation (PFWD, SFWD, PAFT, SAFT)
        :param vector_name:     name for the vector function (for update/removal)
        :param vector_function: function returning the desired vector (tuple/list) for this motor
        :param exclusive:       if True, clears other vector functions for this motor before adding
        :raises TypeError:      if vector_function does not return a tuple or list
        '''
        result = vector_function()
        if not isinstance(result, (tuple, list)):
            raise TypeError("add_vector: vector_function must return a tuple or list representing a vector, not '{}'.".format(type(result)))
        if orientation not in self._vector_functions:
            raise Exception('unsupported orientation in add_vector: {}'.format(orientation))
        if exclusive:
            self._vector_functions[orientation].clear()
        self._vector_functions[orientation][vector_name] = vector_function
        self._log.info('added vector {} to {} motor{}'.format(
            vector_name, orientation.name,
            ' (exclusive)' if exclusive else ''
        ))

    def remove_vector(self, orientation, vector_name):
        '''
        Removes the named vector function from the specified motor.
        '''
        if orientation in self._vector_functions and vector_name in self._vector_functions[orientation]:
            del self._vector_functions[orientation][vector_name]
            self._log.info('removed vector {} from {} motor'.format(vector_name, orientation.name))

    def blend_vectors(self, orientation):
        '''
        Blends all registered vector functions for the specified motor orientation.

        Each registered function must return a tuple or list (vector-style). The blend is
        computed as the average of all vectors returned by the functions. If no functions
        are registered, returns (0.0, 0.0).

        :param orientation: the motor orientation (PFWD, SFWD, PAFT, SAFT)
        :return:            the blended vector as a tuple
        :raises TypeError:  if any registered function does not return a tuple or list
        '''
        if orientation not in self._vector_functions:
            raise Exception('unsupported orientation in blend_vectors: {}'.format(orientation))
        vector_funcs = self._vector_functions[orientation].values()
        if not vector_funcs:
            return (0.0, 0.0)
        speeds = [func() for func in vector_funcs]
        if not all(isinstance(s, (tuple, list)) for s in speeds):
            raise TypeError("blend_vectors: all vector functions must return tuple or list (vector-style).")
        n = len(speeds)
        dim = len(speeds[0])
        sums = [0.0] * dim
        for v in speeds:
            for i in range(dim):
                sums[i] += v[i]
        avgs = tuple(s / n for s in sums)
        return avgs

    def rotate(self, rotation):
        '''
        This sets the rotation callback to set the motors for rotation
        in the prescribed direction. This adds a lambda to the motors
        to alter their rotation to comply with the rotation direction.
        '''
        if rotation is None:
            raise ValueError('rotation argument not provided.')
        self.reset_rotating()
        if rotation is Rotation.STOPPED:
            self._log.info('rotation: none')
            self._reset_slew_rate()
            self.set_motor_speed(Orientation.PFWD, 0.0)
            self.set_motor_speed(Orientation.SFWD, 0.0) 
            self.set_motor_speed(Orientation.PAFT, 0.0)
            self.set_motor_speed(Orientation.SAFT, 0.0)
        elif rotation is Rotation.CLOCKWISE:
            self._set_slew_rate(SlewRate.FASTEST)
            self._log.info('rotate clockwise, rotation speed multiplier: {:5.2f}'.format(self._rotation_speed_multiplier))
            self._pfwd_motor.add_speed_multiplier(MotorController.PORT_CW_ROTATE_LAMBDA_NAME, self._port_cw_rotate_lambda, True)
            self._paft_motor.add_speed_multiplier(MotorController.PORT_CW_ROTATE_LAMBDA_NAME, self._port_cw_rotate_lambda, True)
            self._sfwd_motor.add_speed_multiplier(MotorController.STBD_CW_ROTATE_LAMBDA_NAME, self._stbd_cw_rotate_lambda, True)
            self._saft_motor.add_speed_multiplier(MotorController.STBD_CW_ROTATE_LAMBDA_NAME, self._stbd_cw_rotate_lambda, True)
        elif rotation is Rotation.COUNTER_CLOCKWISE:
            self._set_slew_rate(SlewRate.FASTEST)
            self._log.info('rotate counter-clockwise, rotation speed multiplier: {:5.2f}'.format(self._rotation_speed_multiplier))
            self._pfwd_motor.add_speed_multiplier(MotorController.PORT_CCW_ROTATE_LAMBDA_NAME, self._port_ccw_rotate_lambda, True)
            self._paft_motor.add_speed_multiplier(MotorController.PORT_CCW_ROTATE_LAMBDA_NAME, self._port_ccw_rotate_lambda, True)
            self._sfwd_motor.add_speed_multiplier(MotorController.STBD_CCW_ROTATE_LAMBDA_NAME, self._stbd_ccw_rotate_lambda, True)
            self._saft_motor.add_speed_multiplier(MotorController.STBD_CCW_ROTATE_LAMBDA_NAME, self._stbd_ccw_rotate_lambda, True)
        else:
            raise Exception('expected stopped, clockwise or counter-clockwise argument.')

    def set_steering_mode(self, steering_mode):
        '''
        This sets the motor steering mode callback for repositioning in the
        prescribed direction. This adds a lambda to the motors to alter their 
        rotation to comply with the change in steering mode movement. 
        '''
        if steering_mode is None:
            raise ValueError('steering mode argument not provided.')
        self.reset_rotating()
        if steering_mode is SteeringMode.NONE:
            self._reset_slew_rate()
            self.set_motor_speed(Orientation.PFWD, 0.0)
            self.set_motor_speed(Orientation.SFWD, 0.0) 
            self.set_motor_speed(Orientation.PAFT, 0.0)
            self.set_motor_speed(Orientation.SAFT, 0.0)

        elif steering_mode is SteeringMode.CRAB_PORT:
            self._set_slew_rate(SlewRate.FASTEST)
            self._pfwd_motor.add_speed_multiplier(MotorController.FWD_REPOSITION_ROTATE_LAMBDA_NAME, self._fwd_reposition_rotate_lambda, True)
            self._sfwd_motor.add_speed_multiplier(MotorController.REV_REPOSITION_ROTATE_LAMBDA_NAME, self._rev_reposition_rotate_lambda, True)
            self._paft_motor.add_speed_multiplier(MotorController.REV_REPOSITION_ROTATE_LAMBDA_NAME, self._rev_reposition_rotate_lambda, True)
            self._saft_motor.add_speed_multiplier(MotorController.FWD_REPOSITION_ROTATE_LAMBDA_NAME, self._fwd_reposition_rotate_lambda, True)

        elif steering_mode is SteeringMode.CRAB_STBD:
            self._set_slew_rate(SlewRate.FASTEST)
            self._pfwd_motor.add_speed_multiplier(MotorController.REV_REPOSITION_ROTATE_LAMBDA_NAME, self._rev_reposition_rotate_lambda, True)
            self._sfwd_motor.add_speed_multiplier(MotorController.FWD_REPOSITION_ROTATE_LAMBDA_NAME, self._fwd_reposition_rotate_lambda, True)
            self._paft_motor.add_speed_multiplier(MotorController.FWD_REPOSITION_ROTATE_LAMBDA_NAME, self._fwd_reposition_rotate_lambda, True)
            self._saft_motor.add_speed_multiplier(MotorController.REV_REPOSITION_ROTATE_LAMBDA_NAME, self._rev_reposition_rotate_lambda, True)

        elif steering_mode is SteeringMode.ROTATE or steering_mode is SteeringMode.ROTATE_CW:
            self._set_slew_rate(SlewRate.FASTEST)
            self._pfwd_motor.add_speed_multiplier(MotorController.FWD_REPOSITION_ROTATE_LAMBDA_NAME, self._fwd_reposition_rotate_lambda, True)
            self._sfwd_motor.add_speed_multiplier(MotorController.REV_REPOSITION_ROTATE_LAMBDA_NAME, self._rev_reposition_rotate_lambda, True)
            self._paft_motor.add_speed_multiplier(MotorController.FWD_REPOSITION_ROTATE_LAMBDA_NAME, self._fwd_reposition_rotate_lambda, True)
            self._saft_motor.add_speed_multiplier(MotorController.REV_REPOSITION_ROTATE_LAMBDA_NAME, self._rev_reposition_rotate_lambda, True)

        elif steering_mode is SteeringMode.ROTATE_CCW:
            self._set_slew_rate(SlewRate.FASTEST)
            self._pfwd_motor.add_speed_multiplier(MotorController.REV_REPOSITION_ROTATE_LAMBDA_NAME, self._rev_reposition_rotate_lambda, True)
            self._sfwd_motor.add_speed_multiplier(MotorController.FWD_REPOSITION_ROTATE_LAMBDA_NAME, self._fwd_reposition_rotate_lambda, True)
            self._paft_motor.add_speed_multiplier(MotorController.REV_REPOSITION_ROTATE_LAMBDA_NAME, self._rev_reposition_rotate_lambda, True)
            self._saft_motor.add_speed_multiplier(MotorController.FWD_REPOSITION_ROTATE_LAMBDA_NAME, self._fwd_reposition_rotate_lambda, True)

        else: # e.g., AFRS
            self._set_slew_rate(SlewRate.FASTEST)
            self._pfwd_motor.add_speed_multiplier(MotorController.FWD_REPOSITION_RETURN_LAMBDA_NAME, self._fwd_reposition_return_lambda, True)
            self._sfwd_motor.add_speed_multiplier(MotorController.FWD_REPOSITION_RETURN_LAMBDA_NAME, self._fwd_reposition_return_lambda, True)
            self._paft_motor.add_speed_multiplier(MotorController.AFT_REPOSITION_RETURN_LAMBDA_NAME, self._aft_reposition_return_lambda, True)
            self._saft_motor.add_speed_multiplier(MotorController.AFT_REPOSITION_RETURN_LAMBDA_NAME, self._aft_reposition_return_lambda, True)

    def clamp(self, value):
        '''
        Return the clamp lambda function.
        '''
        return self._clamp(value)

    def _reset_slew_rate(self):
        '''
        Halts any automated acceleration or deceleration.
        '''
        for _motor in self._all_motors:
            _motor.slew_limiter.reset()

    def _set_slew_rate(self, slew_rate):
        '''
        Set the slew rate for all motors to the argument.
        '''
        for _motor in self._all_motors:
            _motor.slew_limiter.slew_rate = slew_rate

    @property
    def all_motors_are_stopped(self):
        '''
        Returns True when all motors are stopped.
        '''
        for _motor in self._all_motors:
            if not _motor.is_stopped:
                return False
        return True

    def reset_stopping(self):
        for _motor in self._all_motors:
            if _motor.has_speed_multiplier(MotorController.STOP_LAMBDA_NAME):
                _motor.remove_speed_multiplier(MotorController.STOP_LAMBDA_NAME)
            if _motor.has_speed_multiplier(MotorController.HALT_LAMBDA_NAME):
                _motor.remove_speed_multiplier(MotorController.HALT_LAMBDA_NAME)
            if _motor.has_speed_multiplier(MotorController.BRAKE_LAMBDA_NAME):
                _motor.remove_speed_multiplier(MotorController.BRAKE_LAMBDA_NAME)
        self._reset_slew_rate()

    def reset_rotating(self):
        for _motor in self._all_motors:
            _motor.remove_speed_multiplier('rotate')
        self._reset_slew_rate()

    def clear_speed_multipliers(self):
        '''
        Clear all motor speed control lambda functions from all motors.
        '''
        for _motor in self._all_motors:
            if _motor:
                _motor.clear_speed_multipliers()
            else:
                raise Exception('null motor in clear list.')
        self._log.info('cleared all speed multiplier lambdas.')

    def list_speed_multipliers(self):
        '''
        List the motor speed control lambda functions from all motors.
        '''
        self._log.info(Fore.GREEN + 'listing speed multipliers for {:d} motors:'.format( len(self._all_motors)))
        for _motor in self._all_motors:
            if _motor:
                _motor.list_speed_multipliers()

    def _braking_function(self, target_speed):
        '''
        This is a lambda function that will slow the motors to zero speed
        at a rather slow rate.
        '''
        print('_braking_function')
        target_speed = target_speed * self._brake_ratio
        if self.all_motors_are_stopped:
            return MotorController.BRAKE_LAMBDA_NAME
        elif isclose(target_speed, 0.0, abs_tol=1e-2):
            return 0.0
        else:
            return target_speed

    def _stopping_function(self, target_speed):
        '''
        This is a lambda function that will slow the motors to zero speed
        very quickly. This additionally directly calls stop() on all motors.
        '''
        print('_stopping_function')
        _stop_ratio = 0.25
        target_speed = target_speed * _stop_ratio
        if self.all_motors_are_stopped:
            self._log.info('full stop now…')
            for _motor in self._all_motors:
                # we rely on this ultimately
                _motor.stop()
            self._log.info('stopped.')
            # return lambda name indicating we're done
            return MotorController.STOP_LAMBDA_NAME
        elif isclose(target_speed, 0.0, abs_tol=1e-2):
            return 0.0
        else:
            return target_speed

    def brake(self):
        '''
        Creates a thread to allow braking via a call to decelerate.
        '''
        self._log.info(Fore.YELLOW + 'brake')
        def run_brake():
            self.decelerate(target_speed=0.0, step=0.02, step_delay_ms=10, enabled=lambda: self.enabled)
        _brake_thread = Thread(target=run_brake, daemon=True)
        _brake_thread.start()

    def stop(self):
        '''
        Stops all motors immediately, with no slewing.

        This differs from both halt() and brake() in that it also suppresses
        all behaviours. TODO
        '''
        self._log.info(Fore.YELLOW + 'stop')
        if self.is_stopped:
            # just in case a motor is still moving
            for _motor in self._all_motors:
                _motor.stop()
            self._log.warning('already stopped.')
            return
        elif self.is_stopping():
            self._log.warning('already stopping.')
            return
        else:
            self._log.info('stopping…')
        if self._external_clock or self._loop_enabled:
            if self._slew_limiter_enabled:
                self._log.info('stopping soft…')
                # use slew limiter for stopping if available
#               self._set_slew_rate(self._stop_slew_rate)
                for _motor in self._all_motors:
                    _motor.add_speed_multiplier(MotorController.STOP_LAMBDA_NAME, self._stopping_function)
            else:
                self._log.info('stopping hard…')
                for _motor in self._all_motors:
                    _motor.stop()
        else:
            self._log.info('stopping very hard…')
            self.emergency_stop()

    def emergency_stop(self):
        '''
        We try to avoid this as it's hard on the gears.
        '''
        self._log.info('emergency stop…')
        for _motor in self._all_motors:
            _motor.stop()
        self._log.info('emergency stopped.')

    def disable(self):
        '''
        Disable the motors.
        '''
        if not self.is_stopped:
            if self._graceful_stop:
                self.brake()
                time.sleep(3)
            else:
                self.emergency_stop()
        if self.enabled:
            if self._external_clock:
                self._log.info('disabling by removing external clock callback…')
                self._external_clock.remove_callback(self._external_callback_method)
            else:
                self._log.info('disabling by stopping loop…')
                self._stop_loop() 
            Component.disable(self)
            self._log.info('disabled.')
        else:
            self._log.debug('already disabled.')

    def close(self):
        '''
        Closes the motor controller.
        '''
        if not self.closed:
            Component.close(self)
            self._log.info('motor controller closed.')
        else:
            self._log.warning('motor controller already closed.')

    def print_info(self, count):
        if self.is_stopped:
            if not self._print_info_done:
                self._log.info(('[{:04d}] '.format(count) if count else '') + 'speed: stopped.')
            self._print_info_done = True
        else:
            self._print_info_done = False
            self._log.info(('[{:04d}] '.format(count) if count else '')
                    + 'speed: '
                    + Fore.RED   + 'pfwd: {:<4.2f} / {:<4.2f}'.format(
                            self._pfwd_motor.target_speed, self._pfwd_motor.modified_speed)
                    + Fore.CYAN  + ' :: '
                    + Fore.GREEN + 'sfwd: {:<4.2f} / {:<4.2f}'.format(
                            self._sfwd_motor.target_speed, self._sfwd_motor.modified_speed)
                    + ' :: '
                    + Fore.RED   + 'paft: {:<4.2f} / {:<4.2f}'.format(
                            self._paft_motor.target_speed, self._paft_motor.modified_speed)
                    + Fore.CYAN  + ' :: '
                    + Fore.GREEN + 'saft: {:<4.2f} / {:<4.2f}'.format(
                            self._saft_motor.target_speed, self._saft_motor.modified_speed))
#EOF
