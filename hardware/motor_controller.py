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
            self._log.info(Fore.WHITE + Style.BRIGHT + 'using existing external clock.')
        elif _create_ext_clock:
            self._log.info('creating IRQ clock…')
            self._external_clock = IrqClock(config, level=Level.INFO)
            self._external_clock.enable()
            self._log.info(Fore.WHITE + Style.BRIGHT + 'using created external clock.')
        if self._external_clock is None:
            self._rate = Rate(self._loop_freq_hz, Level.ERROR)
            self._log.info(Fore.WHITE + Style.BRIGHT + 'using thread loop.')
        else:
            self._rate = None
            self._log.info(Fore.WHITE + Style.BRIGHT + 'using external clock.')

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
        # intent vector registry
        self._intent_vectors = {}  # {name: lambda}
        self.__callback      = None
        self._is_stopped     = True # used to capture state transitions
        self.__state_change_callbacks = [] # anyone who wants to be informed if the robot is moving or stopped
        # base per-motor target speeds used to form a persistent base intent vector
        self._base_pfwd_speed = 0.0
        self._base_sfwd_speed = 0.0
        self._base_paft_speed = 0.0
        self._base_saft_speed = 0.0
        # controller-level speed modifiers applied after blending (name -> fn)
        # modifier fn signature: fn(speeds:list[4]) -> list[4] | str (name to remove) | None
        self._speed_modifiers = {}
        # register the base intent vector so it is always included in blending;
        # returns (vx, vy, omega) derived from the four base motor targets
        self.add_intent_vector("base", lambda: (
            float((self._base_pfwd_speed - self._base_sfwd_speed - self._base_paft_speed + self._base_saft_speed) / 4.0),
            float((self._base_pfwd_speed + self._base_sfwd_speed + self._base_paft_speed + self._base_saft_speed) / 4.0),
            float((self._base_pfwd_speed + self._base_paft_speed - self._base_sfwd_speed - self._base_saft_speed) / 4.0)
        ))
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

    def add_intent_vector(self, name, vector_lambda, exclusive=False):
        '''
        Register a behaviour's intent vector lambda.
        The lambda should return (vx, vy, omega).
        '''
        if exclusive:
            self._intent_vectors.clear()
        else:
            if name in self._intent_vectors:
                raise Exception("intent vector '{}' already registered; ignoring duplicate.".format(name))
        if vector_lambda.__name__ != "<lambda>":
            raise TypeError('expected lambda function, not {}'.format(type(vector_lambda)))
        self._intent_vectors[name] = vector_lambda
        self._log.info('added intent vector: {}'.format(name))

    def remove_intent_vector(self, name):
        '''
        Remove a behaviour's intent vector lambda.
        '''
        if name in self._intent_vectors:
            del self._intent_vectors[name]
            self._log.info('removed intent vector: {}'.format(name))

    def _blend_intent_vectors(self):
        '''
        Blends all registered intent vectors, returning a single (vx, vy[, omega]) tuple.
        '''
        vectors = [fn() for fn in self._intent_vectors.values()]
        if not vectors:
            return (0.0, 0.0, 0.0)
        sum_vector = [0.0, 0.0, 0.0]
        for v in vectors:
            if len(v) != 3:
                raise Exception('expected length of 3, not {}; {}'.format(len(v), v))
            for i in range(3):
                sum_vector[i] += v[i]
        avg_vector = tuple(s / len(vectors) for s in sum_vector)
        return avg_vector

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

    def add_speed_modifier(self, name, modifier_fn, exclusive=True):
        """
        Register a controller-level speed modifier.
        Only accepts lambda functions (enforced).
        modifier_fn(speeds: list[4]) -> list[4] | str | None
        """
        if modifier_fn.__name__ != "<lambda>":
            raise TypeError('expected lambda function, not {}'.format(type(modifier_fn)))
        if exclusive:
            self._speed_modifiers.clear()
        self._speed_modifiers[name] = modifier_fn
        self._log.info('added speed modifier: {}'.format(name))

    def remove_speed_modifier(self, name):
        """
        Remove a previously-registered controller-level speed modifier.
        """
        if name in self._speed_modifiers:
            del self._speed_modifiers[name]
            self._log.info('removed speed modifier: {}'.format(name))

    def list_speed_modifiers(self):
        """
        Log the registered controller-level speed modifiers.
        """
        if not self._speed_modifiers:
            self._log.info('no speed modifiers registered.')
        else:
            for n in self._speed_modifiers:
                self._log.info('speed modifier: {}'.format(n))

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
       
          - blend intent vectors -> (vx, vy, omega)
          - mecanum kinematics -> per-wheel speeds
          - normalize if needed
          - apply controller-level speed modifiers (in registration order)
          - write final speeds into Motor.target_speed and call update_target_speed()
        '''
        intent = self._blend_intent_vectors()
        if len(intent) != 3:
            raise ValueError('expected 3 values, not {}.'.format(len(intent)))
        vx, vy, omega = intent

        # mecanum -> wheel speeds
        pfwd = vy + vx + omega
        sfwd = vy - vx - omega
        paft = vy - vx + omega
        saft = vy + vx - omega
        speeds = [pfwd, sfwd, paft, saft]

        # normalize if any magnitude > 1.0
        max_abs = max(abs(s) for s in speeds)
        if max_abs > 1.0:
            speeds = [s / max_abs for s in speeds]

        # apply controller-level modifiers in registration order
        for name, fn in list(self._speed_modifiers.items()):
            result = fn(list(speeds))
            if isinstance(result, str):
                # modifier requested removal by returning its name
                if result in self._speed_modifiers:
                    del self._speed_modifiers[result]
            elif result is None:
                # no change
                pass
            else:
                # expect an iterable of 4 numeric speeds
                if not hasattr(result, '__iter__') or len(result) != 4:
                    raise Exception('speed modifier {} returned invalid value: {}'.format(name, result))
                speeds = [float(x) for x in result]

        # renormalize after modifiers
        max_abs = max(abs(s) for s in speeds)
        if max_abs > 1.0:
            speeds = [s / max_abs for s in speeds]

        # coerce to native floats and apply final speeds
        speeds = [float(s) for s in speeds]
        self._pfwd_motor.target_speed = speeds[0]
        self._sfwd_motor.target_speed = speeds[1]
        self._paft_motor.target_speed = speeds[2]
        self._saft_motor.target_speed = speeds[3]

        # update motors (apply slew/PID/jerk)
        for _motor in self._all_motors:
            _motor.update_target_speed()

        _count = next(self._event_counter)
        self.print_info(_count, vx, vy, omega)
        self._state_change_check()
        if self._motor_loop_callback is not None:
            self._motor_loop_callback()

    def x_motor_tick(self):
        '''
        The shared logic for a single motor control loop iteration, used by
        both _motor_loop() and _external_callback_method(). Incorporates intent vector 
        blending and full Mecanum wheel kinematic mapping and normalization.
        '''
        intent = self._blend_intent_vectors()
        if len(intent) != 3:
            raise ValueError('expected 3 values, not {}.'.format(len(intent)))
        vx, vy, omega = intent
        # Mecanum equations -> per-wheel speeds
        pfwd = vy + vx + omega
        sfwd = vy - vx - omega
        paft = vy - vx + omega
        saft = vy + vx - omega
        speeds = [pfwd, sfwd, paft, saft]
        # normalise if required
        max_abs = max(abs(s) for s in speeds)
        if max_abs > 1.0:
            speeds = [s / max_abs for s in speeds]
        # apply controller-level speed modifiers (in registration order)
        if len(self._speed_modifiers) > 0:
            # iterate over a copy to allow safe removal from inside modifiers
            for name, fn in list(self._speed_modifiers.items()):
                try:
                    result = fn(list(speeds))
                except Exception as e:
                    self._log.error('speed modifier {} raised: {}'.format(name, e))
                    # on error remove the offending modifier
                    try:
                        del self._speed_modifiers[name]
                    except Exception:
                        pass
                    continue
                if isinstance(result, str):
                    # modifier requested removal by returning its name
                    if result in self._speed_modifiers:
                        del self._speed_modifiers[result]
                        self._log.info('speed modifier {} removed itself.'.format(result))
                elif result is None:
                    # no change
                    continue
                else:
                    # expect an iterable of 4 numeric speeds
                    if not hasattr(result, '__iter__') or len(result) != 4:
                        raise Exception('speed modifier {} returned invalid value: {}'.format(name, result))
                    speeds = [float(x) for x in result]
            # re-normalise after modifiers
            max_abs = max(abs(s) for s in speeds)
            if max_abs > 1.0:
                speeds = [s / max_abs for s in speeds]
        speeds = [float(s) for s in speeds]
        # apply computed wheel speeds directly to Motor.target_speed properties
        if self._pfwd_motor and self._pfwd_motor.enabled:
            self._pfwd_motor.target_speed = speeds[0]
        if self._sfwd_motor and self._sfwd_motor.enabled:
            self._sfwd_motor.target_speed = speeds[1]
        if self._paft_motor and self._paft_motor.enabled:
            self._paft_motor.target_speed = speeds[2]
        if self._saft_motor and self._saft_motor.enabled:
            self._saft_motor.target_speed = speeds[3]
        # update motors (apply slew/pid/etc.)
        for _motor in self._all_motors:
            _motor.update_target_speed()
        _count = next(self._event_counter)
        self.print_info(_count, vx, vy, omega)
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
            self._log.info(Fore.MAGENTA + 'set speeds: {:4.2f} | {:4.2f} | {:4.2f} | {:4.2f}'.format(pfwd_speed, sfwd_speed, paft_speed, saft_speed))
        self.set_motor_speed(Orientation.PFWD, pfwd_speed)
        self.set_motor_speed(Orientation.SFWD, sfwd_speed)
        self.set_motor_speed(Orientation.PAFT, paft_speed)
        self.set_motor_speed(Orientation.SAFT, saft_speed)

    def set_speed(self, orientation, value):
        '''
        Sets the speed of all motors associated with the port or starboard
        orientation.

        For testing we permit a single SAFT motor, which otherwise is not a
        particularly good idea.
        '''
        _speed = self._clamp(value)
        if self._verbose:
            self._log.info(Fore.MAGENTA + 'set {} speed: {:4.2f} clamped to {:4.2f}'.format(orientation.name, value, _speed))
        if orientation is Orientation.ALL:
            self.set_motor_speed(Orientation.PFWD, _speed)
            self.set_motor_speed(Orientation.PAFT, _speed)
            self.set_motor_speed(Orientation.SFWD, _speed)
            self.set_motor_speed(Orientation.SAFT, _speed)
        elif orientation is Orientation.PORT:
            self.set_motor_speed(Orientation.PFWD, _speed)
            self.set_motor_speed(Orientation.PAFT, _speed)
        elif orientation is Orientation.STBD:
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
        """
        Update internal base per-motor target speed variable (forms the base intent).
        The actual Motor.target_speed is set in _motor_tick() after blending and modifiers.
        """
        if not self.enabled:
            self._log.error('motor controller not enabled.')
        if isinstance(target_speed, int):
            raise ValueError('expected target speed as float not int: {:d}'.format(target_speed))
        if not isinstance(target_speed, float):
            raise ValueError('expected float, not {}'.format(type(target_speed)))
        if self._verbose:
            self._log.info(Fore.MAGENTA + 'set {} motor base target: {:5.2f}'.format(orientation.name, target_speed))
        if orientation is Orientation.PFWD:
            self._base_pfwd_speed = target_speed
        elif orientation is Orientation.SFWD:
            self._base_sfwd_speed = target_speed
        elif orientation is Orientation.PAFT:
            self._base_paft_speed = target_speed
        elif orientation is Orientation.SAFT:
            self._base_saft_speed = target_speed
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

    def _braking_function(self, step=0.05):
        '''
        Return a controller-level braking modifier closure.

        The returned modifier is called with the current list of wheel speeds
        [pfwd, sfwd, paft, saft] and should return a modified list of 4 speeds,
        or its own name (string) to indicate it should be removed.

        This modifier gradually reduces wheel speeds by a step factor and, when
        effectively zero, stops the motors and requests removal by returning its name.
        '''
        name = MotorController.BRAKE_LAMBDA_NAME
        # ensure a sane default ratio if not set elsewhere
        _step = float(step) if step and step > 0.0 else 0.05
        factor = 1.0
        def _modifier(speeds):
            nonlocal factor, _step
            # progressively decrease factor
            factor = max(0.0, factor - _step)
            modified = [s * factor for s in speeds]
            # if effectively zero, call Motor.stop() for final assurance and request removal
            if all(isclose(m, 0.0, abs_tol=1e-2) for m in modified) or factor <= 0.0:
                for _m in self._all_motors:
                    try:
                        _m.stop()
                    except Exception:
                        pass
                return name
            return modified
        return _modifier

    def _stopping_function(self, step=0.15):
        '''
        Return a controller-level stopping modifier closure (aggressive).

        Similar to _braking_function but with a larger decrement so stopping
        completes faster. Returns the modifier function that will reduce the
        final wheel speeds and remove itself when the motors have been brought
        to a practical zero and stopped.
        '''
        name = MotorController.STOP_LAMBDA_NAME
        _step = float(step) if step and step > 0.0 else 0.15
        factor = 1.0
        def _modifier(speeds):
            nonlocal factor, _step
            factor = max(0.0, factor - _step)
            modified = [s * factor for s in speeds]
            if all(isclose(m, 0.0, abs_tol=1e-2) for m in modified) or factor <= 0.0:
                for _m in self._all_motors:
                    try:
                        _m.stop()
                    except Exception:
                        pass
                return name
            return modified
        return _modifier

    def emergency_stop(self):
        """
        Immediate hard stop: clear controller-level modifiers and stop every motor.
        """
        self._log.info('emergency stop…')
        self._speed_modifiers.clear()
        for _motor in self._all_motors:
            _motor.stop()
        self._log.info('emergency stopped.')

    def stop(self):
        """
        Aggressive stop: register a fast controller-level modifier to ramp down motors and
        remove itself when complete.
        """
        self._log.info(Fore.YELLOW + 'stop')
        if self.is_stopped:
            for _motor in self._all_motors:
                _motor.stop()
            self._log.warning('already stopped.')
            return

        name = MotorController.STOP_LAMBDA_NAME
        factor = 1.0
        _step = self._decel_step * 3.0
        def _stop_modifier(speeds):
            nonlocal factor, _step
            factor = max(0.0, factor - _step)
            modified = [s * factor for s in speeds]
            if all(isclose(m, 0.0, abs_tol=1e-2) for m in modified) or factor <= 0.0:
                for _m in self._all_motors:
                    _m.stop()
                return name
            return modified

        self.remove_speed_modifier(name)
        # register a lambda (so add_speed_modifier's guard accepts it)
        self.add_speed_modifier(name, (lambda speeds, _fn=_stop_modifier: _fn(speeds)), exclusive=True)
        self._log.info('registered controller-level stopping modifier.')

    def brake(self):
        """
        Soft brake: register a gentle modifier to progressively reduce speeds.
        """
        self._log.info(Fore.YELLOW + 'brake')
        if self.is_stopped:
            self._log.warning('already stopped.')
            return

        name = MotorController.BRAKE_LAMBDA_NAME
        factor = 1.0
        _step = self._decel_step
        def _brake_modifier(speeds):
            nonlocal factor, _step
            factor = max(0.0, factor - _step)
            modified = [s * factor for s in speeds]
            if all(isclose(m, 0.0, abs_tol=1e-2) for m in modified) or factor <= 0.0:
                for _m in self._all_motors:
                    _m.stop()
                return name
            return modified

        self.remove_speed_modifier(name)
        # register a lambda (so add_speed_modifier's guard accepts it)
        self.add_speed_modifier(name, (lambda speeds, _fn=_brake_modifier: _fn(speeds)), exclusive=True)
        self._log.info('registered controller-level braking modifier.')

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

    def print_info(self, count, vx, vy, omega):
        self._simple = True
        if self.is_stopped:
            if not self._print_info_done:
                self._log.info(('[{:04d}] '.format(count) if count else '') + 'speed: stopped.')
            self._print_info_done = True
        elif self._simple:
            if count % 10 == 0:
                self._print_info_done = False
                if vx > 0.0 or omega > 0.0:
                    _color = Fore.WHITE + Style.BRIGHT
                else:
                    _color = Fore.WHITE 
                self._log.info(('[{:04d}] '.format(count) if count else '')
                        + 'sp: '
                        + Fore.RED   + 'pfwd: {:<4.2f}'.format(self._pfwd_motor.target_speed)
                        + Fore.CYAN  + ' :: '
                        + Fore.GREEN + 'sfwd: {:<4.2f}'.format(self._sfwd_motor.target_speed)
                        + Fore.CYAN  + ' :: '
                        + Fore.RED   + 'paft: {:<4.2f}'.format(self._paft_motor.target_speed)
                        + Fore.CYAN  + ' :: '
                        + Fore.GREEN + 'saft: {:<4.2f}'.format(self._saft_motor.target_speed)
                        + Fore.CYAN  + ' :: '
                        + _color + '({:<4.2f}, {:4.2f}, {:4.2f})'.format(vx, vy, omega)
                    )
        else:
            self._print_info_done = False
            self._log.info(('[{:04d}] '.format(count) if count else '')
                    + 'speed: '
                    + Fore.RED   + 'pfwd: {:<4.2f} / {:<4.2f}'.format(
                            self._pfwd_motor.target_speed, self._pfwd_motor.modified_speed)
                    + Fore.CYAN  + ' :: '
                    + Fore.GREEN + 'sfwd: {:<4.2f} / {:<4.2f}'.format(
                            self._sfwd_motor.target_speed, self._sfwd_motor.modified_speed)
                    + Fore.CYAN  + ' :: '
                    + Fore.RED   + 'paft: {:<4.2f} / {:<4.2f}'.format(
                            self._paft_motor.target_speed, self._paft_motor.modified_speed)
                    + Fore.CYAN  + ' :: '
                    + Fore.GREEN + 'saft: {:<4.2f} / {:<4.2f}'.format(
                            self._saft_motor.target_speed, self._saft_motor.modified_speed))

#EOF
