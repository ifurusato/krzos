#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-05
# modified: 2025-12-08

import sys, traceback
import time
import statistics
import itertools
from threading import Event
from math import isclose
from colorama import init, Fore, Style
init()

from core.component import Component
from core.direction import Direction
from core.orientation import Orientation
from core.rate import Rate
from core.util import Util
from core.rotation import Rotation
from core.steering_mode import SteeringMode
from core.logger import Logger, Level
from hardware.i2c_scanner import I2CScanner
from hardware.irq_clock import IrqClock
from hardware.motor_configurer import MotorConfigurer
from hardware.odometer import Odometer
from hardware.slew_limiter import SlewLimiter

class MotorController(Component):
    NAME = 'motor-ctrl'
    BASE_NAME = 'base-intent-vector'
    BRAKE_LAMBDA_NAME = "__brake_lambda"
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
        # configuration
        self._verbose        = _cfg.get('verbose')
        self._loop_freq_hz   = _cfg.get('loop_freq_hz') # main loop frequency
        self._loop_delay_sec = 1 / self._loop_freq_hz
        self._accel_step     = _cfg.get('accel_step', 0.02)
        self._accel_step_delay_ms = _cfg.get('accel_step_delay_ms', 20)
        self._decel_step     = _cfg.get('decel_step', 0.02)
        self._decel_step_delay_ms = _cfg.get('decel_step_delay_ms', 20)
        self._coast_step          = _cfg.get('coast_step', 0.01)   # rate for coasting to a stop (very slowly)
        self._brake_step          = _cfg.get('brake_step', 0.03)   # rate for braking
        self._halt_step           = _cfg.get('halt_step', 0.067)   # rate for halting (quickly)
        self._stop_step           = _cfg.get('stop_step', 0.09)    # rate for stopping (abrupt)
        self._emergency_stop_step = _cfg.get('emergency_stop_step', 0.25)  # rate for emergency stopping (very abrupt)
        self._show_battery        = _cfg.get('show_battery', False) # enable/disable show battery level
        # data logging
        self._data_log = None
        _data_logging = config['kros'].get('application').get('data_logging')
        if _data_logging:
            self._log.data(Fore.GREEN + 'data logging is active.')
            self._data_log = Logger('{}'.format(self.NAME), log_to_file=True, data_logger=True, level=Level.INFO)
            self._pfwd_data_log = Logger('pfwd'.format(self.NAME), log_to_file=True, data_logger=True, level=Level.INFO)
            self._sfwd_data_log = Logger('sfwd'.format(self.NAME), log_to_file=True, data_logger=True, level=Level.INFO)
            self._paft_data_log = Logger('paft'.format(self.NAME), log_to_file=True, data_logger=True, level=Level.INFO)
            self._saft_data_log = Logger('saft'.format(self.NAME), log_to_file=True, data_logger=True, level=Level.INFO)
        # optional vector post processor
        self._post_processor = None
        # odometer
        self._odometer = Odometer(config)
        # eyeballs monitor
        self._use_eyeballs = _cfg.get('use_eyeballs', False)
        self._eyeballs_monitor = None
        if self._use_eyeballs:
            from hardware.eyeballs_monitor import EyeballsMonitor
            self._eyeballs_monitor = EyeballsMonitor(self, level=level)
            self._log.info('eyeballs monitor configured.')
        # slew limiter for intent vectors
        _slew_cfg = _cfg.get('slew_limiter')
        _slew_enabled = _slew_cfg.get('enabled', False)
        self._slew_limiter = None
        if _slew_enabled:
            self._slew_limiter = SlewLimiter(config, level=level)
            self._slew_limiter.enable()
            self._log.info(Fore.GREEN + 'slew limiter enabled.')
        else:
            self._log.warning('slew limiter disabled.')
        _create_ext_clock    = _cfg.get('create_external_clock')
        if external_clock:
            self._external_clock = external_clock
            self._log.info('using existing external clock.')
        elif _create_ext_clock:
            self._log.info('creating IRQ clock…')
            self._external_clock = IrqClock(config, level=Level.INFO)
            self._external_clock.enable()
            self._log.info('using created external clock.')
        if self._external_clock is None:
            self._rate = Rate(self._loop_freq_hz, Level.ERROR)
            self._log.info('using thread loop.')
        else:
            self._rate = None
            self._log.info('using external clock.')
        self._log.info('loop frequency: {}Hz ({:4.2f}s)'.format(self._loop_freq_hz, self._loop_delay_sec))
        # motor power smoothing configuration
        _smoothing_cfg = _cfg.get('motor_power_smoothing')
        self._motor_power_smoothing_enabled = _smoothing_cfg.get('enabled', False)
        self._motor_power_smoothing_alpha = _smoothing_cfg.get('alpha', 0.5)
        self._last_motor_speeds = [0.0, 0.0, 0.0, 0.0]
        if self._motor_power_smoothing_enabled:
            self._log.info('motor power smoothing enabled with alpha={:.2f}'.format(self._motor_power_smoothing_alpha))
        else:
            self._log.info('motor power smoothing disabled')
        # motor controller
        self._closed_loop = None
        self._all_motors  = []
        self._motor_configurer = MotorConfigurer(config, _i2c_scanner, motors_enabled=True, level=level)
        self._pfwd_motor     = self._motor_configurer.get_motor(Orientation.PFWD)
        self._sfwd_motor     = self._motor_configurer.get_motor(Orientation.SFWD)
        self._paft_motor     = self._motor_configurer.get_motor(Orientation.PAFT)
        self._saft_motor     = self._motor_configurer.get_motor(Orientation.SAFT)
        self._all_motors     = self._get_motors()
        self.set_show_battery(self._show_battery)
        self._is_daemon      = True
        self._loop_thread    = None
        self._loop_enabled   = False
        self._event_counter  = itertools.count()
        self._motor_loop_callback = None
        self._use_graceful_stop  = True # TODO config?
        self._braking_active     = False
        self._is_braked          = False
        self._braking_event      = Event()
        self._current_brake_step = None
        # speed and changes to speed
        self._intent_vectors = {}  # {name: lambda}, intent vector registry
        self._blended_intent_vector = (0.0, 0.0, 0.0) # cached blended intent vector from last motor tick
        self.__callback      = None
        self._is_stopped     = True # used to capture state transitions
        self.__state_change_callbacks = [] # anyone who wants to be informed if the robot is moving or stopped
        # base per-motor target speeds used to form a persistent base intent vector
        self._base_pfwd_speed = 0.0
        self._base_sfwd_speed = 0.0
        self._base_paft_speed = 0.0
        self._base_saft_speed = 0.0
        self._base_priority   = 0.3
        # controller-level speed modifiers applied after blending (name -> fn)
        # modifier fn signature: fn(speeds:list[4]) -> list[4] | str (name to remove) | None
        self._speed_modifiers = {}
        _max_speed           = _cfg.get('max_speed') # max speed of motors (0-1.0)
        _min_speed           = -1 * _max_speed
        self._log.info('motor speed clamped at {} to {}.'.format(_min_speed, _max_speed))
        self._clamp          = lambda n: max(min(_max_speed, n), _min_speed)
        # behavior-specific speed multipliers for throttling
        self._intent_vector_multipliers = {}
        self._print_info_done = False
        _closed_loop         = _cfg.get('closed_loop')
        self.set_closed_loop(_closed_loop)
        # finish up…
        self._log.info(Fore.GREEN + 'ready with {} motors.'.format(len(self._all_motors)))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def set_post_processor(self, post_processor):
        '''
        Sets the intent vector post-processor to the argument, removing it if
        the argument is None.
        '''
        self._post_processor = post_processor

    def set_show_battery(self, enable):
        '''
        Enable/disable the show battery feature on both ThunderBorg motor controllers.
        '''
        self._motor_configurer.set_thunderborg_leds(enable)

    def get_odometer(self):
        '''
        Return the Odometer used by this MotorController.
        '''
        return self._odometer

    @property
    def base_priority(self):
        '''
        Returns the current base intent vector priority.
        '''
        return self._base_priority

    @base_priority.setter
    def base_priority(self, value):
        '''
        Set the base intent vector priority (0.0-1.0).
        Used by test scripts or direct motor control to adjust priority relative to behaviors.
        '''
        if not isinstance(value, float):
            raise ValueError('expected float, not {}'.format(type(value)))
        if value < 0.0 or value > 1.0:
            self._log.warning('base priority {} outside typical range [0.0-1.0]'.format(value))
        self._base_priority = value
        self._log.info('base intent vector priority set to {:.2f}'.format(value))

    @property
    def braking_active(self):
        '''
        Returns true if braking is currently active.
        '''
        return self._braking_active

    @property
    def is_braked(self):
        '''
        Returns True if the robot is in a fully braked state.
        This is set True at the end of a brake() sequence and cleared by release_brake().
        '''
        return self._is_braked

    def clear_intent_vectors(self):
        '''
        Clear the existing intent vectors.
        '''
        self._intent_vectors.clear()
        self._blended_intent_vector = (0.0, 0.0, 0.0)

    @property
    def forward_velocity(self):
        '''
        Returns the current normalized forward velocity (vy component) from the
        last blended intent vector.
        '''
        return self._blended_intent_vector[1]

    def set_base_intent_vector(self, enable):
        '''
        Adds or removes the base intent vector from the motor controller.
        '''
        _has_base = self.has_intent_vector(MotorController.BASE_NAME)
        if enable:
            if _has_base:
                self._log.warning('base intent vector already set.')
                return
            # register the base intent vector so it is always included in blending;
            # returns (vx, vy, omega) derived from the four base motor targets
            self.add_intent_vector(
                MotorController.BASE_NAME,
                lambda: (
                    float((self._base_pfwd_speed - self._base_sfwd_speed - self._base_paft_speed + self._base_saft_speed) / 4.0),
                    float((self._base_pfwd_speed + self._base_sfwd_speed + self._base_paft_speed + self._base_saft_speed) / 4.0),
                    float((self._base_pfwd_speed + self._base_paft_speed - self._base_sfwd_speed - self._base_saft_speed) / 4.0)
                ),
                lambda: self._base_priority
            )
        else:
            if not _has_base:
                self._log.warning('base intent vector was not set.')
                return
            self.remove_intent_vector(MotorController.BASE_NAME)

    def has_intent_vector(self, name):
        '''
        Check if a named intent vector is currently registered.

        :param name: the name of the intent vector to check
        :return: True if the intent vector exists, False otherwise
        '''
        return name in self._intent_vectors

    def add_intent_vector(self, name, vector_lambda, priority_lambda=None, exclusive=False):
        '''
        Register a behaviour's intent vector lambda with optional priority lambda.
        The vector lambda should return (vx, vy, omega).
        The priority lambda should return a float (0.0-1.0, default 0.3).
        '''
        if exclusive:
            self.clear_intent_vectors()
        else:
            if name in self._intent_vectors:
                raise Exception("intent vector '{}' already registered; ignoring duplicate.".format(name))
        if vector_lambda.__name__ != "<lambda>":
            raise TypeError('expected lambda function for vector, not {}'.format(type(vector_lambda)))
        if priority_lambda is not None and priority_lambda.__name__ != "<lambda>":
            raise TypeError('expected lambda function for priority, not {}'.format(type(priority_lambda)))

        self._log.debug('adding intent vector: {}'.format(name))
        self._intent_vectors[name] = {
            'vector': vector_lambda,
            'priority': priority_lambda if priority_lambda else lambda: 0.3
        }
        self._log.info('added intent vector: {}'.format(name))

    def remove_intent_vector(self, name):
        '''
        Remove a behaviour's intent vector lambda.
        '''
        if name in self._intent_vectors:
            self._log.info('removing intent vector: {}'.format(name))
            del self._intent_vectors[name]
            self._log.info('removed intent vector: {}'.format(name))

    def set_behavior_speed_multiplier(self, behavior_name, multiplier):
        '''
        Set a speed multiplier for a specific behavior's intent vector.
        multiplier: 0.0-1.0, where 1.0 = full speed, 0.0 = stopped

        This modifies the behavior's contribution during blending.
        '''
        self._intent_vector_multipliers[behavior_name] = max(0.0, min(1.0, multiplier))
        self._log.info('set {} speed multiplier to {:.2f}'.format(behavior_name, multiplier))

    def diag_blend_intent_vectors(self):
        if not self._intent_vectors:
            return (0.0, 0.0, 0.0)
        weighted_sum = [0.0, 0.0, 0.0]
        total_weight = 0.0
        # track largest contributor
        max_magnitude = 0.0
        max_contributor = None
        for name, entry in self._intent_vectors.items():
            vector = entry['vector']()
            priority = entry['priority']()
            magnitude = (vector[0]**2 + vector[1]**2 + vector[2]**2)**0.5
            weighted_magnitude = magnitude * priority
            if weighted_magnitude > max_magnitude:
                max_magnitude = weighted_magnitude
                max_contributor = (name, vector, priority)
            if len(vector) != 3:
                raise Exception('expected length of 3, not {}; {}'.format(len(vector), vector))
            total_weight += priority
            for i in range(3):
                weighted_sum[i] += vector[i] * priority
        if total_weight == 0.0:
            return (0.0, 0.0, 0.0)
        result = tuple(s / total_weight for s in weighted_sum)
        # DIAGNOSTIC: Log the dominant behavior
        if max_contributor:
            name, vec, pri = max_contributor
            self._log.info('dominant: {} vec=({:.2f},{:.2f},{:.2f}) pri={:.2f} -> blended=({:.2f},{:.2f},{:.2f})'.format(
                name, vec[0], vec[1], vec[2], pri, result[0], result[1], result[2]))
        return result

    def _blend_intent_vectors(self):
        '''
        Priority-weighted blending of all intent vectors, scaled by vector magnitude.

        Each behavior's vector is weighted by its dynamic priority value multiplied
        by the magnitude of its intent vector. This makes sure that inactive or weakly
        contributing behaviors (small magnitude) don't dilute the blend as much as
        strongly contributing behaviors.

        Higher priority behaviors have proportionally more influence, but only when
        they are actively contributing (non-zero magnitude).

        Returns a tuple (vx, vy, omega) representing the blended intent vector.
        '''
        ENABLE_BEHAVIOUR_SPEED_MULTIPLIER = True
#       if not self._intent_vectors:
#           return (0.0, 0.0, 0.0)
        weighted_sum = [0.0, 0.0, 0.0]
        total_weight = 0.0
        for name, entry in self._intent_vectors.items():
            if ENABLE_BEHAVIOUR_SPEED_MULTIPLIER:
                vector = entry['vector']()
                priority = entry['priority']()
                if len(vector) != 3:
                    raise Exception('expected length of 3, not {}; {}'.format(len(vector), vector))
                # apply behavior-specific speed multiplier
                multiplier = self._intent_vector_multipliers.get(name, 1.0)
                vector = tuple(v * multiplier for v in vector)
                magnitude = (vector[0]**2 + vector[1]**2 + vector[2]**2)**0.5
            else:
                vector = entry['vector']()
                priority = entry['priority']()
                if len(vector) != 3:
                    raise Exception('expected length of 3, not {}; {}'.format(len(vector), vector))
                # calculate vector magnitude to determine how actively this behavior is contributing
                magnitude = (vector[0]**2 + vector[1]**2 + vector[2]**2)**0.5
            # contribution is priority scaled by magnitude: inactive behaviors contribute proportionally less
            contribution = priority * magnitude
            total_weight += contribution
            for i in range(3):
                weighted_sum[i] += vector[i] * contribution
        if total_weight == 0.0:
            return (0.0, 0.0, 0.0)
        return tuple(s / total_weight for s in weighted_sum)

    def is_closed_loop(self):
        return self._closed_loop

    def set_closed_loop(self, closed_loop):
        '''
        Set the motor controller to open or closed loop.
        '''
        if self._closed_loop == closed_loop:
            self._log.debug('motor controller already set for closed loop.')
            return
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
        if not self.enabled:
            self._log.info('enabling motor controller…')
            Component.enable(self)
#           super().enable()
            if self._eyeballs_monitor:
                self._eyeballs_monitor.enable()
            if self._data_log:
                self._data_log.data('START')
                self._pfwd_data_log.data('START')
                self._sfwd_data_log.data('START')
                self._paft_data_log.data('START')
                self._saft_data_log.data('START')
            if self._external_clock:
                self._external_clock.add_callback(self._external_callback_method)
                for _motor in self._all_motors:
                    _motor.enable()
            else:
                if not self.loop_is_running:
                    self._start_loop()
            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def check_clock_active(self, timeout_sec=0.2):
        '''
        Check if external clock ticks are arriving within the expected interval. 
        Returns True if at least one tick is received within timeout_sec, False otherwise.
        
        Args:
            timeout_sec:   maximum time to wait for a tick (default 0.2s, ~4x loop period)
        '''
        if not self._external_clock:
            self._log.warning('no external clock configured; cannot check clock status.')
            return False
        initial_count = next(self._external_clock._counter)
        time.sleep(timeout_sec)
        final_count = next(self._external_clock._counter)
        ticks_received = final_count - initial_count
        if ticks_received > 0:
            self._log.info('clock active: {} ticks detected within {:4.2f}s'.format(ticks_received, timeout_sec))
            return True
        else:
            self._log.warning('no clock ticks detected within {:4.2f}s'.format(timeout_sec))
            return False

    def _start_loop(self):
        '''
        Start the loop Thread.

        If we're using an external clock, calling this method throws an exception.
        '''
        from threading import Thread

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
        '''
        Register a controller-level speed modifier.
        Only accepts lambda functions (enforced).
        modifier_fn(speeds: list[4]) -> list[4] | str | None
        '''
        if modifier_fn.__name__ != "<lambda>":
            raise TypeError('expected lambda function, not {}'.format(type(modifier_fn)))
        if exclusive:
            self._speed_modifiers.clear()
        self._speed_modifiers[name] = modifier_fn
        self._log.info('added speed modifier: {}'.format(name))

    def remove_speed_modifier(self, prefix):
        '''
        Remove a previously-registered controller-level speed modifier whose
        name either matches or is prefixed by the argument.
        '''
#       if name in self._speed_modifiers:
#           del self._speed_modifiers[name]
#           self._log.info('removed speed modifier: {}'.format(name))
        _to_remove = [key for key in self._speed_modifiers if key.startswith(prefix)]
        for key in _to_remove:
            del self._speed_modifiers[key]
            self._log.info('removed speed modifier: {}'.format(key))

    def list_speed_modifiers(self):
        '''
        Log the registered controller-level speed modifiers.
        '''
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

    def _smooth_motor_powers(self, speeds):
        '''
        Apply exponential moving average to motor powers to absorb timing jitter.
        operates on [pfwd, sfwd, paft, saft] after kinematics and normalization.

        Returns smoothed list of 4 motor speeds.

        Args:
            param speeds:  list of 4 motor speeds from mecanum kinematics
        '''
        alpha = self._motor_power_smoothing_alpha
        smoothed = []
        for i, (last, curr) in enumerate(zip(self._last_motor_speeds, speeds)):
            s = alpha * last + (1 - alpha) * curr
            # zero-crossing protection: if target is stopping, force smooth to zero
            # prevents robot from drifting due to smoothing lag
            if abs(curr) < 0.02:  # target is at or near zero
                if abs(s) < 0.05:  # smoothed value is small
                    s = 0.0
            smoothed.append(s)
        self._last_motor_speeds = smoothed
        return smoothed

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

        if not self._intent_vectors:
            intent = (0.0, 0.0, 0.0)
        else:
            intent = self._blend_intent_vectors()
        if self._post_processor:
            intent = self._post_processor.process_intent_vector(intent)
        if len(intent) != 3:
            raise ValueError('expected 3 values, not {}.'.format(len(intent)))
        # apply time-based slew limiting
        if self._slew_limiter:
            intent = self._slew_limiter.limit(intent)
        vx, vy, omega = intent
        self._blended_intent_vector = intent # for access as property
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

        # 🍀 new motor power smoothing
        if self._motor_power_smoothing_enabled:
            speeds = self._smooth_motor_powers(speeds)

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
#       self._log.info('B. intent: vx={:.3f}, vy={:.3f}, omega={:.3f} -> speeds: {}'.format(vx, vy, omega, ['{:.3f}'.format(s) for s in speeds]))
        self._pfwd_motor.target_speed = speeds[0]
        self._sfwd_motor.target_speed = speeds[1]
        self._paft_motor.target_speed = speeds[2]
        self._saft_motor.target_speed = speeds[3]
        # update motors (apply slew/PID/jerk)
        for _motor in self._all_motors:
            _motor.update_target_speed()
            if self._data_log:
                self._data_log.data('{:.2f}'.format(vx), '{:.2f}'.format(vy), '{:.2f}'.format(omega))
#               self._pfwd_data_log.data('{:.2f}'.format(self._pfwd_motor.target_speed))
#               self._sfwd_data_log.data('{:.2f}'.format(self._sfwd_motor.target_speed))
#               self._paft_data_log.data('{:.2f}'.format(self._paft_motor.target_speed))
#               self._saft_data_log.data('{:.2f}'.format(self._saft_motor.target_speed))
                _pfwd_power = self._pfwd_motor.current_power
                if _pfwd_power:
                    self._pfwd_data_log.data('{:.2f}'.format(_pfwd_power))
                _sfwd_power = self._sfwd_motor.current_power
                if _sfwd_power:
                    self._sfwd_data_log.data('{:.2f}'.format(_sfwd_power))
                _paft_power = self._paft_motor.current_power
                if _paft_power:
                    self._paft_data_log.data('{:.2f}'.format(_paft_power))
                _saft_power = self._saft_motor.current_power
                if _saft_power:
                    self._saft_data_log.data('{:.2f}'.format(_saft_power))
        # odometry update
        step_counts = {
            'pfwd': self._pfwd_motor.steps,
            'sfwd': self._sfwd_motor.steps,
            'paft': self._paft_motor.steps,
            'saft': self._saft_motor.steps,
        }
        self._odometer.update(step_counts, time.monotonic())
        _count = next(self._event_counter)
        if self._verbose:
            self.print_info(_count, vx, vy, omega)
            if _count % 20 == 0:
                self._odometer.print_info()
        self._state_change_check()
        if self._motor_loop_callback is not None:
            self._motor_loop_callback()
        # update eyeballs monitor
        if self._eyeballs_monitor:
            self._eyeballs_monitor.update()

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
            self._log.warning(Fore.WHITE + 'callback method called after disabled.')
            pass

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
        Returns True if the speed (current power) of all motors is zero, False
        if any are moving, i.e., if the motor power of any motor is greater than
        zero.
        '''
        for _motor in self._all_motors:
            if not _motor.is_stopped:
                return False
        return True

    @property
    def is_stopped_target(self):
        '''
        Returns True if the target speed of all motors is (close to) zero,
        False if any have a non-zero target speed.
        '''
        for _motor in self._all_motors:
            if not _motor.is_stopped_target_speed:
                return False
        return True

    def set_speeds(self, pfwd_speed, sfwd_speed, paft_speed, saft_speed):
        '''
        Set all four motors speeds.
        '''
        if not self.enabled:
            self._log.error('motor controller not enabled.')
            return
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
        if not self.enabled:
            self._log.error('motor controller not enabled.')
            return
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
        '''
        Update internal base per-motor target speed variable (forms the base intent).
        The actual Motor.target_speed is set in _motor_tick() after blending and modifiers.
        '''
        if not self.enabled:
            self._log.error('motor controller not enabled.')
            return
        elif not self.has_intent_vector(MotorController.BASE_NAME):
            self._log.warning('base intent vector not available.')
            return
        elif isinstance(target_speed, int):
            raise ValueError('expected target speed as float not int: {:d}'.format(target_speed))
        elif not isinstance(target_speed, float):
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

    @property
    def all_motors_are_stopped(self):
        '''
        Returns True when all motors are stopped.
        '''
        for _motor in self._all_motors:
            if not _motor.is_stopped:
                return False
        return True

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

    def coast(self):
        '''
        Stops the robot smoothly over a longer distance than braking.
        '''
        self._log.info(Fore.YELLOW + 'coast.')
        self._brake('coast', step=self._coast_step)

    def brake(self):
        '''
        Stops the robot smoothly within a short distance.
        '''
        self._log.info(Fore.YELLOW + 'brake.')
        self._brake('brake', step=self._brake_step)

    def halt(self):
        '''
        Stops the robot quickly but without a jerk.
        '''
        self._log.info(Fore.YELLOW + 'halt.')
        self._brake('halt', step=self._halt_step)

    def stop(self):
        '''
        Stops the robot abruptly.
        '''
        self._log.info(Fore.YELLOW + 'stop.')
        self._brake('stop', step=self._stop_step)

    def emergency_stop(self):
        '''
        Immediate hard stop: clear controller-level modifiers and stop every motor.
        This is not recoverable as it clears both the intent vectors and speed modifiers.
        '''
        self._log.warning('emergency stop…')
        self._brake('emergency-stop', step=self._emergency_stop_step)
        self._speed_modifiers.clear()
        self.clear_intent_vectors()
        for _motor in self._all_motors:
            _motor.stop()
        self._log.info('emergency stopped.')

    def _brake(self, name, step=0.02, closing=False):
        '''
        Brake: register a modifier to progressively reduce speeds, with the rate
        determined by the step argument.
        If closing is True this clears all intent vectors as part of shutting down.

        Severity ordering (higher step = more urgent):
            coast < brake < halt < stop < emergency_stop
        A more urgent brake request interrupts a less urgent one.
        '''
        self._log.info('brake {} with step {}'.format(name, step))
        if self.is_stopped:
            self._log.info('already stopped: brake {} with step {}'.format(name, step))
            self._log.info('already stopped.')
            self._braking_active = False
            self._current_brake_step = None
            self._is_braked = True
            return
        if self._braking_active:
            self._log.info('braking active: brake {} with step {}'.format(name, step))
            if self._current_brake_step is not None and step <= self._current_brake_step:
                self._log.info('brake request with step {} ignored; already braking with step {}'.format(step, self._current_brake_step))
                return
            else:
                self._log.info('brake request with step {} interrupting slower brake with step {}'.format(step, self._current_brake_step))
        self._braking_active = True
        self._is_braked = False
        self._current_brake_step = step
        factor = 1.0
        _step = step
        speed_signs = None

        def _brake_modifier(speeds):
            nonlocal factor, _step, speed_signs
            if speed_signs is None:
                speed_signs = [1.0 if s >= 0.0 else -1.0 for s in speeds]
            factor = max(0.0, factor - _step)
            modified = []
            for i, s in enumerate(speeds):
                scaled = s * factor
                if speed_signs[i] >= 0.0:
                    modified.append(max(0.0, scaled))
                else:
                    modified.append(min(0.0, scaled))
            if all(isclose(m, 0.0, abs_tol=1e-2) for m in modified) or factor <= 0.0:
                for _m in self._all_motors:
                    _m.stop()
                    _m.pid_controller.reset()
                if closing:
                    self._speed_modifiers.clear()
                    self.clear_intent_vectors()
                self._braking_active = False
                self._current_brake_step = None
                self._is_braked = True
                self._braking_event.set()  # Signal completion
                return MotorController.BRAKE_LAMBDA_NAME
            return modified
        self._log.info('adding brake modified for {} with step {}'.format(name, step))
        self.remove_speed_modifier(MotorController.BRAKE_LAMBDA_NAME)
        self.add_speed_modifier('{}-{}'.format(MotorController.BRAKE_LAMBDA_NAME, name),
                (lambda speeds, _fn=_brake_modifier: _fn(speeds)), exclusive=True)
        self._log.info('registered controller-level braking modifier.')
        try:
            timeout = 5.0
            self._braking_event.clear()
            if self._braking_event.wait(timeout):
                self._log.info('brake complete.')
            else:
                self._log.warning('brake timeout after {:4.2f}s'.format(timeout))
        finally:
            if MotorController.BRAKE_LAMBDA_NAME in self._speed_modifiers:
                self.remove_speed_modifier(MotorController.BRAKE_LAMBDA_NAME)
            self._braking_active = False
            self._current_brake_step = None
            self._is_braked = True

    def release_brake(self, duration_sec=1.5):
        '''
        Gradually restores motor responsiveness after a brake has been applied.
        This ramps a speed multiplier from 0.0 to 1.0 over the specified duration.
        '''
        if not self._is_braked:
            self._log.info("release_brake called, but robot is not in a braked state. Ignoring.")
            return
        self._log.info("releasing brake over {}s...".format(duration_sec))
        self._is_braked = False
        factor = 0.0
        # step per loop iteration
        step = 1.0 / (duration_sec / self._loop_delay_sec)
        def _release_modifier(speeds):
            nonlocal factor, step
            factor = min(1.0, factor + step)
            modified = [s * factor for s in speeds]
            if factor >= 1.0:
                self._log.info("brake released.")
                return MotorController.RELEASE_BRAKE_LAMBDA_NAME # request removal
            return modified
        # check that no other brake/release modifiers are active
        self.remove_speed_modifier(MotorController.BRAKE_LAMBDA_NAME)
        self.remove_speed_modifier(MotorController.RELEASE_BRAKE_LAMBDA_NAME)
        self.add_speed_modifier(MotorController.RELEASE_BRAKE_LAMBDA_NAME,
                (lambda speeds, _fn=_release_modifier: _fn(speeds)), exclusive=True)

    def disable(self):
        '''
        Disable the motor controller, first braking the motors to a stop.
        '''
        if self.enabled:
            if self._eyeballs_monitor:
                self._eyeballs_monitor.disable()
            if not self.is_stopped:
                if self._use_graceful_stop:
                    self._log.info('🍀 calling coast…')
                    self._brake('coast', step=self._coast_step, closing=True)
                else:
                    self._log.info('🍀 calling brake…')
                    self._brake('stop', step=self._stop_step, closing=True)
                self._log.info('🍀 waiting 3 seconds…')
                time.sleep(1.5)
#               self.set_speeds(0.0, 0.0, 0.0, 0.0)
                time.sleep(1.5)
            else:
                self._log.info('🍀 motors were already stopped.')
            if not self.is_stopped_target:
                self._log.warning('calling emergency stop after failing to stop normally…')
                self.emergency_stop()
            # reset smoothing state
            self._last_motor_speeds = [0.0, 0.0, 0.0, 0.0]
            super().disable()
            [ motor.disable() for motor in self._all_motors ]
            if self._external_clock:
                self._log.debug('disabling by removing external clock callback…')
                self._external_clock.remove_callback(self._external_callback_method)
            if self._loop_enabled:
                self._log.info('disabling by stopping loop…')
                time.sleep(1)
                self._stop_loop()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

    def close(self):
        '''
        Closes the motor controller.
        '''
        if not self.closed:
            if self._data_log:
                self._data_log.data('END')
                self._pfwd_data_log.data('END')
                self._sfwd_data_log.data('END')
                self._paft_data_log.data('END')
                self._saft_data_log.data('END')
            super().close()
            self._log.info('closed.')
        else:
            self._log.warning('already closed.')

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
