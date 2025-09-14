#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-04
# modified: 2025-06-26

import sys
import uasyncio as asyncio
from asyncio import ThreadSafeFlag
from math import isclose
import utime
from pyb import Timer

from colorama import Fore, Style

from logger import Logger, Level
from config_loader import ConfigLoader
from motor import Motor, ChannelUnavailableError
from slew_limiter import SlewLimiter
from zero_crossing_handler import ZeroCrossingHandler
from pid import PID
from mode import Mode
from util import Util

class MotorController:
    '''
    A controller for four brushless motors. This operates in both open- and
    closed loop mode, and optionally supports slew limiting and zero-crossing
    behaviours.

    Args:
        config:          The application-level configuration.
        status:          The Status indicator.
        level:           The log level.
    '''
    def __init__(self, config=None, status=None, level=Level.INFO):
        self._log = Logger('motor-ctrl', level=level)
        self._log.info('initialising Motor Controller…')
        if config is None:
            raise ValueError('no configuration provided.')
        self._config     = config
        self._status     = status
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _app_cfg         = config["kros"]["application"]
        _cfg             = config["kros"]["motor_controller"]
        _motors_cfg      = config["kros"]["motors"]
        _slew_cfg        = config["kros"]["slew_limiter"]
        _zch_cfg         = config["kros"]["zero_crossing_handler"]
        self._verbose    = _app_cfg["verbose"]
        self._max_motor_speed         = _cfg.get('max_motor_speed')
        self._use_closed_loop         = _cfg.get('use_closed_loop', True)
        self._enable_slew_limiter     = _slew_cfg['enabled']                  # True
        self._max_delta_rpm_per_sec   = _slew_cfg['max_delta_rpm_per_sec']    # closed loop: 120.0
        self._max_delta_speed_per_sec = _slew_cfg['max_delta_speed_per_sec']  # open loop: 100.0
        self._enable_zc_handler       = _zch_cfg['enabled']                   # True
        # variables ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._motors          = {}
        self._motor_list      = []
        self._motor_numbers   = [0, 1, 2, 3]
        self._slew_limiters   = {}
        self._pid_controllers = {} # PID instances
        self._zc_handlers     = {}
        self._hard_reset_delay_sec = 3
        self._enabled    = False
        self._paused_by_display = False
        self._feedforward_gain = 0.0
        if self._use_closed_loop:
            self._feedforward_gain = config['kros']['motor_controller']['feedforward_gain']
            self._motor_target_rpms = {}   # target RPM for each motor
            self._pid_gains = _cfg.get('pid_gains', {'Kp': 0.5, 'Ki': 0.01, 'Kd': 0.01}) # default PID gains if not in config
            _pid_timer_number = _cfg['pid_timer_number']
            _pid_timer_freq   = _cfg['pid_timer_frequency']
            self._pid_timer = Timer(_pid_timer_number, freq=_pid_timer_freq)
            self._pid_signal_flag = ThreadSafeFlag()
            self._last_global_pid_cycle_time = utime.ticks_us() # initialize last global update time
            asyncio.create_task(self._run_pid_task())
            self._log.info(Fore.GREEN + 'closed loop enabled; PID timer {} configured with frequency of {}Hz.'.format(_pid_timer_number, _pid_timer_freq))
        else:
            self._log.info(Fore.GREEN + 'open-loop enabled; PID disabled.')
        self._log.info(Fore.MAGENTA + 'verbose: {}'.format(self._verbose))
        try:
            self._log.debug('configuring timers…')
            # RPM timer ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            _rpm_timer_enabled = _cfg['rpm_timer_enable']
            if _rpm_timer_enabled: # not currently used (we use hardware callbacks on the encoder pins)
                _rpm_timer_number  = _cfg['rpm_timer_number']
                _rpm_timer_freq    = _cfg['rpm_timer_frequency']
                self._rpm_timer    = Timer(_rpm_timer_number, freq=_rpm_timer_freq)
                self._log.info(Fore.MAGENTA + 'configured Timer {} for RPM calculation at {}Hz'.format(_rpm_timer_number, _rpm_timer_freq))
            else:
                self._rpm_timer    = None
            # Logging timer ┈┈┈┈┈┈┈┈┈┈┈┈
            self._logging_enabled = _cfg['log_timer_enable']
            _log_timer_number  = _cfg['log_timer_number']
            _log_timer_freq    = _cfg['log_timer_frequency']
            self._logging_task = None # store the logging task to manage it
            self._loop         = None # asyncio loop instance
            # configure enable/disable motors
            enable_m0 = _cfg['enable_m0']
            enable_m1 = _cfg['enable_m1']
            enable_m2 = _cfg['enable_m2']
            enable_m3 = _cfg['enable_m3']
            motors_enabled = (enable_m0, enable_m1, enable_m2, enable_m3)
            self._log.info(Fore.MAGENTA + 'enable motors m0={}; m1={}; m2={}; m3={}'.format(enable_m0, enable_m1, enable_m2, enable_m3))
            # motors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            _pwm_timer_number = _cfg["pwm_timer_number"]
            _pwm_frequency    = _cfg['pwm_frequency']
            _pwm_timer_af     = _cfg["pwm_timer_af"]
            _pwm_timer = Timer(_pwm_timer_number, freq=_pwm_frequency)
            self._log.info(Fore.MAGENTA + 'configured PWM on timer {}, af={} polling at {:,}Hz'.format(_pwm_timer_number, _pwm_timer_af, _pwm_frequency))
            _safe_threshold   = _slew_cfg['safe_threshold'] # 10.0
            _max_delta_per_sec = self._max_delta_rpm_per_sec if self._use_closed_loop else self._max_delta_speed_per_sec
            for index in range(4):
                if not motors_enabled[index]:
                    continue
                _motor_key = "motor{}".format(index)
                _motor_cfg = _motors_cfg[_motor_key]
                _name = _motor_cfg["name"]
                self._log.debug('configuring motor {}…'.format(_name))
                _motor = Motor(_motor_cfg, pwm_timer=_pwm_timer, af=_pwm_timer_af, max_speed=self._max_motor_speed if self._use_closed_loop else 100.0)
                self._motors[index] = _motor
                self._motor_list.append(_motor)
                # instantiate PID controller for this motor if closed-loop enabled
                if self._use_closed_loop:
                    self._pid_controllers[index] = PID(_name, config, level=level)
                    self._motor_target_rpms[index] = 0.0
                # instantiate SlewLimiter for each motor
                if self._enable_slew_limiter:
                    self._slew_limiters[index] = SlewLimiter(name=_name, max_delta_per_sec=_max_delta_per_sec, safe_threshold=_safe_threshold)
                # instantiate ZeroCrossingHandler for each motor
                if self._enable_zc_handler:
                    self._zc_handlers[index] = ZeroCrossingHandler(
                        self._motors[index],
                        self._pid_controllers[index],
                        self._slew_limiters.get(index)
                    )
            self._log.info('ready.')
        except ChannelUnavailableError:
            # hard reset if coms are down
            self._log.fatal('cannot start: ' + Fore.RED + 'performing hard reset in {} seconds…'.format(self._hard_reset_delay_sec))
            import machine, time
            time.sleep(self._hard_reset_delay_sec)
            machine.reset()
        except Exception as e:
            self._log.error('{} raised by motor controller constructor: {}'.format(type(e), e))
            sys.print_exception(e)

    def _rpm_timer_callback(self, timer):
        '''
        This callback's primary role is to signal when PID updates are needed.
        Not currently used in this implementation.
        '''
        pass

    def pause(self):
        '''
        Called by PayloadRouter when a blocking display update is about to start.
        Sets a flag to prevent further PID updates, and pre-emptively resets PID's
        derivative state to the current motor RPM to avoid a jump when resuming.
        '''
        self._paused_by_display = True
        for motor in self.motors:
            if motor.id in self._pid_controllers:
                pid_ctrl = self._pid_controllers[motor.id]
                current_motor_rpm = motor.rpm # current RPM at moment of pause
                pid_ctrl.reset_derivative_state(current_motor_rpm)

    def resume(self):
        '''
        Called by PayloadRouter after a blocking display update has finished.
        Resynchronizes global PID timing and allows PID updates to resume.
        '''
        self._last_global_pid_cycle_time = utime.ticks_us()
        self._paused_by_display = False

    async def _run_pid_task(self):
        self._log.info("starting asynchronous PID control task, driven by hardware timer.")
        while True:
            await self._pid_signal_flag.wait()
            self._pid_signal_flag.clear() # Clear the flag for the next
            if self._paused_by_display:
                # if paused, we acknowledge the signal but skip PID update and
                # timing update. The loop will resume here after the display
                # unfreezes, but it will wait for the next signal without
                # acting on this one.
                continue # skip rest of loop and go back to `await self._pid_signal_flag.wait()`
            current_time = utime.ticks_us()
            global_cycle_dt_us = utime.ticks_diff(current_time, self._last_global_pid_cycle_time)
            self._last_global_pid_cycle_time = current_time # update for next cycle
            for motor in self.motors:
                if motor.id in self._pid_controllers:
                    pid_ctrl = self._pid_controllers[motor.id]
                    # use ZCH if enabled and active for this motor
                    if self._enable_zc_handler and motor.id in self._zc_handlers and self._zc_handlers[motor.id].is_active:
                        target_rpm_signed = self._zc_handlers[motor.id].get_effective_pid_target_rpm
                    else:
                        target_rpm_signed = self._motor_target_rpms.get(motor.id, 0.0)
                    current_motor_rpm = motor.rpm
                    pid_ctrl.setpoint = target_rpm_signed
                    # call PID update, returns output in RPM (range: -motor.max_speed to +motor.max_speed)
                    pid_output_rpm = pid_ctrl.update(current_motor_rpm, global_cycle_dt_us)
                    # feedforward
                    feedforward = self._feedforward_gain * target_rpm_signed
                    combined_output_rpm = pid_output_rpm + feedforward
                    # scale PID output (RPM) to percent (-100 to 100)
                    speed_percent = (combined_output_rpm / motor.max_speed) * 100
                    # clip to range [-100, 100]
                    speed_percent = Util.clip(speed_percent, -100, 100)
                    # apply zero-speed deadband
                    if abs(speed_percent) < pid_ctrl._deadband:
                        speed_percent = 0.0 
                    _speed = int(round(speed_percent))
                    motor.speed = _speed

    @property
    def motor_ids(self):
        '''
        Return a list of motor IDs (keys) for all instantiated motors.
        '''
        return list(self._motors.keys())

    @property
    def motors(self):
        '''
        Return a list of all instantiated motors.
        '''
        return self._motor_list

    @property
    def enabled(self):
        return self._enabled

    def enable_rpm_logger(self, interval_ms: int = 1000):
        '''
        Starts the asynchronous RPM logger.
        '''
        if self._loop is None:
            self._loop = asyncio.get_event_loop()
        if not self._logging_enabled:
            self._log.info(Fore.MAGENTA + "starting RPM logger with interval: {}ms.".format(interval_ms))
            self._logging_task = self._loop.create_task(self._rpm_logger_coro(interval_ms))
            self._logging_enabled = True
        else:
            self._log.info("RPM logger already running.")

    def disable_rpm_logger(self):
        '''
        Cancels the RPM logger task if running.
        '''
        if self._logging_enabled and self._logging_task is not None:
            self._logging_task.cancel()
            self._logging_task = None
            self._logging_enabled = False
            self._log.info("RPM logger stopped.")
        else:
            self._log.info("RPM logger was not running.")

    def _get_motor_target_rpms(self, motor_id):
        if self._use_closed_loop:
            return self._motor_target_rpms[motor_id]
        else:
            return -1.0

    async def _rpm_logger_coro(self, interval_ms: int):
        '''
        Periodically logs the current RPM and tick count for all motors.
        If there is no change in the log message it is printed in DIM.

        Args:
            interval_ms: The interval between calls to the logger.
        '''
        if self._verbose:
            self._log.info(Fore.MAGENTA + "called RPM logger coro with interval: {}ms.".format(interval_ms))
            _pid_cfg = self._config['kros']['pid']
            _kp = _pid_cfg['kp']
            _ki = _pid_cfg['ki']
            _kd = _pid_cfg['kd']
            self._log.info("pid config: " + Fore.MAGENTA + "kp={:>5.3f}; ki={:>5.3f}; kd={:>5.3f}; ff={:>3.2f}; limits: {}🠊 {}".format(
                    _kp, _ki, _kd, self._feedforward_gain, -self._max_motor_speed, self._max_motor_speed))
        try:
            last_rpm_values = None
            while self._logging_enabled:
                if self._motor_list:
                    rpm_values = ", ".join(
                        (
                            "{:2s} | ".format(motor.name) 
                                + "pid:{:7.3f} {:7.3f} {:7.3f} | SP:{:6.1f} OUT:{:6.1f} | ".format(*self._pid_controllers[motor.id].info)
                                + "PWM: {:5.1f}% {:6.1f} RPM".format(motor.speed, motor.rpm)
                            if self._use_closed_loop else
                                "{:2s} | RPM:{:8.2f} | Ticks:{:5d}".format(motor.name, motor.rpm, motor.tick_count)
                        )
                        for motor in self._motor_list
                    )
                    if self._verbose:
                        if  last_rpm_values != rpm_values:
                            self._log.info('set: ' + Fore.MAGENTA + "{}".format(rpm_values))
                        else:
                            self._log.info('set: ' + Fore.MAGENTA + Style.DIM + "{}".format(rpm_values))
                    last_rpm_values = rpm_values
                await asyncio.sleep_ms(interval_ms)
        except asyncio.CancelledError:
            self._log.info("RPM logger task cancelled.")
        finally:
            self._log.info(Fore.MAGENTA + "rpm_logger_coro done.")

    def enable(self):
        '''
        Enables the motor controller.
        '''
        if self.enabled:
            self._log.warning(Style.DIM + "motor controller already enabled.")
        else:
            for motor in self.motors:
                motor.enable()
            self._enabled = True
            if self._rpm_timer:
                self._rpm_timer.callback(self._rpm_timer_callback)
            if self._use_closed_loop:
                # Configure the _pid_timer to call our ISR callback, set the flag from within the lambda callback
                self._pid_timer.callback(lambda t: self._pid_signal_flag.set())
            self.enable_rpm_logger() # already starts timer & logging
            self._log.info("motor controller enabled.")

    def get_motor(self, index):
        '''
        Returns the corresponding motor.

        Args:
            index (int):  The motor number (0-3).
        '''
        if isinstance(index, int):
            return self._motors[index]
        raise ValueError('expected an int, not a {}'.format(type(index)))

    @staticmethod
    def _apply_mode(base_power, mode):
        return tuple(p * m for p, m in zip(base_power, mode.speeds))

    def go(self, payload):
        '''
        Set the navigation mode and speeds for all motors.

        Args:
            payload:  the transmitted payload, with mode and four motor speeds, in order: pfwd, sfwd, paft, saft
        '''
        mode = Mode.from_code(payload.code)
        speeds = self._remap_speeds(payload.speeds)
        transform = MotorController._apply_mode(speeds, mode)
        if self._status:
            self._status.motors(transform)
        # slew limiter logic
        if self._enable_slew_limiter:
            transform = list(transform)  # convert to list for mutability
            for i in range(len(transform)):
                if i in self._slew_limiters:
                    transform[i] = self._slew_limiters[i].limit(transform[i])
            self._log.debug('slew-limited transform: {}'.format(transform))
        # closed-loop logic
        if self._use_closed_loop:
            for i in self._pid_controllers:
                target_rpm = transform[i]
                pid = self._pid_controllers[i]
                current_rpm = self._motors[i].rpm
                self._motor_target_rpms[i] = float(target_rpm)
                # ZeroCrossingHandler logic
                if self._enable_zc_handler and i in self._zc_handlers:
                    zc_handler = self._zc_handlers[i]
                    zc_handler.handle_new_command(target_rpm, current_rpm, pid)
                    # do not set self._motors[i].speed directly; handler/PID will manage transitions
                else:
                    # if ZCH is disabled, fallback to direct control as before
                    if pid.deadband_enabled and abs(target_rpm) < pid.deadband:
                        self._motors[i].speed = 0
                    else:
                        self._motors[i].speed = target_rpm
            self._log.debug('setting target RPMs to {}'.format(transform))
        else:
            self._set_motor_speed(transform)

    def _remap_speeds(self, speeds):
        '''
        To swap port-for-starbard, remap the order of the speed tuple:

            (M0, M1, M2, M3) → (M1, M0, M3, M2)
        '''
        return (speeds[1], speeds[0], speeds[3], speeds[2])

    def x_go(self, payload):
        '''
        Set the navigation mode and speeds for all motors.

        Args:
            payload:  the transmitted payload, with mode and four motor speeds, in order: pfwd, sfwd, paft, saft
        '''
        mode = Mode.from_code(payload.code)
#       if mode is Mode.STOP:
#           self.stop()
#           return
        speeds = payload.speeds
        transform = MotorController._apply_mode(speeds, mode)
#       self._log.debug('go: {}; speeds: {}; type: {}; transform: {}'.format(mode, speeds, type(transform), transform))
        if self._status:
            self._status.motors(transform)
        if self._enable_slew_limiter:
            transform = list(transform) # convert to list for mutability
            for i in range(len(transform)):
                if i in self._slew_limiters:
                    transform[i] = self._slew_limiters[i].limit(transform[i])
            self._log.debug('slew-limited transform: {}'.format(transform))
        if self._use_closed_loop:
            for i in self._pid_controllers:
                target_rpm = transform[i]
                pid = self._pid_controllers[i]
                if pid.deadband_enabled and abs(target_rpm) < pid.deadband:
                    self._motor_target_rpms[i] = 0.0
                    self._motors[i].speed = 0
                else:
                    self._motor_target_rpms[i] = float(target_rpm)
                    self._motors[i].speed = target_rpm
            self._log.debug('setting target RPMs to {}'.format(transform))
        else:
            self._set_motor_speed(transform)

    def _set_motor_speed(self, speeds):
        '''
        An internal call to set the speeds for all motors.

        Args:
            speed: Sets motor speed as a percentage (0–100).
        '''
        if not self.enabled:
            raise RuntimeError('motor controller not enabled.')
        if self._verbose:
            self._log.info('set speeds to {}'.format(speeds))
        for motor, speed in zip(self._motor_list, list(speeds)):
            motor.speed = speed

    async def accelerate(self, target_speed, step=1, delay_ms=50):
        if self._use_closed_loop:
            self._log.warning("`accelerate` with closed-loop is a simplified placeholder. Setting all motor target RPMs to `target_speed` (magnitude).")
            for motor_id in self._motor_target_rpms:
                self._motor_target_rpms[motor_id] = abs(float(target_speed))
        else:
            done = False
            while not done:
                done = True
                for motor in self._motor_list:
                    current_pwm = motor.speed
                    target_pwm = target_speed

                    if current_pwm == target_pwm:
                        continue
                    done = False

                    delta = target_pwm - current_pwm
                    direction_change_step = 1 if delta > 0 else -1
                    new_pwm = current_pwm + direction_change_step * min(step, abs(delta))
                    motor.speed = new_pwm
                await asyncio.sleep_ms(delay_ms)

    async def decelerate_to_stop(self, step=1, delay_ms=50):
        '''
        Gradually slow one or more motors to a stop (speed = 100).

        Args:
            step:          The speed increment per step.
            delay_ms:      The delay in milliseconds between steps.
        '''
        if self._verbose:
            self._log.info("decelerating motors to stop…")
        await self.accelerate(target_speed=Motor.STOPPED, step=step, delay_ms=delay_ms)

    def log_pin_configuration(self):
        '''
        Print pin mappings for each motor using clean, labeled pin names.
        '''
        self._log.info("Motor        PWM           Dir           Enc")
        col_widths = [12, 18, 18, 18]
        for num in sorted(self._motors):
            m = self._motors[num]
            line = "{}{:<{}} {:<{}} {:<{}} {:<{}}{}".format(
                Fore.CYAN, "Motor {}".format(num), col_widths[0],
                Fore.GREEN + m.pwm_pin_name, col_widths[1],
                Fore.GREEN + m.direction_pin_name, col_widths[2],
                Fore.GREEN + (m.encoder_pin_name or "N/A"), col_widths[3],
                Style.RESET_ALL
            )
            self._log.info(line)

    def stop(self):
        '''
        Stop all motors.
        '''
        for motor in self.motors:
            motor.stop()
        if self._use_closed_loop:
            for pid_ctrl in self._pid_controllers.values():
                pid_ctrl.reset()
        if self._verbose:
            self._log.info("all motors stopped.")
        return True

    def disable(self):
        '''
        Stop and disable all motors, then disable the motor controller.
        '''
        if not self.enabled:
            self._log.warning("motor controller already disabled.")
        else:
            self._enabled = False
            _ = self.stop()
            if self._use_closed_loop:
                self._pid_timer.callback(None)
            for motor in self.motors:
                motor.disable()
                if self._enable_slew_limiter and motor.id in self._slew_limiters:
                    self._slew_limiters[motor.id].reset()
            if self._use_closed_loop and motor.id in self._pid_controllers:
                self._pid_controllers[motor.id].reset()
            if self._rpm_timer:
                self._rpm_timer.callback(None)
            self._log.info("motor controller disabled.")

    def close(self):
        '''
        Stop and disable motors, their respective callbacks, and close the motor controller.
        '''
        self.disable()
        for motor in self.motors:
            motor.close()
        self._log.info("closed.")

#EOF
