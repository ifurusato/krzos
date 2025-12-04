#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-04
# modified: 2025-07-08
#
# A 12V JGA25-2418-1245 (25mm dia., 18mm length) DC brushless motor:
#
#  * motor speed: 4500rpm; * gear ratio: 78:1; shaft speed: 55rpm
#  * rated load: 40rpm 0.08A 0.6kg.cm torque; stall: 3 kg.cm, 0.5A
#
# source: https://precisionminidrives.com/product/7kg-cm-micro-brushless-worm-gearmotors-12v-24v-model-nfp-jga25-2418-ce

import utime
from math import isclose
from pyb import Pin, ExtInt, Timer
from colorama import Fore, Style

from logger import Logger, Level
from util import Util

class Motor:
    STOPPED           = 0
    FULL_SPEED        = 100
    DIRECTION_FORWARD = 1
    DIRECTION_REVERSE = 0

    '''
    This class supports control over a single JGA25 DC brushless motor, with its
    own internal PWM controller, a direction input pin, and a single encoder pin.

    The motor speed is controlled by an inverted PWM signal, i.e., with a 100%
    PWM duty cycle the motor is stopped, a 0% duty cycle is full speed. In order
    to not be confused by this logic, the hardware Timer's PWM channel is defined
    using `Timer.PWM_INVERTED`.

    Direction of the motor is set by a direction pin, with forward as 1/high,
    reverse as 0/low.

    The motor has a single encoder pin so it cannot determine direction via phase
    as is typically done with quadratic encoders. It's therefore possible to "fool"
    the motor by changing the direction pin quickly while the motor is at speed in
    one direction.

    Args:
        config:      configuration specific to this motor
        pwm_timer:   the hardware Timer used for all motors
        af:          the Alternate Function for the Pins (may be None)
        max_speed:   the maximum target speed permitted
        level:       the logging level
    '''
    def __init__(self, config=None, pwm_timer=None, af=None, max_speed=None, level=Level.INFO):
        try:
            self._log = None
            if config is None:
                raise TypeError('no configuration provided.')
            if pwm_timer is None:
                raise TypeError('no PWM timer provided.')
            self._id   = config["id"]
            self._name = config["name"]
            self._log  = Logger('motor-{}'.format(self._name), level=level)
            self._log.debug('initialising motor {}…'.format(self._name))
            self._enabled         = False
            self._reverse         = config["reverse"]
            self._tick_count      = 0
            self._rpm             = 0
            self._max_speed       = max_speed
            self._duty_cycle      = 0
            self._update_interval = 0.1
            self._ticks_per_output_rev      = 1350    # observed (ticks per motor rev * gear ratio)
            self._no_tick_timeout_us        = 70_000  # how much time we wait before declaring motor stopped (at 6 RPM this would be 37,037)
            self._soft_stop_threshold_rpm   = 6.0     # RPM below which motor is considered 'stopped enough' for direction change
            # setup PWM channel with pin
            pwm_pin = config["pwm_pin"]
            if af is None:
                self._pwm_pin     = Pin(pwm_pin)
            else:
                self._pwm_pin     = Pin(pwm_pin, mode=Pin.AF_PP, af=af)
            self._pwm_pin_name    = config["pwm_pin_name"]
            pwm_channel = config["pwm_channel"]
            self._pwm_channel = pwm_timer.channel(pwm_channel, Timer.PWM_INVERTED, pin=self._pwm_pin)
            self._speed = self.STOPPED # speed as 0-100%
            self._pwm_channel.pulse_width_percent(0) # confirm motor starts off
            # setup direction GPIO pin
            direction_pin = config["direction_pin"]
            self._direction_pin = Pin(direction_pin, Pin.OUT)
            self._direction_pin.value(Motor.DIRECTION_FORWARD)
            self._direction_pin_name = config["direction_pin_name"]
            # setup encoder pin
            encoder_pin = config["encoder_pin"]
            self._encoder_pin = Pin(encoder_pin, Pin.IN, Pin.PULL_UP)
            self._encoder_pin_name = config["encoder_pin_name"]
            # variables for utime-based interval and debouncing (NEW)
            self._last_encoder_pulse_us = utime.ticks_us() # Initialize with current time for first pulse
            self._debounce_period_us = 500  # 0.5 milliseconds debounce period. Adjust if needed.
            self._last_calculated_interval_us = 0 # Stores the interval for RPM calculation in main loop
#           self._encoder_pin.irq(trigger=Pin.IRQ_FALLING, handler=self._encoder_callback)
            try:
                self._encoder_irq = ExtInt(self._encoder_pin, ExtInt.IRQ_FALLING, Pin.PULL_UP, self._encoder_callback)
            except ValueError as e: # e.g., ExtInt vector 6 is already in use
                self._log.error('motor IRQ already in use; ' + Style.BRIGHT + 'hard reset required.')
                raise ChannelUnavailableError('motor IRQ already in use; hard reset required.')
            self._encoder_irq.disable()
            # info
            self._log.info('motor {} PWM timer: '.format(self._name) + Fore.GREEN + '{}'.format(pwm_timer))
            self._log.info('motor {} channel:   '.format(self._name) + Fore.GREEN + '{}'.format(self._pwm_channel))
            self._log.info('motor {} pins: PWM: '.format(self._name) + Fore.GREEN + '{}'.format(pwm_pin)
                                         + Fore.CYAN + '; direction: ' + Fore.GREEN + '{}'.format(direction_pin)
                                         + Fore.CYAN + '; encoder: ' + Fore.GREEN + '{}'.format(encoder_pin))
            self._log.info('maximum motor speed: ' + Fore.GREEN + '{} rpm'.format(max_speed))
            self._log.info('motor {} ready.'.format(self._name))
        except Exception as e:
            if self._log:
                self._log.error('{} raised by motor: {}'.format(type(e), e))
            else:
                print('{} raised by motor: {}'.format(type(e), e))
            raise

    def _pin_irq_callback(self, arg):
        self._irq_count += 1
        print(self._irq_count)

    @property
    def id(self):
        return self._id

    @property
    def name(self):
        return self._name

    @property
    def enabled(self):
        return self._enabled

    @property
    def tick_count(self):
        return self._tick_count

    @property
    def direction(self):
        return self._direction_pin.value()

    @direction.setter
    def direction(self, value):
        physical_pin_state_to_set = value ^ self._reverse
#       physical_pin_state_to_set = value
        current_physical_pin_state = self._direction_pin.value()
        if physical_pin_state_to_set == current_physical_pin_state:
            return
        self._log.info('direction changed to '
                + Fore.YELLOW + '{}'.format('FORWARD' if value == Motor.DIRECTION_FORWARD else 'REVERSE')
                + Fore.CYAN + ': (pin {} -> {}); '.format(current_physical_pin_state, physical_pin_state_to_set)
                + 'speed: {}'.format(self.speed))
        self._direction_pin.value(physical_pin_state_to_set)

    @property
    def max_speed(self):
        return self._max_speed

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, value):
        '''
        Set the motor speed as a percentage between -100 and 100.
        Handles direction pin automatically if value is negative.
        Always uses absolute value for PWM.
        '''
        if not self.enabled:
            raise Exception('cannot set speed: motor {} not enabled.'.format(self._name))
        # determine direction from sign
        if isclose(value, 0.0, abs_tol=1.5):
            intended_direction = self.direction
        elif value < 0:
            intended_direction = Motor.DIRECTION_REVERSE
            value = abs(value)
        else:
            intended_direction = Motor.DIRECTION_FORWARD
        # clip value to 0..100 percent
        value = Util.clip(value, 0, 100)
        # only allow direction change if RPM is near stopped, or if it's same direction
        if (self.direction != intended_direction and abs(self.rpm) < self._soft_stop_threshold_rpm) \
                or (self.direction == intended_direction):
            self.direction = intended_direction
        else:
            self._log.debug('direction change ignored.')
            pass
        self._speed = value if intended_direction == Motor.DIRECTION_FORWARD else -value
        self._duty_cycle = value
        self._pwm_channel.pulse_width_percent(self._duty_cycle)

    @property
    def duty_cycle(self):
        return self._duty_cycle

    @property
    def rpm(self):
        '''
        Calculate the motor RPM and return the value as a property.
        '''
        if self._last_calculated_interval_us == 0 or \
                    utime.ticks_diff(utime.ticks_us(), self._last_encoder_pulse_us) > self._no_tick_timeout_us:
            self._last_calculated_interval_us = 0
            self._rpm = 0.0
            return self._rpm
        pulses_per_second = 1_000_000 / self._last_calculated_interval_us
        revolutions_per_second = pulses_per_second / self._ticks_per_output_rev
        calculated_rpm_magnitude = revolutions_per_second * 60.0
        if (self.direction == Motor.DIRECTION_REVERSE) ^ self._reverse:
            calculated_rpm_magnitude = -calculated_rpm_magnitude
        self._rpm = calculated_rpm_magnitude
        return self._rpm

    def _encoder_callback(self, pin):
        current_time_us = utime.ticks_us()
        last_pulse_time = self._last_encoder_pulse_us
        debounce_period = self._debounce_period_us
        raw_interval_us = utime.ticks_diff(current_time_us, last_pulse_time)
        if raw_interval_us >= debounce_period:
            self._last_encoder_pulse_us = current_time_us
            if self.direction == Motor.DIRECTION_FORWARD:
                self._tick_count += 1
            else:
                self._tick_count -= 1
            self._last_calculated_interval_us = raw_interval_us

    def stop(self):
        '''
        Stop the motor.
        '''
        self.speed = Motor.STOPPED
        self._log.info('stopped.')

    def enable(self):
        '''
        Enable the motor and its IRQ.
        '''
        if self.enabled:
            self._log.warning("motor already enabled.")
        else:
            self._enabled = True
            self._encoder_irq.enable()
            # anything?
            self._log.info("motor enabled.")

    def disable(self):
        '''
        Disable the motor and its IRQ.
        '''
        if self.enabled:
            self._enabled = False
            self._encoder_irq.disable()
            self._log.info("motor disabled.")
        else:
            self._log.warning("motor already disabled.")

    def close(self):
        '''
        Disable and close the motor, disable the callback.
        '''
        if self.enabled:
            self.disable()
        self._encoder_irq = None
        self._log.info('disabled.')

class ChannelUnavailableError(Exception):
    '''
    Thrown when a required Timer channel is not available.
    '''
    def __init__(self, message):
        super().__init__(message)

#EOF
