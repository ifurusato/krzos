#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2023-05-01
# modified: 2025-11-15

import sys
import time
import itertools
import traceback
import numpy as np
from math import isclose
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.event import Event
from behave.async_behaviour import AsyncBehaviour
from hardware.easing import Easing
from hardware.roam_sensor import RoamSensor
from hardware.digital_pot import DigitalPotentiometer
from hardware.motor_controller import MotorController

class Roam(AsyncBehaviour):
    NAME = 'roam'
    '''
    Implements a forward roaming behaviour with obstacle-based speed scaling.

    Roam moves the robot forward while monitoring front obstacles via RoamSensor.
    Speed is scaled based on obstacle distance:
    - Far obstacles (> max_distance): full speed
    - Near obstacles (< min_distance): stop
    - Between: proportional scaling

    Roam does NOT handle steering or rotation. It only controls forward motion (vy).
    Other behaviors (e.g., Scout) handle rotation (omega) independently.
    '''
    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Roam.NAME, level)
        _component_registry = Component.get_registry()
        _motor_controller = _component_registry.get(MotorController.NAME)
        if _motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        AsyncBehaviour.__init__(self, self._log, config, message_bus, message_factory, _motor_controller, level=level)
        self.add_event(Event.AVOID)
        # configuration
        _cfg = config['kros'].get('behaviour').get('roam')
        self._counter = itertools.count()
        self._verbose = _cfg.get('verbose', False)
        self._use_color = True
        self._default_speed       = _cfg.get('default_speed', 0.8)
        self._priority            = _cfg.get('default_priority', 0.3) 
        self._use_dynamic_speed   = _cfg.get('use_dynamic_speed', True)
        self._deadband_threshold  = _cfg.get('deadband_threshold', 0.07)
        _easing_value = _cfg.get('obstacle_easing', 'SQUARE_ROOT')
        self._obstacle_easing = Easing.from_string(_easing_value)
        self._log.info('obstacle easing function: {}'.format(self._obstacle_easing.name))
        # roam sensor configuration
        _rs_cfg = config['kros'].get('hardware').get('roam_sensor')
        self._min_distance = _rs_cfg.get('min_distance')
        self._max_distance = _rs_cfg.get('max_distance')
        # state variables
        self._front_distance = 0.0
        self._last_amplitude = 0.0
        # component access
        self._roam_sensor = _component_registry.get(RoamSensor.NAME)
        if self._roam_sensor is None:
            self._log.info('creating Roam sensor…')
            self._roam_sensor = RoamSensor(config, level=Level.INFO)
        else:
            self._log.info('using existing Roam sensor.')
        # digital pot for dynamic speed control
        self._digital_pot = None
        if self._use_dynamic_speed:
            self._digital_pot = _component_registry.get(DigitalPotentiometer.NAME)
        self._log.info('ready.')

    @property
    def name(self):
        return Roam.NAME

    @property
    def is_ballistic(self):
        return False

    def callback(self):
        raise NotImplementedError('callback unsupported in Roam.')

    def execute(self, message):
        raise NotImplementedError('execute unsupported in Roam.')

    def start_loop_action(self):
        pass

    def stop_loop_action(self):
        pass

    def _dynamic_set_default_speed(self):
        '''
        Updates default speed from digital potentiometer if available.
        '''
        if self._digital_pot:
            _speed = self._digital_pot.get_scaled_value(False)
            if isclose(_speed, 0.0, abs_tol=0.08):
                self._digital_pot.set_black()
                self._default_speed = 0.0
            else:
                self._digital_pot.set_rgb(self._digital_pot.value)
                self._default_speed = _speed

    async def _poll(self):
        '''
        The asynchronous poll, returns the intent vector.
        '''
        try:
            if next(self._counter) % 5 == 0:
                self._dynamic_set_default_speed()
            return self._update_intent_vector()
        except Exception as e:
            self._log.error("{} thrown while polling: {}\n{}".format(type(e), e, traceback.format_exc()))
            self.disable()
            return (0.0, 0.0, 0.0)

    def _update_intent_vector(self):
        '''
        Update the intent vector based on forward motion and obstacle detection.

        Roam only controls forward motion (vy). No lateral (vx) or rotational (omega).

        Robot-relative components of the vector:
            vx:    lateral velocity (always 0.0 for Roam)
            vy:    longitudinal velocity (forward/backward), scaled by obstacles
            omega: angular velocity (always 0.0 for Roam)
            
        Returns (vx, vy, omega) tuple.
        '''
        if self._motor_controller.braking_active:
            self._log.debug('braking active: intent vector suppressed')
            return (0.0, 0.0, 0.0)
        amplitude = self._default_speed
        if self._digital_pot:
            amplitude = self._digital_pot.get_scaled_value(False)
        if isclose(amplitude, 0.0, abs_tol=0.01):
            self._front_distance = 0.0
            if self._verbose:
                self._display_info('stopped', 0.0, 0.0, 0.0)
            return (0.0, 0.0, 0.0)
        # obstacle scaling only for forward motion
        if amplitude > 0.0:
#           self._front_distance = self._roam_sensor.get_distance()
#           if self._front_distance is None:
#               self._front_distance = self._max_distance
            try:
                self._front_distance = self._roam_sensor.get_distance()
                if self._front_distance is None:
                    self._front_distance = self._max_distance
            except (TypeError, ValueError) as e:
                self._log.warning('sensor error: {}, using max distance'.format(e))
                self._front_distance = self._max_distance
            if self._front_distance >= self._max_distance:
                obstacle_scale = 1.0  # beyond sensor range, full speed
            elif self._front_distance <= self._min_distance:
                obstacle_scale = 0.0  # too close, stop
            else:
                # scale between min_distance and max_distance
                normalised = (self._front_distance - self._min_distance) / (self._max_distance - self._min_distance)
                normalised = np.clip(normalised, 0.0, 1.0)
                # Apply easing function to shape the speed response curve
                obstacle_scale = self._obstacle_easing.apply(normalised)
            amplitude *= obstacle_scale
        else:
            # reverse motion - no obstacle detection
            self._front_distance = 0.0
        # smooth amplitude changes
        max_amplitude_change = 0.10
        delta = amplitude - self._last_amplitude
        if abs(delta) > max_amplitude_change:
            amplitude = self._last_amplitude + (max_amplitude_change if delta > 0 else -max_amplitude_change)
        self._last_amplitude = amplitude
        if self._verbose:
            # log every 10 cycles
            if next(self._counter) % 10 == 0:
                direction = "FWD" if amplitude >= 0.0 else "REV"
                self._log.info('{}: amp: {:.3f}; dist: {:.1f}mm'.format(direction, amplitude, self._front_distance))
        # deadband
        if abs(amplitude) < self._deadband_threshold:
            if self._verbose:
                self._display_info('deadband', 0.0, 0.0, 0.0)
            return (0.0, 0.0, 0.0)
        else:
            vx = 0.0
            vy = amplitude
            omega = 0.0
            if self._verbose:
                self._display_info('active', vx, vy, omega)
            return (vx, vy, omega)

    def _display_info(self, message, vx, vy, omega):
        if self._use_color:
            if vx + vy == 0.0:
                self._log.info(Style.DIM + "{} intent vector: ({:4.2f}, {:4.2f}, {:4.2f}); ".format(
                        message, vx, vy, omega)
                    + Fore.YELLOW + Style.NORMAL + 'distance: {:3.1f}mm'.format(self._front_distance if self._front_distance is not None else 0.0))
            else:
                self._log.info("{} intent vector: ({:4.2f}, {:4.2f}, {:4.2f}); ".format(
                        message, vx, vy, omega)
                    + Fore.YELLOW + Style.NORMAL + 'distance: {:3.1f}mm'.format(self._front_distance if self._front_distance is not None else 0.0))
        else:
            self._log.info("intent vector: ({:.2f},{:.2f},{:.2f})".format(vx, vy, omega))

    def enable(self):
        if self.enabled:
            self._log.debug("already enabled.")
            return
        self._log.info("enabling roam…")
        if not self._roam_sensor.enabled:
            self._roam_sensor.enable()
        AsyncBehaviour.enable(self)
        self._log.info("roam enabled.")

    def disable(self):
        if not self.enabled:
            self._log.debug("already disabled.")
            return
        self._log.info("disabling roam…")
        AsyncBehaviour.disable(self)
        self._log.info('disabled.')

#EOF
