#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2023-05-01
# modified:  2025-10-14

import time
import itertools
import numpy as np
from math import isclose, copysign
from threading import Thread, Event as ThreadEvent
import asyncio
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.cardinal import Cardinal
from core.event import Event
from behave.behaviour import Behaviour
from hardware.roam_sensor import RoamSensor
from hardware.digital_pot import DigitalPotentiometer
from hardware.compass_encoder import CompassEncoder
from hardware.motor_controller import MotorController
from core.orientation import Orientation

class Roam(Behaviour):
    NAME = 'roam'
    '''
    Implements a roaming behaviour. The end result of this Behaviour is to
    provide a forward speed limit based on a fused distance value provided
    by RoamSensor (PWM + VL53L5CX). If no obstacle is perceived, the velocity
    limit is removed.

    Roam uses a vector-based motor control, allowing for heading-based
    driving. Rotational alignment is performed when heading changes,
    using Mecanum wheel kinematics and encoder counts.
    '''

    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Roam.NAME, level)
        Behaviour.__init__(self, self._log, config, message_bus, message_factory, suppressed=True, enabled=False, level=level)
        self.add_event(Event.AVOID)
        _cfg = config['kros'].get('behaviour').get('roam')
        self._loop_delay_ms = _cfg.get('loop_delay_ms', 50)
        self._counter  = itertools.count()
        self._verbose  = _cfg.get('verbose', False)
        self._use_color = True
        self._default_speed = _cfg.get('default_speed', 0.8)
        self._dynamic_speed = _cfg.get('dynamic_speed', True)
        self._dynamic_heading = _cfg.get('dynamic_heading', True)
        self._deadband_threshold = _cfg.get('deadband_threshold', 0.07)
        self._use_world_coordinates = _cfg.get('use_world_coordinates')
        _rs_cfg = config['kros'].get('hardware').get('roam_sensor')
        self._min_distance  = _rs_cfg.get('min_distance')
        self._max_distance  = _rs_cfg.get('max_distance')
        self._polling_rate_hz = _rs_cfg.get('polling_rate_hz', None)
        self._heading_degrees = 0.0
        self._intent_vector = (0.0, 0.0, 0.0)
        self._target_heading_degrees = None
        self._is_rotating = False
        self._rotation_speed = _cfg.get('rotation_speed', 0.5)
        self._rotation_decel_window = _cfg.get('rotation_decel_window', 12)
        self._rotation_direction = 1
        self._rotation_required_degrees = None
        self._rotation_accumulated_degrees = 0.0
        self._rotation_tolerance = _cfg.get('rotation_tolerance', 2.0)
        # Motor access
        _component_registry = Component.get_registry()
        self._digital_pot = None
        if self._dynamic_speed:
            self._digital_pot = _component_registry.get(DigitalPotentiometer.NAME)
        self._compass_encoder = None
        if self._dynamic_heading:
            self._compass_encoder = _component_registry.get(CompassEncoder.NAME)
        self._roam_sensor = _component_registry.get(RoamSensor.NAME)
        if self._roam_sensor is None:
            self._log.info(Fore.WHITE + 'creating Roam sensor…')
            self._roam_sensor = RoamSensor(config, level=Level.INFO)
        else:
            self._log.info(Fore.WHITE + 'using existing Roam sensor.')
        self._motor_controller = _component_registry.get(MotorController.NAME)
        if self._motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        # Get motor objects
        self._motor_pfwd = self._motor_controller.get_motor(Orientation.PFWD)
        self._motor_sfwd = self._motor_controller.get_motor(Orientation.SFWD)
        self._motor_paft = self._motor_controller.get_motor(Orientation.PAFT)
        self._motor_saft = self._motor_controller.get_motor(Orientation.SAFT)
        # geometry from YAML/config & Velocity instance
        velocity = self._motor_pfwd.get_velocity()
        self._steps_per_rotation = velocity.steps_per_rotation
        self._wheel_diameter_mm = velocity._wheel_diameter
        self._wheel_track_mm = config['kros']['geometry']['wheel_track']
        # compute steps_per_degree using geometry and velocity
        wheel_circumference_cm = np.pi * self._wheel_diameter_mm / 10.0
        rotation_circle_cm = np.pi * self._wheel_track_mm / 10.0
        steps_per_degree_theoretical = (rotation_circle_cm / wheel_circumference_cm * self._steps_per_rotation) / 360.0
        # use config override if present, otherwise use calculated value
        # this could be manually tuned to better approximate the actual rotation by the robot
        self._steps_per_degree = _cfg.get('steps_per_degree', steps_per_degree_theoretical)
        self._log.info('steps_per_degree set to: {}'.format(self._steps_per_degree))
        self._register_intent_vector()
        self._poll_delay_sec = self._get_poll_delay_sec()
        self._log.info('ready.')

    def _get_poll_delay_sec(self):
        if self._polling_rate_hz:
            return 1.0 / self._polling_rate_hz
        return self._loop_delay_ms / 1000.0

    def _register_intent_vector(self):
        self._motor_controller.add_intent_vector("roam", lambda: self._intent_vector)
        self._log.info('intent vector lambda registered with motor controller.')

    def _remove_intent_vector(self):
        self._motor_controller.remove_intent_vector("roam")
        self._log.info('intent vector lambda removed from motor controller.')

    def set_heading_degrees(self, degrees):
        degrees = float(degrees) % 360.0
        # Only start rotation if not already rotating to this heading
        if self._is_rotating and self._target_heading_degrees is not None and \
            abs((degrees - self._target_heading_degrees + 180.0) % 360.0 - 180.0) < self._rotation_tolerance:
            self._log.info('Already rotating to {:.2f}°, ignoring duplicate heading request.'.format(degrees))
            return
        # If already at heading (within tolerance), ignore
        if abs((degrees - self._heading_degrees + 180.0) % 360.0 - 180.0) < self._rotation_tolerance:
            self._log.info('Already at {:.2f}°, within tolerance. No rotation needed.'.format(degrees))
            return
        # Otherwise, initiate new rotation
        current = self._heading_degrees % 360.0
        target = degrees
        delta = (target - current + 180.0) % 360.0 - 180.0
        self._rotation_direction = copysign(1, delta)
        self._rotation_required_degrees = abs(delta)
        self._target_heading_degrees = degrees
        self._is_rotating = True
        self._rotation_accumulated_degrees = 0.0
        # Reset encoder start points for this rotation
        self._rotation_start_pfwd = self._motor_pfwd.decoder.steps
        self._rotation_start_sfwd = self._motor_sfwd.decoder.steps
        self._rotation_start_paft = self._motor_paft.decoder.steps
        self._rotation_start_saft = self._motor_saft.decoder.steps
        self._log.info('🐹 heading change requested: rotating {:.2f} degrees {} to {:.2f}.'.format(
            self._rotation_required_degrees,
            "cw" if self._rotation_direction > 0 else "ccw",
            degrees))

    def x_set_heading_degrees(self, degrees):
        degrees = float(degrees) % 360.0
        current = self._heading_degrees % 360.0
        target = degrees
        delta = (target - current + 180.0) % 360.0 - 180.0
        self._rotation_direction = copysign(1, delta)
        self._rotation_required_degrees = abs(delta)
        self._target_heading_degrees = degrees
        self._is_rotating = True
        self._rotation_accumulated_degrees = 0.0
        # Always reset encoder start points on every heading change
        self._rotation_start_pfwd = self._motor_pfwd.decoder.steps
        self._rotation_start_sfwd = self._motor_sfwd.decoder.steps
        self._rotation_start_paft = self._motor_paft.decoder.steps
        self._rotation_start_saft = self._motor_saft.decoder.steps
        self._log.info('🐹 heading change requested: rotating {:.2f} degrees {} to {:.2f}.'.format(
            self._rotation_required_degrees,
            "cw" if self._rotation_direction > 0 else "ccw",
            degrees))

    def set_heading_radians(self, radians):
        self.set_heading_degrees(np.degrees(radians))

    def set_heading_cardinal(self, cardinal):
        self.set_heading_degrees(cardinal.degrees)

    @property
    def name(self):
        return Roam.NAME

    @property
    def is_ballistic(self):
        return False

    def callback(self):
        self._log.info('roam behaviour callback.')

    def execute(self, message):
        print('execute message {}.'.format(message))
        if self.suppressed:
            self._log.info(Style.DIM + 'roam suppressed; message: {}'.format(message.event.label))
        else:
            self._log.info('roam execute; message: {}'.format(message.event.label))
            _payload = message.payload
            _event = _payload.event
            _timestamp = self._message_bus.last_message_timestamp
            if _timestamp is None:
                self._log.info('roam loop execute; no previous messages.')
            else:
                _elapsed_ms = (time.time() - _timestamp.timestamp()) * 1000.0
                self._log.info('roam loop execute; message age: {:7.2f} ms'.format(_elapsed_ms))
            if self.enabled:
                self._log.info('roam enabled, execution on message {}; '.format(message.name) + Fore.YELLOW + ' event: {};'.format(_event.label))
            else:
                self._log.info('roam disabled, execution on message {}; '.format(message.name) + Fore.YELLOW + ' event: {};'.format(_event.label))

    async def _loop_main(self):
        self._log.info("roam loop started with {:.2f}s delay…".format(self._poll_delay_sec))
        try:
            if not self.suppressed:
                self._accelerate()
            while not self._stop_event.is_set():
                if not self.suppressed:
                    await self._poll()
                else:
                    self._log.info(Fore.WHITE + "suppressed…")
                await asyncio.sleep(self._poll_delay_sec)
                if not self.enabled:
                    break
        except asyncio.CancelledError:
            self._log.info("roam loop cancelled.")
        except Exception as e:
            self._log.error('{} encountered in roam loop: {}'.format(type(e), e))
            self.disable()
        finally:
            if not self.suppressed:
                self._decelerate()
            self._log.info("roam loop stopped.")

    def _dynamic_set_default_speed(self):
        if self._digital_pot:
            _speed = self._digital_pot.get_scaled_value(False)
            if isclose(_speed, 0.0, abs_tol=0.08):
                self._digital_pot.set_black()
                self._default_speed = 0.0
            else:
                self._digital_pot.set_rgb(self._digital_pot.value)
                self._default_speed = _speed
                self._log.info(Fore.BLUE + "set default speed: {:4.2f}".format(self._default_speed))

    def _dynamic_set_heading(self):
        if self._compass_encoder:
            self._compass_encoder.update()
            _degrees = self._compass_encoder.get_degrees()
            self.set_heading_degrees(_degrees)

    async def _poll(self):
        try:
            if next(self._counter) % 5 == 0:
                self._dynamic_set_default_speed()
                self._dynamic_set_heading()
            self._update_intent_vector()
        except Exception as e:
            self._log.error("{} thrown while polling: {}".format(type(e), e))
            self.disable()

    def _update_intent_vector(self):
        '''
        Update the intent vector based on heading, speed, obstacle logic, and rotation alignment.
        For rotation, use all four wheel encoder counts.
        This version handles overshoot in both directions robustly.
        '''
        import math
        if self._is_rotating and self._target_heading_degrees is not None:
            # Compute deltas
            delta_pfwd = self._motor_pfwd.decoder.steps - self._rotation_start_pfwd
            delta_sfwd = self._motor_sfwd.decoder.steps - self._rotation_start_sfwd
            delta_paft = self._motor_paft.decoder.steps - self._rotation_start_paft
            delta_saft = self._motor_saft.decoder.steps - self._rotation_start_saft
            # Mecanum rotation estimation (signs per your wiring):
            # For rotation-in-place, port motors CW (positive), starboard CCW (negative)
            rotation_steps = (delta_pfwd + delta_paft - delta_sfwd - delta_saft) / 4.0
            degrees_rotated = abs(rotation_steps / self._steps_per_degree)
            self._rotation_accumulated_degrees = degrees_rotated
            abs_remaining = self._rotation_required_degrees - degrees_rotated
#           self._log.info(
#               'ROTATE: pfwd: {}, sfwd: {}, paft: {}, saft: {}, steps: {}, deg_rotated: {}, deg_needed: {}, deg_remaining: {}'.format(
#                   delta_pfwd, delta_sfwd, delta_paft, delta_saft, rotation_steps, degrees_rotated, self._rotation_required_degrees, abs_remaining
#               )
#           )
            self._log.info(
                'ROTATION DEBUG: degrees_rotated={}, abs_remaining={}, tolerance={}'.format(degrees_rotated, abs_remaining, self._rotation_tolerance)
            )
            # Deceleration and intent vector logic
            if abs_remaining < self._rotation_decel_window and abs_remaining > 0.0:
                rot_speed = self._rotation_speed * (abs_remaining / self._rotation_decel_window)
            else:
                rot_speed = self._rotation_speed
            rot_speed = max(rot_speed, 0.08)
            omega = rot_speed * self._rotation_direction
            # stop logic: handle overshoot in both directions
            # if we are within tolerance or have overshot the required rotation in either direction, just stop
            if math.isclose(degrees_rotated, self._rotation_required_degrees, abs_tol=self._rotation_tolerance) or \
                degrees_rotated >= self._rotation_required_degrees:
                self._heading_degrees = self._target_heading_degrees
                self._is_rotating = False
                self._target_heading_degrees = None
                omega = 0.0
                self._log.info('rotation complete; now facing {:.2f} degrees.'.format(self._heading_degrees))
            self._intent_vector = (0.0, 0.0, omega)
            if self._verbose:
                self._display_info('rotating')
            return
        # normal movement intent vector
        radians = np.deg2rad(self._heading_degrees)
        amplitude = self._default_speed
        if amplitude == 0.0:
            self._intent_vector = (0.0, 0.0, 0.0)
            if self._verbose:
                self._display_info()
            return
        if self._digital_pot:
            amplitude = self._digital_pot.get_scaled_value(False)
        # obstacle logic: scale amplitude if obstacle detected
        front_distance = self._roam_sensor.get_distance()
        min_distance = self._min_distance
        max_distance = self._max_distance
        if front_distance is None or front_distance >= max_distance:
            obstacle_scale = 1.0
        elif front_distance <= min_distance:
            obstacle_scale = 0.0
        else:
            obstacle_scale = (front_distance - min_distance) / (max_distance - min_distance)
            obstacle_scale = np.clip(obstacle_scale, 0.0, 1.0)
        amplitude *= obstacle_scale
        if self._deadband_threshold > 0 and (amplitude < self._deadband_threshold):
            self._intent_vector = (0.0, 0.0, 0.0)
        else:
            vx = np.sin(radians) * amplitude
            vy = np.cos(radians) * amplitude
            omega = 0.0
            self._intent_vector = (vx, vy, omega)
        if self._verbose:
            self._display_info()

    def _display_info(self, message=''):
        if self._use_color:
            self._log.info("{} intent vector: {}({:4.2f},{:4.2f}, {:4.2f}){}".format(
                    message,
                    self.get_highlight_color(self._intent_vector[0]),
                    self._intent_vector[0], self._intent_vector[1], self._intent_vector[2], Style.RESET_ALL
                )
            )
        else:
            self._log.info("intent vector: ({:.2f},{:.2f},{:.2f})".format(
                    self._intent_vector[0], self._intent_vector[1], self._intent_vector[2]
                )
            )

    def get_highlight_color(self, value):
        from colorama import Fore, Style
        if value <= 0.1:
            return Style.DIM
        elif value <= 0.2:
            return Fore.BLUE
        elif value <= 0.3:
            return Fore.GREEN
        elif value <= 0.5:
            return Fore.YELLOW
        elif value <= 0.6:
            return Fore.RED
        else:
            return Fore.MAGENTA

    def _accelerate(self):
        self._log.info("accelerate…")
        if self._motor_controller:
            self._motor_controller.accelerate(self._default_speed, enabled=lambda: not self._stop_event.is_set())

    def _decelerate(self):
        self._log.info("decelerate…")
        if self._motor_controller:
            self._motor_controller.decelerate(0.0, enabled=lambda: not self._stop_event.is_set())

    def enable(self):
        if self.enabled:
            self._log.debug("already enabled.")
            return
        if self._motor_controller:
            self._motor_controller.enable()
            self._register_intent_vector()
        if not self._roam_sensor.enabled:
            self._roam_sensor.enable()
        Component.enable(self)
        self._loop_instance = asyncio.new_event_loop()
        self._stop_event = ThreadEvent()
        asyncio.set_event_loop(self._loop_instance)
        self._task = self._loop_instance.create_task(self._loop_main())
        self._thread = Thread(target=self._loop_instance.run_forever, daemon=True)
        self._thread.start()
        self._log.info("roam enabled.")

    def suppress(self):
        Behaviour.suppress(self)
        self._remove_intent_vector()
        self._log.info("roam suppressed.")

    def release(self):
        Behaviour.release(self)
        self._register_intent_vector()
        self._log.info("roam released.")

    def disable(self):
        if not self.enabled:
            self._log.debug("already disabled.")
            return
        self._log.info("roam disabling…")
        self._stop_event.set()
        time.sleep(0.1)
        if self._loop_instance:
            self._loop_instance.stop()
            self._loop_instance.call_soon_threadsafe(self._shutdown)
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=1.0)
        if self._motor_controller:
            self._motor_controller.disable()
        Component.disable(self)
        self._log.info(Fore.YELLOW + 'disabled.')

    def _shutdown(self):
        self._log.info("shutting down tasks and event loop…")
        self._motor_controller.brake()
        time.sleep(2)
        try:
            tasks = [task for task in asyncio.all_tasks(self._loop_instance) if not task.done()]
            for task in tasks:
                task.cancel()
            self._loop_instance.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))
        except Exception as e:
            self._log.error("{} raised during shutdown: {}".format(type(e), e))
        self._motor_controller.stop()
        self._loop_instance.stop()
        self._loop_instance.close()
        self._log.info(Fore.YELLOW + 'shut down complete.')

    def close(self):
        super().close()
        self._log.info(Fore.YELLOW + 'closed.')

#EOF
