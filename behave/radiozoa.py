#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-10
# modified: 2025-10-11

import time
import numpy as np
import itertools
from math import isclose
from threading import Thread, Event as ThreadEvent
import asyncio
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.cardinal import Cardinal
from core.event import Event
from core.orientation import Orientation
from behave.behaviour import Behaviour
from hardware.radiozoa_sensor import RadiozoaSensor
from hardware.digital_pot import DigitalPotentiometer
from hardware.motor_controller import MotorController

class Radiozoa(Behaviour):
    NAME = 'radiozoa'
    RADIOZOA_PFWD_VECTOR_NAME = "__radiozoa_pfwd"
    RADIOZOA_SFWD_VECTOR_NAME = "__radiozoa_sfwd"
    RADIOZOA_PAFT_VECTOR_NAME = "__radiozoa_paft"
    RADIOZOA_SAFT_VECTOR_NAME = "__radiozoa_saft"

    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Radiozoa.NAME, level)
        Behaviour.__init__(self, self._log, config, message_bus, message_factory, suppressed=True, enabled=False, level=level)
        self.add_event(Event.AVOID)
        _cfg = config['kros'].get('behaviour').get('radiozoa')
        self._loop_delay_ms = _cfg.get('loop_delay_ms', 50)
        self._counter  = itertools.count()
        _default_speed = _cfg.get('default_speed', 1.0)
        _dynamic_speed = _cfg.get('dynamic_speed')
        self._verbose   = False
        self._use_color = True
        # per-motor speed (for vectors)
        self._pfwd_vector = (0.0, 0.0)
        self._sfwd_vector = (0.0, 0.0)
        self._paft_vector = (0.0, 0.0)
        self._saft_vector = (0.0, 0.0)
        # directional vectors
        self._pairs = [
            (Cardinal.NORTH, Cardinal.SOUTH),
            (Cardinal.NORTHWEST, Cardinal.SOUTHEAST),
            (Cardinal.WEST, Cardinal.EAST),
            (Cardinal.NORTHEAST, Cardinal.SOUTHWEST),
        ]
        # direction vectors for each axis (unit vectors)
        self._directions = {
            (Cardinal.NORTH, Cardinal.SOUTH): np.array([0, 1]),
            (Cardinal.NORTHWEST, Cardinal.SOUTHEAST): np.array([-1, 1]),
            (Cardinal.WEST, Cardinal.EAST): np.array([-1, 0]),
            (Cardinal.NORTHEAST, Cardinal.SOUTHWEST): np.array([1, 1])
        }
        # registry lookups
        _component_registry = Component.get_registry()
        self._digital_pot = None
        if _dynamic_speed:
            self._default_speed = 0.0
            self._digital_pot = _component_registry.get(DigitalPotentiometer.NAME)
        else:
            self._default_speed = _default_speed
        self._radiozoa_sensor = _component_registry.get(RadiozoaSensor.NAME)
        if self._radiozoa_sensor is None:
            self._log.info(Fore.WHITE + 'creating Radiozoa sensor…')
            self._radiozoa_sensor = RadiozoaSensor(config, level=Level.INFO)
        else:
            self._log.info(Fore.WHITE + 'using existing Radiozoa sensor.')
        self._motor_controller = _component_registry.get(MotorController.NAME)
        if self._motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        self._register_motor_vectors()
        self._log.info('ready.')

    def _register_motor_vectors(self):
        '''
        Register vector functions for each motor, replacing speed multiplier lambdas.
        '''
        self._motor_controller.add_vector(Orientation.PFWD, self.RADIOZOA_PFWD_VECTOR_NAME, lambda: self._pfwd_vector, exclusive=True)
        self._motor_controller.add_vector(Orientation.SFWD, self.RADIOZOA_SFWD_VECTOR_NAME, lambda: self._sfwd_vector, exclusive=True)
        self._motor_controller.add_vector(Orientation.PAFT, self.RADIOZOA_PAFT_VECTOR_NAME, lambda: self._paft_vector, exclusive=True)
        self._motor_controller.add_vector(Orientation.SAFT, self.RADIOZOA_SAFT_VECTOR_NAME, lambda: self._saft_vector, exclusive=True)
        self._log.info('vector functions added to motors.')

    def _remove_motor_vectors(self):
        '''
        Remove Radiozoa vector functions from all motors.
        '''
        self._motor_controller.remove_vector(Orientation.PFWD, self.RADIOZOA_PFWD_VECTOR_NAME)
        self._motor_controller.remove_vector(Orientation.SFWD, self.RADIOZOA_SFWD_VECTOR_NAME)
        self._motor_controller.remove_vector(Orientation.PAFT, self.RADIOZOA_PAFT_VECTOR_NAME)
        self._motor_controller.remove_vector(Orientation.SAFT, self.RADIOZOA_SAFT_VECTOR_NAME)
        self._log.info('vector functions removed from motors.')

    @property
    def name(self):
        return Radiozoa.NAME

    @property
    def is_ballistic(self):
        return False

    def callback(self):
        self._log.info('radiozoa behaviour callback.')

    def execute(self, message):
        print('execute message {}.'.format(message))
        if self.suppressed:
            self._log.info(Style.DIM + 'radiozoa suppressed; message: {}'.format(message.event.label))
        else:
            self._log.info('radiozoa execute; message: {}'.format(message.event.label))
            _payload = message.payload
            _event = _payload.event
            _timestamp = self._message_bus.last_message_timestamp
            if _timestamp is None:
                self._log.info('radiozoa loop execute; no previous messages.')
            else:
                _elapsed_ms = (time.time() - _timestamp.timestamp()) * 1000.0
                self._log.info('radiozoa loop execute; message age: {:7.2f} ms'.format(_elapsed_ms))
            if self.enabled:
                self._log.info('radiozoa enabled, execution on message {}; '.format(message.name) + Fore.YELLOW + ' event: {};'.format(_event.label))
            else:
                self._log.info('radiozoa disabled, execution on message {}; '.format(message.name) + Fore.YELLOW + ' event: {};'.format(_event.label))

    async def _loop_main(self):
        self._log.info("radiozoa loop started with {}ms delay…".format(self._loop_delay_ms))
        try:
            if not self.suppressed:
                self._accelerate()
            while not self._stop_event.is_set():
                if not self.suppressed:
                    await self._poll()
                else:
                    self._log.info(Fore.WHITE + "suppressed…")
                await asyncio.sleep(self._loop_delay_ms / 1000)
                if not self.enabled:
                    break
        except asyncio.CancelledError:
            self._log.info("radiozoa loop cancelled.")
        except Exception as e:
            self._log.error('{} encountered in radiozoa loop: {}'.format(type(e), e))
            self.disable()
        finally:
            if not self.suppressed:
                self._decelerate()
            self._log.info("radiozoa loop stopped.")

    def _dynamic_set_default_speed(self):
        _speed = self._digital_pot.get_scaled_value(False) # values 0.0-1.0
        if isclose(_speed, 0.0, abs_tol=0.08):
            self._digital_pot.set_black() # only on digital pot
            self._default_speed = 0.0
            self._log.info(Fore.BLACK + "default speed: stopped")
        else:
            self._digital_pot.set_rgb(self._digital_pot.value)
            self._default_speed = _speed
            self._log.info(Fore.BLUE + "set default speed: {:4.2f}".format(self._default_speed))

    async def _poll(self):
        try:
#           self._log.debug("polling…")
            if next(self._counter) % 5 == 0:
                self._dynamic_set_default_speed()
            distances = self._radiozoa_sensor.get_distances()
            if not distances or all(d is None or d > RadiozoaSensor.FAR_THRESHOLD for d in distances):
                # stop when sensors are unavailable or out of range.
                self._pfwd_speed = 0.0
                self._sfwd_speed = 0.0
                self._paft_speed = 0.0
                self._saft_speed = 0.0
            else:
                self._update_motor_speeds(distances)
        except Exception as e:
            self._log.error("{} thrown while polling: {}".format(type(e), e))
            self.disable()

    def get_highlight_color(self, value):
        """Return colorama color/style for multiplier legend."""
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
        else:  # 0.61 - 0.8
            return Fore.MAGENTA

    def _update_motor_speeds(self, distances):
        """
        Center robot in space using four opposing sensor pairs as force vectors.
        Robot does not move if both sensors in a pair are far (open space).
        The amplitude of each pair's contribution is proportional to the imbalance between the two sensors.
        The resulting vector is mapped to Mecanum wheels.
        """
        far_threshold = RadiozoaSensor.FAR_THRESHOLD * 0.95 # use a cutoff just below max range
        force_vec = np.zeros(2)
        pair_active = False
        for c1, c2 in self._pairs:
            d1 = self._radiozoa_sensor.get_sensor_by_cardinal(c1).get_distance()
            d2 = self._radiozoa_sensor.get_sensor_by_cardinal(c2).get_distance()
            d1 = d1 if d1 is not None and d1 > 0 else RadiozoaSensor.FAR_THRESHOLD
            d2 = d2 if d2 is not None and d2 > 0 else RadiozoaSensor.FAR_THRESHOLD

            # TEMP
    #       d1 = RadiozoaSensor.FAR_THRESHOLD
    #       d2 = RadiozoaSensor.FAR_THRESHOLD

            # if both sensors are far, ignore this pair (no imbalance to correct)
            if d1 >= far_threshold and d2 >= far_threshold:
                continue
            # imbalance sets amplitude; move toward centering
            diff = d1 - d2
            vec = self._directions[(c1, c2)] * diff
            force_vec += vec
            pair_active = True
            # TEMP
    #       self._log.info('PAIR: {}-{} d1={} d2={} diff={} vec={}'.format(c1.label, c2.label, d1, d2, diff, vec))

        # if no pair contributed, robot is centered or in open space
        if not pair_active or np.linalg.norm(force_vec) < 1.0:
            self._pfwd_vector = self._sfwd_vector = self._paft_vector = self._saft_vector = (0.0, 0.0)
            return
        # normalize, scale, map to wheel vectors
        max_abs = np.max(np.abs(force_vec)) if np.max(np.abs(force_vec)) > 1.0 else 1.0
        vx, vy = force_vec / max_abs
        self._pfwd_vector = (vx, vy)
        self._sfwd_vector = (vx, vy)
        self._paft_vector = (vx, vy)
        self._saft_vector = (vx, vy)
        # display
        if self._verbose:
            self._display_info()
        else:
    #       self._log.debug(Fore.BLACK + "…")
    #       print(Fore.BLACK + "…" + Style.RESET_ALL)
            pass

    def x_update_motor_speeds(self, distances):
        """
        NOTE: OLDER VERSION
        Center robot in space using four opposing sensor pairs as force vectors.
        Robot does not move if both sensors in a pair are far (open space).
        The amplitude of each pair's contribution is proportional to the imbalance between the two sensors.
        The resulting vector is mapped to Mecanum wheels.
        """
        far_threshold = RadiozoaSensor.FAR_THRESHOLD * 0.95 # use a cutoff just below max range
        force_vec = np.zeros(2)
        pair_active = False
        for c1, c2 in self._pairs:
            d1 = self._radiozoa_sensor.get_sensor_by_cardinal(c1).get_distance()
            d2 = self._radiozoa_sensor.get_sensor_by_cardinal(c2).get_distance()
            d1 = d1 if d1 is not None and d1 > 0 else RadiozoaSensor.FAR_THRESHOLD
            d2 = d2 if d2 is not None and d2 > 0 else RadiozoaSensor.FAR_THRESHOLD

            # TEMP
#           d1 = RadiozoaSensor.FAR_THRESHOLD
#           d2 = RadiozoaSensor.FAR_THRESHOLD

            # if both sensors are far, ignore this pair (no imbalance to correct)
            if d1 >= far_threshold and d2 >= far_threshold:
                continue
            # imbalance sets amplitude; move toward centering
            diff = d1 - d2
            vec = self._directions[(c1, c2)] * diff
            force_vec += vec
            pair_active = True

            # TEMP
#           self._log.info('PAIR: {}-{} d1={} d2={} diff={} vec={}'.format(c1.label, c2.label, d1, d2, diff, vec))

        # if no pair contributed, robot is centered or in open space
        if not pair_active or np.linalg.norm(force_vec) < 1.0:
            self._pfwd_speed = self._sfwd_speed = self._paft_speed = self._saft_speed = 0.0
            return
        # normalize, scale, map to wheel speeds
        max_abs = np.max(np.abs(force_vec)) if np.max(np.abs(force_vec)) > 1.0 else 1.0
        vx, vy = force_vec / max_abs
        pfwd = vy + vx
        sfwd = vy - vx
        paft = vy - vx
        saft = vy + vx
        max_motor = max(abs(pfwd), abs(sfwd), abs(paft), abs(saft), 1.0)
        self._pfwd_speed = float(np.clip(pfwd * self._default_speed / max_motor, -self._default_speed, self._default_speed))
        self._sfwd_speed = float(np.clip(sfwd * self._default_speed / max_motor, -self._default_speed, self._default_speed))
        self._paft_speed = float(np.clip(paft * self._default_speed / max_motor, -self._default_speed, self._default_speed))
        self._saft_speed = float(np.clip(saft * self._default_speed / max_motor, -self._default_speed, self._default_speed))
        # display
        if self._verbose:
            self._display_info()
        else:
#           self._log.debug(Fore.BLACK + "…")
#           print(Fore.BLACK + "…" + Style.RESET_ALL)
            pass

    def _display_info(self, message=''):
        if self._use_color:
            self._log.info("{} vectors: pfwd={}({:4.2f},{:4.2f}){} sfwd={}({:4.2f},{:4.2f}){} paft={}({:4.2f},{:4.2f}){} saft={}({:4.2f},{:4.2f}){}".format(
                    message,
                    self.get_highlight_color(self._pfwd_vector[0]),
                    self._pfwd_vector[0], self._pfwd_vector[1], Style.RESET_ALL,
                    self.get_highlight_color(self._sfwd_vector[0]),
                    self._sfwd_vector[0], self._sfwd_vector[1], Style.RESET_ALL,
                    self.get_highlight_color(self._paft_vector[0]),
                    self._paft_vector[0], self._paft_vector[1], Style.RESET_ALL,
                    self.get_highlight_color(self._saft_vector[0]),
                    self._saft_vector[0], self._saft_vector[1], Style.RESET_ALL
                )
            )
        else:
            self._log.info("vectors: pfwd=({:.2f},{:.2f}) sfwd=({:.2f},{:.2f}) paft=({:.2f},{:.2f}) saft=({:.2f},{:.2f})".format(
                    self._pfwd_vector[0], self._pfwd_vector[1],
                    self._sfwd_vector[0], self._sfwd_vector[1],
                    self._paft_vector[0], self._paft_vector[1],
                    self._saft_vector[0], self._saft_vector[1]
                )
            )

    def x_display_info(self, message=''):
        if self._use_color:
            self._log.info("{} speeds: pfwd={}{:4.2f}{} sfwd={}{:4.2f}{} paft={}{:4.2f}{} saft={}{:4.2f}{}".format(
                    message,
                    self.get_highlight_color(self._pfwd_speed),
                    self._pfwd_speed,
                    Style.RESET_ALL,
                    self.get_highlight_color(self._sfwd_speed),
                    self._sfwd_speed,
                    Style.RESET_ALL,
                    self.get_highlight_color(self._paft_speed),
                    self._paft_speed,
                    Style.RESET_ALL,
                    self.get_highlight_color(self._saft_speed),
                    self._saft_speed,
                    Style.RESET_ALL
                )
            )
        else:
            self._log.info("speeds: pfwd={:.2f} sfwd={:.2f} paft={:.2f} saft={:.2f}".format(
                    self._pfwd_speed, self._sfwd_speed, self._paft_speed, self._saft_speed
                )
            )
 
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
            self._register_motor_vectors()
        Component.enable(self)
        self._loop_instance = asyncio.new_event_loop()
        self._stop_event = ThreadEvent()
        asyncio.set_event_loop(self._loop_instance)
        self._task = self._loop_instance.create_task(self._loop_main())
        self._thread = Thread(target=self._loop_instance.run_forever, daemon=True)
        self._thread.start()
        self._log.info("radiozoa enabled.")

    def suppress(self):
        Behaviour.suppress(self)
        self._remove_motor_vectors()
        self._log.info("radiozoa suppressed.")

    def release(self):
        '''
        Releases suppression of the behaviour, re-registering vector functions.
        '''
        Behaviour.release(self)
        self._register_motor_vectors()
        self._log.info("radiozoa released.")

    def disable(self):
        if not self.enabled:
            self._log.debug("already disabled.")
            return
        self._log.info("radiozoa disabling…")
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
        # if brake hasn't finished we stop anyway
        self._motor_controller.stop()
        self._loop_instance.stop()
        self._loop_instance.close()
        self._log.info(Fore.YELLOW + 'shut down complete.')

    def close(self):
        super().close()
        self._log.info(Fore.YELLOW + 'closed.')

#EOF
