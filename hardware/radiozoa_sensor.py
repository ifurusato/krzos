#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-09-08
# modified: 2025-10-09
#

import sys
from datetime import datetime as dt
import asyncio
import time
from threading import Thread, Lock, Event
import RPi.GPIO as GPIO
from colorama import init, Fore, Style
init()

import ioexpander as io

import pkg_resources
SMBUS='smbus2'
for dist in pkg_resources.working_set:
    if dist.project_name == 'smbus':
        break
    if dist.project_name == 'smbus2':
        SMBUS='smbus2'
        break
if SMBUS == 'smbus':
    import smbus
elif SMBUS == 'smbus2':
    import smbus2 as smbus

from core.logger import Logger, Level
from core.component import Component, MissingComponentError
from core.cardinal import Cardinal
from hardware.i2c_scanner import I2CScanner
from hardware.proximity_sensor import ProximitySensor

class RadiozoaSensor(Component):
    NAME = 'radiozoa-sensor'
    # thresholds in millimeters
    CLOSE_THRESHOLD = 100
    NEAR_THRESHOLD  = 200
    MID_THRESHOLD   = 600
    FAR_THRESHOLD   = 1000
    '''
    Manages an array of VL53L0X proximity sensors as implemented on the Radiozoa
    sensor board. Assumes all sensors are configured and enabled. If any sensor
    is unavailable, raises a MissingComponentError.
    '''
    def __init__(self, config, level=Level.INFO):
        '''
        Initializes the sensor array, setting up sensors.

        Args:
            config (dict): The configuration dictionary for the sensors.
            level (Level): The logging level.
        '''
        self._log = Logger(RadiozoaSensor.NAME, level=level)
        if config is None:
            raise ValueError('no configuration provided.')
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._level = level
        self._poll_interval = 0.01  # 50ms
        # config ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._config = config
        _cfg_radiozoa        = config.get('kros').get('hardware').get('radiozoa')
        self._cfg_devices    = _cfg_radiozoa.get('devices')
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self._i2c_bus_number = _cfg_radiozoa.get('i2c_bus_number')
        if not isinstance(self._i2c_bus_number, int):
            raise ValueError('expected an int for an I2C bus number, not a {}.'.format(type(self._i2c_bus_number)))
        self._i2c_bus = smbus.SMBus()
        self._i2c_bus.open(bus=self._i2c_bus_number)
        self._log.debug('I2C{} open.'.format(self._i2c_bus_number))
        self._sensor_count   = 8
        self._i2c_scanner = I2CScanner(config=self._config, i2c_bus_number=self._i2c_bus_number, i2c_bus=self._i2c_bus, level=Level.INFO)
        self._sensors = []
        _has_default_i2c_address = self._i2c_scanner.has_hex_address(['0x29']) # default I2C address
        # set up IO Expander ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._ioe = None
        _ioe_i2c_address = '0x{:02X}'.format(_cfg_radiozoa.get('ioe_i2c_address'))
        if self._i2c_scanner.has_hex_address([_ioe_i2c_address]):
            try:
                self._ioe = io.IOE(i2c_addr=0x18, smbus_id=self._i2c_bus_number)
                self._log.info('found IO Expander at {} on I2C{}.'.format(_ioe_i2c_address, self._i2c_bus_number))
            except Exception as e:
                self._log.error('{} raised setting up IO Expander: {}'.format(type(e), e))
                raise
        else:
            raise MissingComponentError('did not find IO Expander at {} on I2C{}.'.format(_ioe_i2c_address, self._i2c_bus_number))
        # create all sensors, raise exception if any are missing
        self._create_sensors()
        missing = []
        for sensor in self._sensors:
            if not self._i2c_scanner.has_hex_address(["0x{:02X}".format(sensor.i2c_address)]):
                missing.append(sensor.label)
        if missing:
            raise MissingComponentError("missing required Radiozoa sensors: {}".format(", ".join(missing)))
        self._distances      = [None for _ in range(self._sensor_count)]
        self._distances_lock = Lock()
        self._polling_stop_event = Event()
        self._polling_thread = None
        self._polling_loop   = None
        self._polling_task   = None
        self._callback       = None
        self._log.info('ready.')

    def set_visualisation(self, enable):
        if enable:
            self.set_callback(self.print_distances)
        else:
            self.set_callback(None)

    def set_callback(self, callback=None):
        '''
        Registers a callback to be called after every polling cycle.
        The callback receives the latest distances list and the 
        elapsed time in milliseconds as its two arguments.
        '''
        self._callback = callback

    def start_ranging(self):
        '''
        Starts the ranging process for all active sensors.
        Raises MissingComponentError if any sensor is not active.
        '''
        self._log.info(Fore.GREEN + 'start ranging…')
        for sensor in self._sensors:
            self._log.debug('opening sensor {}…'.format(sensor.abbrev))
            if sensor.enabled:
                sensor.open()
        time.sleep(0.5)
        self._start_polling()
        # confirm all sensors are available and active
        for sensor in self._sensors:
            self._log.debug('checking availability of sensor {}…'.format(sensor.abbrev))
            if sensor.enabled and not sensor.active:
                raise MissingComponentError('Sensor {} is inactive.'.format(sensor.abbrev))
        for sensor in self._sensors:
            self._log.debug('start ranging sensor {}…'.format(sensor.abbrev))
            if sensor.enabled:
                sensor.start_ranging()

    def _create_sensors(self):
        '''
        Creates the list of the eight proximity sensors.
        '''
        for sensor_id, sensor_config in self._cfg_devices.items():
            _cardinal = Cardinal.from_index(sensor_id)
            self._log.info('creating sensor ' + Fore.GREEN + '{}'.format(_cardinal.label) + Fore.CYAN + '…')
            proximity_sensor = ProximitySensor(
                    cardinal    = _cardinal,
                    i2c_bus     = self._i2c_bus,
                    i2c_address = sensor_config.get('i2c_address'),
                    ioe         = self._ioe,
                    enabled     = sensor_config.get('enabled'),
                    level       = self._level
            )
            self._sensors.append(proximity_sensor)

    def _start_polling(self):
        '''
        Starts the background polling thread and asyncio event loop.
        '''
        if not self.enabled:
            self._log.warning('disabled upon startup: cannot start polling.')
            return
        self._polling_stop_event.clear()
        def run_loop():
            self._polling_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._polling_loop)
            self._polling_task = self._polling_loop.create_task(self._poll_sensors())
            try:
                self._polling_loop.run_forever()
            finally:
                self._polling_loop.close()
        self._polling_thread = Thread(target=run_loop, daemon=True)
        self._polling_thread.start()
        self._log.info(Fore.MAGENTA + 'background polling started.')

    async def _poll_sensors(self):
        '''
        Coroutine to poll all sensors at regular intervals and update the internal distances list.
        '''
        self._log.info('start polling sensors…')
        while self.enabled and not self._polling_stop_event.is_set():
#           self._log.debug('polling sensors…; stop event: {}'.format(self._polling_stop_event.is_set()))
            distances = [None for _ in range(self._sensor_count)]
            _start_time = dt.now()
            for sensor in self._sensors:
                if sensor.enabled:
                    try:
                        distance = sensor.get_distance()
                        distances[sensor.id] = distance
                    except Exception as e:
                        self._log.warning("{} reading sensor {}: {}".format(type(e), sensor.abbrev, e))
                        distances[sensor.id] = None
                else:
                    distances[sensor.id] = None
            _elapsed_ms = round((dt.now() - _start_time).total_seconds() * 1000.0)
#           self._log.info('sensor polling complete: {}ms elapsed.'.format(_elapsed_ms))
            with self._distances_lock:
                self._distances = distances
            # execute user callback
            if self._callback:
                try:
                    self._callback(list(distances), _elapsed_ms)
                except Exception as e:
                    self._log.warning("Callback raised {}: {}".format(type(e).__name__, e))
            await asyncio.sleep(self._poll_interval)
        self._log.info(Fore.WHITE + 'polling sensors stopped.')

    def get_distances(self, cardinals=None):
        '''
        Returns the latest distance readings for the specified cardinal directions, or all eight
        if no argument.

        Args:
            cardinals (list of Cardinal, optional): List of Cardinal enum values to select readings.
                                                   If None, returns all eight sensor readings.
        Returns:
            list: List of distances (or None for disabled/unavailable sensors), order matches argument.
        '''
        with self._distances_lock:
            if cardinals is None:
                # return all eight distances
                return list(self._distances)
            else:
                # Return only those specified by Cardinal enum values, in argument order
                indices = [c.value[0] for c in cardinals]
                return [self._distances[i] if 0 <= i < self._sensor_count else None for i in indices]

    def print_configuration(self):
        for sensor_id, sensor_config in self._cfg_devices.items():
            label = sensor_config.get('label')
            if sensor_config.get('enabled'):
                self._log.info("sensor {} ({}) enabled.".format(label, sensor_id))
            else:
                self._log.info("sensor {} ({}) disabled.".format(label, sensor_id))

    def enable(self):
        all_connected = True
        for sensor in self._sensors:
            self._log.debug('connecting to sensor {}…'.format(sensor.abbrev))
            if sensor.enabled:
                if not sensor.connect():
                    all_connected = False
        if all_connected:
            super().enable()
            self._log.info('all sensors connected.')
        else:
            self._log.warning('unable to connect to all sensors.')

    def _stop_ranging(self):
        '''
        Stops ranging and shuts down all active sensors.
        '''
        self._log.info('stop ranging…')
        for sensor in self._sensors:
            sensor.stop_ranging()

    def _stop_polling(self):
        '''
        Stops the polling thread and asyncio loop.
        '''
        self._polling_stop_event.set()
        if self._polling_loop:
            self._polling_loop.call_soon_threadsafe(self._polling_loop.stop)
        if self._polling_thread:
            self._polling_thread.join(timeout=2)
        self._polling_thread = None
        self._polling_loop   = None
        self._polling_task   = None
        self._log.info('background polling stopped.')

    # display ...........
    
    def _color_for_distance(self, dist):
        '''
        out of range: >= FAR_THRESH or None/<=0
        '''
        match dist:
            case d if d is None or d <= 0:
                return Fore.BLACK + Style.DIM
            case d if d < self.CLOSE_THRESHOLD:
                return Fore.MAGENTA + Style.BRIGHT
            case d if d < self.NEAR_THRESHOLD:
                return Fore.RED
            case d if d < self.MID_THRESHOLD:
                return Fore.YELLOW
            case d if d < self.FAR_THRESHOLD:
                return Fore.GREEN
            case _:
                return Fore.BLACK
                
    def print_distances(self, distances, elapsed_ms):
        '''
        Callback to print distance values from all eight sensors, with colorized output
        based on thresholds for "close", "near", "mid", "far" and "out of range".
        '''
        msg = "["
        for i, dist in enumerate(distances):
            color = self._color_for_distance(dist)
            if dist is not None and dist > 0:
                msg += f" {color}{dist:>4}mm{Style.RESET_ALL}"
            else:
                msg += f" {color}None{Style.RESET_ALL}"
            if i < len(distances)-1:
                msg += ","
        msg += Style.DIM + " ] {}ms elapsed.".format(elapsed_ms) + Style.RESET_ALL
        print(msg)

    # ...
    def disable(self):
        self._log.info('disabling…')
        super().disable()

    def close(self):
        self._log.info('closing…')
        self._polling_stop_event.set()
        if self.closed:
            self._log.warning('already closed.')
        else:
            self._log.info('closing…')
            try:
                super().close()
                self._stop_ranging()
                self._stop_polling()
                self._log.info('closed.')
            except Exception as e:
                self._log.error("{} raised closing radiozoa: {}".format(type(e), e))
            finally:
                if not self.closed:
                    self._log.warning('did not close properly, left in ambiguous state.')
                if not self._ioe:
                    GPIO.cleanup()
        self._log.info('closed.')

#EOF
