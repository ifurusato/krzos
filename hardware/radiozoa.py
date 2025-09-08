#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-09-08
# modified: 2025-09-08
#

import threading
import asyncio
import time
import RPi.GPIO as GPIO
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from hardware.i2c_scanner import I2CScanner
from hardware.proximity_sensor import ProximitySensor

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Radiozoa(object):
    SENSOR_COUNT = 8
    POLL_INTERVAL_S = 0.05  # 50ms
    '''
    Manages an array of VL53L0X proximity sensors as implemented on the Radiozoa board.
    '''
    def __init__(self, config, level=Level.INFO):
        '''
        Initializes the sensor array, setting up sensors.

        Args:
            config (dict): The configuration dictionary for the sensors.
            level (Level): The logging level.
        '''
        self._log = Logger('proximity', level=level)
        self._config = config
        self._cfg_vl53l0x = config.get('kros').get('hardware').get('vl53l0x')
        self._i2c_bus_number = self._cfg_vl53l0x.get('i2c_bus_number')
        self._cfg_devices = self._cfg_vl53l0x.get('devices')
        self._active_sensors = []
        self._enabled = False
        self._closing = False
        self._closed  = False
        self._disable_delay_s = 0.5
        self._callback: Optional[Callable[[List[Optional[int]]], None]] = None
        # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        # internal state for distance readings, always length 8.
        self._distances = [None for _ in range(self.SENSOR_COUNT)]
        self._distances_lock = threading.Lock()
        # asyncio polling members
        self._polling_thread = None
        self._polling_loop = None
        self._polling_task = None
        self._polling_stop_event = threading.Event()
        self._log.info('ready.')

    def set_callback(self, callback=None):
        '''
        Registers a callback to be called after every polling cycle.
        The callback receives the latest distances list as its only argument.
        '''
        self._callback = callback

    def enable(self):
        if self._enabled:
            self._log.warning('already enabled.')
        else:
            self.disable_all_sensors()
            self._enabled = self._setup()

    def _start_polling(self):
        '''
        Starts the background polling thread and asyncio event loop.
        '''
        self._polling_stop_event.clear()
        def run_loop():
            self._polling_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._polling_loop)
            self._polling_task = self._polling_loop.create_task(self._poll_sensors())
            try:
                self._polling_loop.run_forever()
            finally:
                self._polling_loop.close()
        self._polling_thread = threading.Thread(target=run_loop, daemon=True)
        self._polling_thread.start()
        self._log.info('background polling started.')

    async def _poll_sensors(self):
        '''
        Coroutine to poll all sensors at regular intervals and update the internal distances list.
        '''
        while not self._polling_stop_event.is_set():
            distances = [None for _ in range(self.SENSOR_COUNT)]
            # Map sensors to their config order
            sorted_keys = sorted(self._cfg_devices.keys())
            sensor_map = {sensor.id: sensor for sensor in self._active_sensors}
            for idx, sensor_id in enumerate(sorted_keys):
                sensor_config = self._cfg_devices[sensor_id]
                enabled = sensor_config.get('enabled', False)
                sensor = sensor_map.get(sensor_id)
                if enabled and sensor is not None and sensor.enabled:
                    try:
                        distance = sensor.get_distance()
                        distances[idx] = distance
                    except Exception as e:
                        self._log.warning("{} reading sensor {}: {}".format(type(e), sensor.local, e))
                        distances[idx] = None
                else:
                    distances[idx] = None
            with self._distances_lock:
                self._distances = distances
            # execute user callback
            if self._callback:
                try:
                    self._callback(list(distances))
                except Exception as e:
                    self._log.warning(f"Callback raised {type(e).__name__}: {e}")
            await asyncio.sleep(self.POLL_INTERVAL_S)

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
                # Return all eight distances
                return list(self._distances)
            else:
                # Return only those specified by Cardinal enum values, in argument order
                indices = [c.value[0] for c in cardinals]
                return [self._distances[i] if 0 <= i < self.SENSOR_COUNT else None for i in indices]

    def disable_all_sensors(self):
        '''
        Disable all VL53L0X devices.
        '''
        _i2c_addresses = {}
        for sensor in self._active_sensors:
            self._log.debug('closing sensor {}…'.format(sensor.label))
            sensor.close()
            _i2c_addresses[sensor.i2c_address] = sensor.xshut_pin
        _='''
        self._log.info('Disabling all VL53L0X sensors (setting XSHUT pins LOW)...')
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        # iterate through all devices in config, regardless of 'enabled'
        for sensor_config in self._cfg_devices.values():
            label = sensor_config.get('label')
            xshut_pin = sensor_config.get('xshut')
            i2c_address = sensor_config.get('i2c_address')
            if xshut_pin is None:
                raise Exception('no XSHUT pin configured for sensor {}.'.format(label))
            GPIO.setup(xshut_pin, GPIO.OUT)
            GPIO.output(xshut_pin, GPIO.LOW)
            self._log.info(Style.DIM + "disable sensor {} via XSHUT pin {}".format(label, xshut_pin))
        time.sleep(1.0) # wait to ensure sensors shut down
        '''
        self._confirm_clean_bus(_i2c_addresses)

    def _confirm_clean_bus(self, i2c_addresses):
        _i2c_scanner = I2CScanner(self._config, bus_number=self._i2c_bus_number, level=Level.INFO)
        for i2c_address, xshut_pin in i2c_addresses.items():
            _address_str = '0x{:02X}'.format(i2c_address)
            self._log.info('checking device at {}…'.format(_address_str))
            if _i2c_scanner.has_hex_address([_address_str]):
                self._log.warning('sensor at {} still online.'.format(_address_str))
                self._reset_xshut_pin(xshut_pin, i2c_address)
            time.sleep(0.5)
            if _i2c_scanner.has_hex_address([_address_str]):
                self._log.warning('device remains at {}.'.format(_address_str))
            else:
                self._log.info('device clean at {}.'.format(_address_str))
        self._log.info('all VL53L0X sensors disabled.')

    def _reset_xshut_pin(self, xshut_pin, i2c_address):
        try:
            self._log.info(Fore.WHITE + Style.BRIGHT + 'resetting xshut pin {} for device at 0x{:02X}.'.format(xshut_pin, i2c_address))
            GPIO.setup(xshut_pin, GPIO.OUT)
            GPIO.output(xshut_pin, GPIO.LOW)
            time.sleep(0.5)
        except Exception as e:
            self._log.error('{} raised resetting xshut pin: {}'.format(type(e).__name__, e))

    def _setup(self):
        '''
        Creates and enables each sensor sequentially using the ProximitySensor class.

        Returns:
            bool: True if setup was successful for all enabled sensors, False otherwise.
        '''
        self._log.info(Fore.WHITE + 'setting up sensors…')
        try:
            for _sensor_id, sensor_config in self._cfg_devices.items():
                _label       = sensor_config.get('label')
                _i2c_address = sensor_config.get('i2c_address')
                _xshut       = sensor_config.get('xshut')
                _enabled     = sensor_config.get('enabled')
                # the sensor instantiates as shut down, changes its I2C address and then becomes available if enabled
                sensor = ProximitySensor(
                        i2c_bus_number=self._i2c_bus_number,
                        sensor_id=_sensor_id,
                        label=_label,
                        i2c_address=_i2c_address,
                        xshut_pin=_xshut,
                        enabled=_enabled
                )
                if _enabled:
                    self._log.info('created sensor {} ({}) at I2C address 0x{:02X} with XSHUT pin {}…'.format(
                            _label, _sensor_id, _i2c_address, _xshut))
                    self._active_sensors.append(sensor)
                else:
                    self._log.info(Style.DIM + 'created disabled sensor {} ({}) at I2C address 0x{:02X} with XSHUT pin {}…'.format(
                            _label, _sensor_id, _i2c_address, _xshut))
            self._log.info('sensors ready.')
#           self.print_configuration()
            if self._enabled:
                self._start_polling()
            return True
        except Exception as e:
            self._log.error('{} raised setting up sensors: {}'.format(type(e).__name__, e))
            self.stop_ranging()
            return False

    def print_configuration(self):
        for sensor_id, sensor_config in self._cfg_devices.items():
            label = sensor_config.get('label')
            if sensor_config.get('enabled'):
                self._log.info("sensor {} ({}) enabled.".format(label, sensor_id))
            else:
                self._log.info("sensor {} ({}) disabled.".format(label, sensor_id))

    @property
    def enabled(self):
        '''
        Returns True if the sensor array was successfully enabled.
        '''
        return self._enabled

    def start_ranging(self):
        '''
        Starts the ranging process for all active sensors.
        '''
        self._log.info(Fore.GREEN + 'start ranging…')
        if self._active_sensors:
            for sensor in self._active_sensors:
                self._log.debug('opening sensor {}…'.format(sensor.id))
                sensor.open()
            for sensor in self._active_sensors:
                self._log.debug('start ranging sensor {}…'.format(sensor.id))
                sensor.start_ranging()

    def _stop_ranging(self):
        '''
        Stops ranging and shuts down all active sensors.
        '''
        self._log.info('stop ranging…')
        for sensor in self._active_sensors:
            sensor.stop()

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

    def close(self):
        if self._closing:
            self._log.warning('already closing…')
        elif self._closed:
            self._log.warning('already closed.')
        else:
            self._closing = True
            self._log.info('closing…')
            try:
                self._stop_ranging()
                self._stop_polling()
                self.disable_all_sensors()
                GPIO.cleanup()
                self._closed = True
                self._log.info('closed.')
            except Exception as e:
                self._log.warning("{} reading sensor {}: {}".format(type(e), sensor.local, e))
            finally:
                if not self._closed:
                    self._log.warning('did not close properly, left in ambiguous state.')

#EOF
