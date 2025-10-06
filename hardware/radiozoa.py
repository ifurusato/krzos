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

import sys
from threading import Thread, Lock, Event
import asyncio
import time
import RPi.GPIO as GPIO
from colorama import init, Fore, Style
init()

# import smbus ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
import pkg_resources
SMBUS='smbus2'
for dist in pkg_resources.working_set:
    #print(dist.project_name, dist.version)
    if dist.project_name == 'smbus':
        break
    if dist.project_name == 'smbus2':
        SMBUS='smbus2'
        break
if SMBUS == 'smbus':
    import smbus
elif SMBUS == 'smbus2':
    import smbus2 as smbus
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

from core.logger import Logger, Level
from core.cardinal import Cardinal
from hardware.i2c_scanner import I2CScanner
from hardware.proximity_sensor import ProximitySensor

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Radiozoa(object):
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
        self._log = Logger('radiozoa', level=level)
        self._level = level
        self._poll_interval = 0.05  # 50ms
        # config ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._config = config
        self._cfg_vl53l0x     = config.get('kros').get('hardware').get('vl53l0x')
        self._cfg_devices     = self._cfg_vl53l0x.get('devices')
        # GPIO ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self._i2c_bus_number = self._cfg_vl53l0x.get('i2c_bus_number')
        if not isinstance(self._i2c_bus_number, int):
            raise ValueError('expected an int for an I2C bus number, not a {}.'.format(type(self._i2c_bus_number)))
        self._i2c_bus = smbus.SMBus()
        self._i2c_bus.open(bus=self._i2c_bus_number)
        self._log.info(Fore.BLUE + 'I2C{} open.'.format(self._i2c_bus_number))
        # general configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._ioe_i2c_address = '0x{:02X}'.format(self._cfg_vl53l0x.get('ioe_i2c_address'))
        self._log.info(Fore.BLUE + 'IO Expander address: {}; type: {}'.format(self._ioe_i2c_address, type(self._ioe_i2c_address)))
        self._default_i2c_address = '0x29' # as string
        self._i2c_scanner = I2CScanner(config=self._config, i2c_bus_number=self._i2c_bus_number, i2c_bus=self._i2c_bus, level=Level.INFO)
        self._has_default_i2c_address = self._i2c_scanner.has_hex_address([self._default_i2c_address])
        self._has_ioe = self._i2c_scanner.has_hex_address([self._ioe_i2c_address])
        self._ioe = None
        if self._has_ioe:
            try:
                import ioexpander as io

                self._log.info(Fore.BLUE + Style.BRIGHT + 'found IO Expander at {} on I2C{}.'.format(self._ioe_i2c_address, self._i2c_bus_number))
                self._ioe = io.IOE(i2c_addr=0x18, smbus_id=self._i2c_bus_number)
            except Exception as e:
                self._log.error('{} raised setting up IO Expander: {}'.format(type(e), e))
                raise
        else:
            self._log.info(Fore.BLUE + Style.DIM + 'did not find IO Expander at {} on I2C{}.'.format(self._ioe_i2c_address, self._i2c_bus_number))
        self._enabled = False
        self._closing = False
        self._closed  = False
        self._callback = None
        # a dict of Cardinal -> ProximitySensor
        self._sensors = []
        self._create_sensors()
        self._sensor_count  = 8 # actually a constant
        # internal state for distance readings, always length 8.
        self._distances = [None for _ in range(self._sensor_count)]
        # asyncio support ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._distances_lock = Lock()
        self._polling_stop_event = Event()
        self._polling_thread = None
        self._polling_loop   = None
        self._polling_task   = None
        self._log.info('ready.')

    def _create_sensors(self):
        '''
        Creates the list of the eight proximity sensors. Each sensor instantiates
        initially as shut down. If enabled, upon connection it changes its I2C
        address and then becomes active.
        '''
        for sensor_id, sensor_config in self._cfg_devices.items():
            _cardinal = Cardinal.from_index(sensor_id)
            self._log.info('creating sensor ' + Fore.GREEN + '{}'.format(_cardinal.label) + Fore.CYAN + '…')
            proximity_sensor = ProximitySensor(
                    cardinal    = _cardinal,
                    i2c_bus     = self._i2c_bus,
                    i2c_address = sensor_config.get('i2c_address'),
                    xshut_pin   = sensor_config.get('xshut'),
                    ioe         = self._ioe,
                    enabled     = sensor_config.get('enabled'),
                    level       = self._level
            )
            self._sensors.append(proximity_sensor)

    def set_callback(self, callback=None):
        '''
        Registers a callback to be called after every polling cycle.
        The callback receives the latest distances list as its only argument.
        '''
        self._callback = callback

    def enable(self):
        '''
        Connects with all enabled sensors. If any device shows up at 0x29 this
        first shuts down all sensors so that they may be brought up individually
        and their respective I2C addresses changed.
        If all sensors are successfully connected, sets the enabled flag True.
        '''
        if self._enabled:
            self._log.warning('already enabled.')
        else:
            if self._has_default_i2c_address:
                self._log.warning('found 0x29, shutting down all sensors.')
                if not self._disconnect_all_sensors():
                    self._log.warning('cannot continue: unable to disconnect all sensors.')
                    self._enabled = False
                    return
            else:
                self._log.info('0x29 not found.')
            self._enabled = self._connect()

    def _start_polling(self):
        '''
        Starts the background polling thread and asyncio event loop.
        '''
        if not self._enabled:
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
        self._log.info('background polling started.')

    async def _poll_sensors(self):
        '''
        Coroutine to poll all sensors at regular intervals and update the internal distances list.
        '''
        while not self._polling_stop_event.is_set():
            distances = [None for _ in range(self._sensor_count)]
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
            with self._distances_lock:
                self._distances = distances
            # execute user callback
            if self._callback:
                try:
                    self._callback(list(distances))
                except Exception as e:
                    self._log.warning(f"Callback raised {type(e).__name__}: {e}")
            await asyncio.sleep(self._poll_interval)

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
                return [self._distances[i] if 0 <= i < self._sensor_count else None for i in indices]

    def close_sensors(self):
        '''
        Close the VL53L0X devices.
        '''
        self._log.info('closing {} sensors…'.format(_remaining))
        for sensor in self._sensors:
            sensor.close()

    def _connect(self):
        '''
        Connects each enabled sensor in sequence.

        Returns:
            bool: True if setup was successful for all enabled sensors, False otherwise.
        '''
        if self._enabled:
            self._log.warning('disabled upon startup: cannot connect.')
            return False
        self._log.info('connecting sensors…')
        try:
            for sensor in self._sensors:
                self._log.info('connecting sensor {} ({}) at I2C address 0x{:02X} with XSHUT pin {}…'.format(
                        sensor.label, sensor.id, sensor.i2c_address, sensor.xshut_pin))
                active = sensor.connect(self._i2c_scanner)
                if not active:
                    self._log.info(Fore.WHITE + 'sensor {} inactive.'.format(sensor.abbrev))
            self._log.info('sensors connected and ready.')
            return True
        except Exception as e:
            self._log.error('{} raised connecting sensors: {}'.format(type(e).__name__, e))
            return False

    def _disconnect_all_sensors(self):
        self._log.info(Fore.WHITE + 'disconnecting sensors…')
        for sensor in self._sensors:
            sensor.shutdown()
        time.sleep(1)
        # now check to see that all sensors are offline
        remaining = []
        for sensor_id, sensor_config in self._cfg_devices.items():
            cardinal = Cardinal.from_index(sensor_id)
            i2c_address = '0x{:02X}'.format(sensor_config.get('i2c_address'))
            self._log.info('checking availability of sensor {} at I2C address {}…'.format(cardinal.label, i2c_address) + Fore.CYAN + '…')
            if self._i2c_scanner.has_hex_address([i2c_address]):
                remaining.append(i2c_address)
                self._log.warning('found sensor {} at address {}.'.format(cardinal.label, i2c_address))
            else:
                self._log.info('did not find sensor {} at address {}.'.format(cardinal.label, i2c_address))
        if len(remaining) == 0:
            return True
        else:
            self._log.warning('cannot continue: sensor {} still online.'.format(remaining))
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
        for sensor in self._sensors:
            self._log.info('opening sensor {}…'.format(sensor.abbrev))
            if sensor.enabled:
                sensor.open()
        time.sleep(0.5)
        self._start_polling()
        # confirm all sensors are available and active
        for sensor in self._sensors:
            self._log.info('checking availability of sensor {}…'.format(sensor.abbrev))
            if sensor.enabled:
                if sensor.active:
                    self._log.info('sensor {} is active.'.format(sensor.abbrev))
                else:
                    raise Exception('sensor {} is inactive.'.format(sensor.abbrev))
            else:
                self._log.info('sensor {} disabled.'.format(sensor.abbrev))
        for sensor in self._sensors:
            self._log.info('start ranging sensor {}…'.format(sensor.abbrev))
            if sensor.enabled:
                sensor.start_ranging()

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

    def close(self):
        self._log.info(Fore.WHITE + Style.BRIGHT + 'close ……………………………………………………………………………………………………………………………………………………………………………………… ')
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
#               self.close_sensors()

                self._log.info(Fore.WHITE + Style.BRIGHT + 'closing smbus ……………………………………………………………………………………………………………………………………………………………………… ')
                self._i2c_bus.close()

                self._closed = True
                self._log.info('closed.')
            except Exception as e:
                self._log.error("{} raised closing radiozoa: {}".format(type(e), e))
            finally:
                if not self._closed:
                    self._log.warning('did not close properly, left in ambiguous state.')
                self._log.info(Fore.WHITE + Style.BRIGHT + 'GPIO cleanup ……………………………………………………………………………………………………………………………………………………………………… ')
                GPIO.cleanup()

#EOF
