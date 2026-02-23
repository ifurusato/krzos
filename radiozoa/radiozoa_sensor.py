#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-01-29
# modified: 2026-02-17

import time
from machine import I2C
from colorama import Fore, Style

from logger import Logger, Level
from cardinal import Cardinal
from device import Device
from vl53l0x import VL53L0X
from vl53l1x import VL53L1X

class RadiozoaSensor:
    '''
    Manages an array of eight VL53L0X proximity sensors as implemented on the
    Radiozoa sensor board. Assumes all sensors are already configured at addresses
    0x30-0x37 by the RadiozoaConfig script.
    '''
    # thresholds in millimeters
    CLOSE_THRESHOLD = 100
    NEAR_THRESHOLD  = 200
    MID_THRESHOLD   = 600
    FAR_THRESHOLD   = 1000

    def __init__(self, i2c_id=1, level=Level.INFO):
        self._log = Logger('radiozoa', level=level)
        self._i2c = I2C(i2c_id)
        self._sensors = {}
        self._is_ranging = False
        self._create_sensors()
        self._distance_offset = 50
        self._log.info('ready.')

    @property
    def is_ranging(self): 
        return self._is_ranging

    def _create_sensors(self):
        '''
        Creates VL53L0X, VL53L1X or null instances for all eight sensors using Device pseudo-enum.
        '''
        for dev in Device.all():
            cardinal = Cardinal._registry[dev.index]
            self._log.info('creating sensor {} at 0x{:02X}…'.format(cardinal.name, dev.i2c_address))
            try:
                if dev.impl == 'VL53L0X':
                    sensor = VL53L0X(self._i2c, address=dev.i2c_address)
                elif dev.impl == 'VL53L1X':
                    sensor = VL53L1X(self._i2c, address=dev.i2c_address)
                else:
                    sensor = None
                self._sensors[cardinal] = sensor
                self._log.info('sensor {} created.'.format(cardinal.name))
            except Exception as e:
                self._log.error('{} raised creating sensor {}: {}'.format(type(e), cardinal.name, e))
                raise

    def dump(self):
        for cardinal, sensor in self._sensors.items():
            if sensor:
                sensor.stop()

    def start_ranging(self):
        '''
        Starts ranging for all sensors.
        '''
        if not self._is_ranging:
            self._log.info('starting ranging…')
            for cardinal, sensor in self._sensors.items():
                if sensor:
                    try:
                        sensor.start()
                        self._log.info('sensor {} ranging started.'.format(cardinal.name))
                    except Exception as e:
                        self._log.error('{} raised starting sensor {}: {}'.format(type(e), cardinal.name, e))
            self._is_ranging = True
            time.sleep_ms(100)
            self._log.info('ranging started.')
        else:
            self._log.warning('already ranging.')

    def stop_ranging(self):
        '''
        Stops ranging for all sensors.
        '''
        if self._is_ranging:
            self._log.info('stopping ranging…')
            for cardinal, sensor in self._sensors.items():
                if sensor:
                    try:
                        sensor.stop()
                        self._log.info('sensor {} ranging stopped.'.format(cardinal.name))
                    except Exception as e:
                        self._log.error('{} raised stopping sensor {}: {}'.format(type(e), cardinal.name, e))
            self._is_ranging = False
            self._log.info('ranging stopped.')
        else:
            self._log.warning('not currently ranging.')

    def get_distance(self, cardinal):
        '''
        Get distance reading from a single sensor. This subtracts the distance offset constant.

        Args:
            cardinal: Cardinal direction

        Returns:
            int: distance in millimeters, or None on error
        '''
        sensor = self._sensors.get(cardinal)
        if sensor:
            try:
                dist = max(0, sensor.read() - self._distance_offset)
                return dist
            except Exception as e:
                self._log.error('{} raised reading sensor {}: {}'.format(type(e), cardinal.name, e))
                return Sensor.OUT_OF_RANGE
        else:
            self._log.warning('no sensor for cardinal {}'.format(cardinal.name))
            return Sensor.OUT_OF_RANGE

    def get_distances(self, cardinals=None):
        '''
        Returns distance readings for specified cardinal directions, or all eight if no argument.

        Args:
            cardinals (list of Cardinal, optional): List of Cardinal values to select readings.
                                                   If None, returns all eight sensor readings.

        Returns:
            list: List of distances in mm (or Sensor.OUT_OF_RANGE for failed reads), order matches
            Cardinal registry.
        '''
        if cardinals is None:
            # return all eight distances in registry order
            distances = []
            for cardinal in Cardinal._registry:
                sensor = self._sensors.get(cardinal)
                if sensor:
                    try:
                        dist = max(0, sensor.read() - self._distance_offset)
                        distances.append(dist)
                    except Exception as e:
                        self._log.error('{} reading sensor {}: {}'.format(type(e), cardinal.name, e))
                        distances.append(Sensor.OUT_OF_RANGE)
                else:
                    distances.append(Sensor.OUT_OF_RANGE)
            return distances
        else:
            # return only specified cardinals
            distances = []
            for cardinal in cardinals:
                sensor = self._sensors.get(cardinal)
                if sensor:
                    try:
                        dist = max(0, sensor.read() - self._distance_offset)
                        distances.append(dist)
                    except Exception as e:
                        self._log.error('{} reading sensor {}: {}'.format(type(e), cardinal.name, e))
                        distances.append(Sensor.OUT_OF_RANGE)
                else:
                    distances.append(Sensor.OUT_OF_RANGE)
            return distances

    def _color_for_distance(self, dist):
        '''
        Return color code for distance value based on thresholds.
        '''
        if dist is None or dist <= 0:
            return Fore.BLACK + Style.DIM
        elif dist < self.CLOSE_THRESHOLD:
            return Fore.MAGENTA + Style.BRIGHT
        elif dist < self.NEAR_THRESHOLD:
            return Fore.RED
        elif dist < self.MID_THRESHOLD:
            return Fore.YELLOW
        elif dist < self.FAR_THRESHOLD:
            return Fore.GREEN
        else:
            return Fore.BLACK

    def print_distances(self):
        '''
        Print colorized distance values from all eight sensors.
        '''
        distances = self.get_distances()
        msg = "["
        for i, dist in enumerate(distances):
            color = self._color_for_distance(dist)
            if dist is not None and dist > 0:
                msg += " {}{:>4}mm{}".format(color, dist, Style.RESET_ALL)
            else:
                msg += " {}None{}".format(color, Style.RESET_ALL)
            if i < len(distances) - 1:
                msg += ","
        msg += " ]"
        print(msg)

    def close(self):
        '''
        Stop ranging and clean up.
        '''
        self._log.info('closing…')
        if self._is_ranging:
            self.stop_ranging()
        self._log.info('closed.')

#EOF
