#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-07
# modified: 2025-10-16

from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level
from core.orientation import Orientation
from hardware.distance_sensor import DistanceSensor
from hardware.easing import Easing

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DistanceSensors(Component):
    NAME = 'distances'
    '''
    Collects the three IR distance sensors into a class. This is a raw sensing
    class with no timing or integration into the rest of the operating system.
    It does support weighted averages of the center and port, and center and
    starboard sensors.

    This uses PSID (port side), SSID (starboard side) and FWD (forward) as the
    three orientations for the sensors, matching the current hardware layout.

    :param config:            the application configuration
    :param level:             the log level
    '''
    def __init__(self, config, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._level = level
        self._log = Logger(DistanceSensors.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['kros'].get('hardware').get('distance_sensors')
        self._min_distance     = _cfg.get('min_distance', 80)
        self._default_distance = _cfg.get('max_distance', 270)
        _easing_value          = _cfg.get('easing', 'linear')
        self._easing           = Easing.from_string(_easing_value)
        # sensors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._psid_sensor = DistanceSensor(config, Orientation.PSID)
        self._fwd_sensor  = DistanceSensor(config, Orientation.FWD)
        self._ssid_sensor = DistanceSensor(config, Orientation.SSID)
        self._sensors = {
           Orientation.PSID: self._psid_sensor,
           Orientation.FWD:  self._fwd_sensor,
           Orientation.SSID: self._ssid_sensor
        }
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enable the three sensors as well as this class.
        '''
        for _sensor in self._sensors.values():
            _sensor.enable()
        Component.enable(self)
        self._log.info('enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def sensors(self):
        '''
        Return the Orientation-keyed dictionary of sensors as a property.
        '''
        return self._sensors

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __iter__(self):
        '''
        Iterate over the sensors, permitting this construction:

            for _sensor in _sensors:
                distance_mm = _sensor.distance
        '''
        return iter(self._sensors.values())

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def psid(self):
        return self._psid_sensor.distance

    @property
    def fwd(self):
        return self._fwd_sensor.distance

    @property
    def ssid(self):
        return self._ssid_sensor.distance

    @property
    def all(self):
        return self._psid_sensor.distance, self._fwd_sensor.distance, self._ssid_sensor.distance

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get(self, orientation: Orientation):
        '''
        Get method to retrieve the sensor by Orientation.
        '''
        return self._sensors.get(orientation, None)  # Returns the sensor or None if not found

    # weighted averages support ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def normalise_distance(self, dist):
        '''
        Normalize distance to a value between 0.0 and 1.0 using the instance's
        configured easing method and min/max distance settings.
        '''
        dist = max(min(dist, self._default_distance), self._min_distance)
        normalised = (dist - self._min_distance) / (self._default_distance - self._min_distance)
        return self._easing.apply(normalised)

    def get_weighted_averages(self):
        '''
        Read current distances or substitute default. This returns a tuple
        containing the port and starboard values, normalised with the center
        value.

        NOTE: This only has validity if the three sensors are all aiming
        relatively forward, e.g., the center sensor is 0°, the port sensor is
        at -45°, the starboard sensor 45°. This is kept in the code based for
        use with robots that have that configuration.
        '''
        port = self._psid_sensor.distance or self._default_distance
        cntr = self._fwd_sensor.distance  or self._default_distance
        stbd = self._ssid_sensor.distance or self._default_distance
        # compute pairwise averages
        port_avg = (port + cntr) / 2
        stbd_avg = (stbd + cntr) / 2
        # normalise using instance method
        port_norm = self.normalise_distance(port_avg)
        stbd_norm = self.normalise_distance(stbd_avg)
        return port_norm, stbd_norm

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable the three sensors as well as this class.
        '''
        Component.disable(self)
        for _sensor in self._sensors.values():
            _sensor.disable()
        self._log.info('disabled.')

#EOF
