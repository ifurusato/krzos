#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-25
# modified: 2025-11-13
#
# A simple wrapper for the PWM-based DistanceSensor, used by the Avoid behaviour.

from core.logger import Logger, Level
from core.component import Component
from core.orientation import Orientation
from hardware.distance_sensor import DistanceSensor

class AvoidSensor(Component):
    '''
    A simple wrapper around a DistanceSensor, providing a stable, named
    interface for the Avoid behaviour. It is used to get distance readings
    from the port, starboard, and aft sides of the robot.

    :param config:      The application configuration.
    :param orientation: The orientation of the sensor (PORT, STBD, or AFT).
    :param level:       The logging level.
    '''
    def __init__(self, config, orientation, level=Level.INFO):
        if not isinstance(orientation, Orientation):
            raise ValueError('invalid orientation argument: {}'.format(type(orientation)))
        if config is None:
            raise ValueError('no configuration provided.')
        self._orientation = orientation
        self._name = 'avoid-{}'.format(orientation.label)
        self._log = Logger(self._name, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._distance_sensor = DistanceSensor(config, orientation, level)
        self._log.info('ready.')

    @property
    def name(self):
        return self._name

    def get_distance(self):
        '''
        Returns the distance in millimeters from the underlying DistanceSensor.
        '''
        return self._distance_sensor.get_distance()

    def enable(self):
        '''
        Enables the underlying sensor.
        '''
        if not self.enabled:
            self._distance_sensor.enable()
            Component.enable(self)
            self._log.info('enabled.')
        else:
            self._log.info('already enabled.')

    def disable(self):
        '''
        Disables the underlying sensor.
        '''
        if self.enabled:
            self._distance_sensor.disable()
            Component.disable(self)
            self._log.info('disabled.')
        else:
            self._log.info('already disabled.')

    def close(self):
        '''
        Closes the underlying sensor and frees resources.
        '''
        if not self.closed:
            self.disable()
            self._distance_sensor.close()
            Component.close(self)
            self._log.info('closed.')
        else:
            self._log.info('already closed.')

#EOF
