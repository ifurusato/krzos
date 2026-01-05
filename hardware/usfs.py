#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Portions copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file
# is part of the Robot Operating System project, released under the MIT License.
# Please see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2024-09-03
# modified: 2025-12-19
#
# The Usfs class is used for running the USFS (Ultimate Sensor Fusion Solution)
# SENtral sensor hub as an IMU. This combines a MPU9250 9 DoF IMU (itself
# composed of an MPU6500 accel/gyro with an embedded AK8963C magnetometer), then
# coupled with a Bosch BMP280 pressure/temperature sensor.
#
# See:       https://github.com/simondlevy/USFS
# See:       https://github.com/simondlevy/USFS/tree/master/examples/WarmStartAndAccelCal
# See also:  https://github.com/kriswiner/EM7180_SENtral_sensor_hub/tree/master
# Source:    https://github.com/simondlevy/USFS/blob/master/python/mastertest.py
#
'''
   mastertest.py: Example Python script for running USFS SENtral sensor
   hub in master mode.

   Copyright (C) 2018 Simon D. Levy

   This file is part of USFS.

   USFS is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   USFS is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with USFS. If not, see <http://www.gnu.org/licenses/>.
'''

from usfs import USFS_Master

import math
from collections import deque
from colorama import init, Fore, Style
init()

from core.component import Component
from core.rdof import RDoF
from core.logger import Logger, Level
from hardware.digital_pot import DigitalPotentiometer
from matrix11x7.fonts import font3x5

class Usfs(Component):
    NAME = 'usfs'
    MAG_RATE       = 100  # Hz
    ACCEL_RATE     = 200  # Hz
    GYRO_RATE      = 200  # Hz
    BARO_RATE      = 50   # Hz
    Q_RATE_DIVISOR = 3    # 1/3 gyro rate
    '''
    Wraps the functionality of the Pesky Products EM7180 sensor hub with an
    Invensense MPU9250 gyro/accelerometer and Asahi Kasei AK8963C magnetometer,
    providing options for displaying the heading on an 11x7 LED matrix, and an
    option for providing a digital potentiometer for adjusting the heading (yaw)
    trim.

    To determine the magnetic declination for your location, use NOAA's Magnetic
    Field Calculator at:  https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml

    Args:
        config:       application configuration
        matrix11x7:   optional 11x7 matrix to display heading
        level:        log level
    '''
    def __init__(self, config, matrix11x7=None, level=Level.INFO):
        self._log = Logger(Usfs.NAME, level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._log.info('initialising usfs…')
        if not isinstance(config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(name)))
        self._matrix11x7 = matrix11x7
        # configuration
        _cfg = config['kros'].get('hardware').get('usfs')
        # declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        self._declination_degrees = _cfg.get('declination', 13.8) # set for your location
        self._declination = math.radians(self._declination_degrees)
        self._log.info('declination: {:5.3f}° ({:.6f} rad)'.format(self._declination_degrees, self._declination))
        self._invert_roll  = _cfg.get('invert_roll', False)
        self._invert_pitch = _cfg.get('invert_pitch', False)
        # mount orientation
        self._swap_pitch_roll = _cfg.get('swap_pitch_roll', False)
        if self._swap_pitch_roll:
            self._log.info('pitch and roll axes will be swapped')
        # create USFS
        self._usfs = USFS_Master(self.MAG_RATE, self.ACCEL_RATE, self.GYRO_RATE, self.BARO_RATE, self.Q_RATE_DIVISOR)
        # start the USFS in master mode
        if not self._usfs.begin():
            self._log.error('unable to start USFS: {}'.format(self._usfs.getErrorString()))
            self.close()
        self._use_matrix = matrix11x7 != None
        self._verbose    = False # if true display to console
        # internal values stored in radians
        self._pitch = 0.0
        self._roll  = 0.0
        self._yaw   = 0.0
        # trim values from config (in degrees), converted to radians
        _pitch_trim_degrees = _cfg.get('pitch_trim', 0.0)
        _roll_trim_degrees  = _cfg.get('roll_trim', 0.0)
        _yaw_trim_degrees   = _cfg.get('yaw_trim', None)
        self._pitch_trim = math.radians(_pitch_trim_degrees)
        self._roll_trim  = math.radians(_roll_trim_degrees)
        self._fixed_yaw_trim = math.radians(_yaw_trim_degrees) if _yaw_trim_degrees is not None else None
        self._yaw_trim = 0.0
        # queue for stability check stats
        self._stdev = 0.0
        self._queue_length = _cfg.get('queue_length', 100)  # default to 100 if not in config
        self._queue = deque([], self._queue_length)
        # info
        self._log.info('pitch trim: {:+2.2f}° ({:+.6f} rad); roll trim: {:+2.2f}° ({:+.6f} rad)'.format(
            _pitch_trim_degrees, self._pitch_trim, _roll_trim_degrees, self._roll_trim))
        if self._fixed_yaw_trim:
            self._log.info('using fixed yaw trim: {:+2.2f}° ({:+.6f} rad)'.format(_yaw_trim_degrees, self._fixed_yaw_trim))
        # corrected values (after trim applied), stored in radians
        self._corrected_pitch = 0.0
        self._corrected_roll  = 0.0
        self._corrected_yaw   = 0.0
        # digital pot for dynamic trim adjustment
        self._digital_pot = None
        self._trim_adjust = 0.0
        self._adjust_rdof = None
        _component_registry = Component.get_registry()
        _digital_pot = _component_registry.get(DigitalPotentiometer.NAME)
        if _digital_pot:
            self._digital_pot = _digital_pot
            self._log.info('digital pot available for trim adjustment')
        else:
            self._log.warning('digital pot not available for trim adjustment')
        # barometer/altimeter
        self._pressure    = 0.0
        self._temperature = 0.0
        self._altitude    = 0.0
        # raw sensor values
        self._ax = self._ay = self._az = 0.0
        self._gx = self._gy = self._gz = 0.0
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def is_adjusting_trim(self):
        '''
        Returns True if trim adjust is active.
        '''
        return self._adjust_rdof is not None

    @property
    def trim_adjust(self):
        return self._trim_adjust

    def adjust_trim(self, rdof):
        '''
        Enable trim adjustment for the specified rotational degree of freedom.
        '''
        if not isinstance(rdof, RDoF):
            raise ValueError('argument must be RDoF enum')
        if self._digital_pot is None:
            self._log.warning('digital potentiometer not available, trim adjustment disabled.')
            return
        self._fixed_yaw_trim = None # override any configuration
        self._adjust_rdof = rdof
        self._log.info('trim adjustment enabled for: {}'.format(rdof.label))

    def set_fixed_yaw_trim(self, yaw_degrees):
        '''
        The IMU will use the digital potentiometer for dynamically adjusting
        the yaw trim. If a fixed value is set it is used instead, disabling
        use of the potentiometer.
        '''
        self._fixed_yaw_trim = math.radians(yaw_degrees)

    def set_pitch_trim(self, pitch_trim_degrees):
        '''
        set the pitch trim value in degrees.
        '''
        self._pitch_trim = math.radians(pitch_trim_degrees)

    def set_roll_trim(self, roll_trim_degrees):
        '''
        set the roll trim value in degrees.
        '''
        self._roll_trim = math.radians(roll_trim_degrees)

    def set_verbose(self, verbose):
        self._verbose = verbose

    # roll ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def uncorrected_roll(self):
        '''
        After calling poll(), returns the latest uncorrected roll value in degrees.
        '''
        return math.degrees(self._roll)

    @property
    def uncorrected_roll_radians(self):
        '''
        After calling poll(), returns the latest uncorrected roll value in radians.
        '''
        return self._roll

    @property
    def roll(self):
        '''
        After calling poll(), returns the latest corrected (trimmed) roll value in degrees.
        '''
        return math.degrees(self._corrected_roll)

    @property
    def roll_radians(self):
        '''
        After calling poll(), returns the latest corrected (trimmed) roll value in radians.
        '''
        return self._corrected_roll

    @property
    def roll_trim(self):
        '''
        Returns the current roll trim value in degrees.
        '''
        return math.degrees(self._roll_trim)

    @property
    def roll_trim_radians(self):
        '''
        Returns the current roll trim value in radians.
        '''
        return self._roll_trim

    # pitch ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def uncorrected_pitch(self):
        '''
        After calling poll(), returns the latest uncorrected pitch value in degrees.
        '''
        return math.degrees(self._pitch)

    @property
    def uncorrected_pitch_radians(self):
        '''
        After calling poll(), returns the latest uncorrected pitch value in radians.
        '''
        return self._pitch

    @property
    def pitch(self):
        '''
        After calling poll(), returns the latest corrected (trimmed) pitch value in degrees.
        '''
        return math.degrees(self._corrected_pitch)

    @property
    def pitch_radians(self):
        '''
        After calling poll(), returns the latest corrected (trimmed) pitch value in radians.
        '''
        return self._corrected_pitch

    @property
    def pitch_trim(self):
        '''
        Returns the current pitch trim value in degrees.
        '''
        return math.degrees(self._pitch_trim)

    @property
    def pitch_trim_radians(self):
        '''
        Returns the current pitch trim value in radians.
        '''
        return self._pitch_trim

    # yaw ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def uncorrected_yaw(self):
        '''
        After calling poll(), returns the latest uncorrected yaw value in degrees.
        '''
        return math.degrees(self._yaw)

    @property
    def uncorrected_yaw_radians(self):
        '''
        After calling poll(), returns the latest uncorrected yaw value in radians.
        '''
        return self._yaw

    @property
    def yaw(self):
        '''
        After calling poll(), returns the latest corrected (trimmed) yaw value in degrees.
        '''
        return math.degrees(self._corrected_yaw)

    @property
    def yaw_radians(self):
        '''
        After calling poll(), returns the latest corrected (trimmed) yaw value in radians.
        '''
        return self._corrected_yaw

    @property
    def yaw_trim(self):
        '''
        Returns the current yaw trim value in degrees.
        '''
        return math.degrees(self._yaw_trim)

    @property
    def yaw_trim_radians(self):
        '''
        Returns the current yaw trim value in radians.
        '''
        return self._yaw_trim

    @property
    def standard_deviation(self):
        '''
        Return the current value of the standard deviation of yaw readings
        calculated from the queue.
        '''
        return self._stdev

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def pressure(self):
        '''
        After calling poll(), returns the latest pressure value.
        '''
        return self._pressure

    @property
    def temperature(self):
        '''
        After calling poll(), returns the latest temperature value.
        '''
        return self._temperature

    @property
    def altitude(self):
        '''
        After calling poll(), returns the latest altitude value.
        '''
        return self._altitude

    @property
    def accelerometer(self):
        '''
        After calling poll(), returns the x,y,z tuple value from the accelerometer.
        '''
        return self._ax, self._ay, self._az

    @property
    def gyroscope(self):
        '''
        After calling poll(), returns the x,y,z tuple value from the gyroscope.
        '''
        return self._gx, self._gy, self._gz

    def poll(self):
        '''
        Polls the hardware and sets all the available properties, returning
        the corrected yaw (as that's our primary interest) in degrees.
        '''
        if self.closed:
            self._log.warning('usfs is closed.')
            return None
        self._usfs.checkEventStatus()
        if self._usfs.gotError():
            self._log.error('error starting USFS: {}'.format(self._usfs.getErrorString()))
            self.close()
            return None

        # Define output variables from updated quaternion---these are Tait-Bryan
        # angles, commonly used in aircraft orientation. In this coordinate
        # system, the positive z-axis is down toward Earth. Yaw is the angle
        # between Sensor x-axis and Earth magnetic North (or true North if
        # corrected for local declination, looking down on the sensor positive
        # yaw is counterclockwise. Pitch is angle between sensor x-axis and
        # Earth ground plane, toward the Earth is positive, up toward the sky is
        # negative. Roll is angle between sensor y-axis and Earth ground plane,
        # y-axis up is positive roll. These arise from the definition of the
        # homogeneous rotation matrix constructed from q. Tait-Bryan
        # angles as well as Euler angles are non-commutative that is, the get
        # the correct orientation the rotations must be applied in the correct
        # order which for this configuration is yaw, pitch, and then roll. For
        # more see http://en.wikipedia.org/wiki/Conversion_between_q_and_Euler_angles
        # which has additional links.

        if (self._usfs.gotQuaternion()):
            qw, qx, qy, qz = self._usfs.readQuaternion()
            # calculate roll and pitch from quaternion (in radians)
            _roll_calc  = math.atan2(2.0 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
            _pitch_calc = -math.asin(2.0 * (qx * qz - qw * qy))
            # swap axes if configured for different mount orientation
            if self._swap_pitch_roll:
                self._roll  = _pitch_calc
                self._pitch = _roll_calc
            else:
                self._roll  = _roll_calc
                self._pitch = _pitch_calc
            # invert pitch and/or roll if configured
            if self._invert_roll:
                self._roll *= -1.0
            if self._invert_pitch:
                self._pitch *= -1.0
            # calculate yaw (in radians)
            self._yaw = math.atan2(2.0 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz)
            # apply declination
            self._yaw += self._declination
            # keep yaw between 0 and 2π
            if self._yaw < 0:
                self._yaw += 2 * math.pi
            elif self._yaw >= 2 * math.pi:
                self._yaw -= 2 * math.pi

            # apply trim corrections (all in radians)
            if self._adjust_rdof == RDoF.PITCH and self._digital_pot:
                self._trim_adjust = self._digital_pot.get_scaled_value(False)
                self._corrected_pitch = self._pitch + self._pitch_trim + self._trim_adjust
            else:
                self._corrected_pitch = self._pitch + self._pitch_trim

            if self._adjust_rdof == RDoF.ROLL and self._digital_pot:
                self._trim_adjust = self._digital_pot.get_scaled_value(False)
                self._corrected_roll = self._roll + self._roll_trim + self._trim_adjust
            else:
                self._corrected_roll = self._roll + self._roll_trim

            # yaw trim handling
            if self._fixed_yaw_trim is not None:
                self._yaw_trim = self._fixed_yaw_trim
            elif self._adjust_rdof == RDoF.YAW and self._digital_pot:
                self._trim_adjust = self._digital_pot.get_scaled_value(False)
                self._yaw_trim = self._trim_adjust
            else:
                self._yaw_trim = 0.0
            self._corrected_yaw = self._yaw - self._yaw_trim

            # update stability tracking
            self._queue.append(self._corrected_yaw)
            if len(self._queue) > 1:
                self._stdev = self._circular_stdev(self._queue)

            # keep corrected yaw between 0 and 2π
            if self._corrected_yaw < 0:
                self._corrected_yaw += 2 * math.pi
            elif self._corrected_yaw >= 2 * math.pi:
                self._corrected_yaw -= 2 * math.pi

            if self._verbose:
                _info = Fore.RED + 'roll: {:+6.2f}° (corrected: {:+6.2f}°); '.format(
                    math.degrees(self._roll), math.degrees(self._corrected_roll))
                _info += Fore.GREEN + 'pitch: {:+6.2f}° (corrected: {:+6.2f}°); '.format(
                    math.degrees(self._pitch), math.degrees(self._corrected_pitch))
                _info += Fore.BLUE + 'yaw: {:+6.2f}° (corrected: {:+6.2f}°); '.format(
                    math.degrees(self._yaw), math.degrees(self._corrected_yaw))
                if self._adjust_rdof:
                    if self._adjust_rdof == RDoF.YAW:
                        _info += Fore.CYAN + Style.DIM + 'yaw trim: adj={:7.4f}'.format(
                            math.degrees(self._trim_adjust))
                    elif self._adjust_rdof == RDoF.PITCH:
                        _info += Fore.CYAN + Style.DIM + 'pitch trim: fxd={:7.4f} / adj={:7.4f}'.format(
                            math.degrees(self._pitch_trim), math.degrees(self._trim_adjust))
                    elif self._adjust_rdof == RDoF.ROLL:
                        _info += Fore.CYAN + Style.DIM + 'roll trim: fxd={:7.4f} / adj={:7.4f}'.format(
                            math.degrees(self._roll_trim), math.degrees(self._trim_adjust))
                self._log.info(_info)
            if self._use_matrix:
                self._matrix11x7.clear()
                self._matrix11x7.write_string('{:>3}'.format(int(math.degrees(self._corrected_yaw))), y=1, font=font3x5)
                self._matrix11x7.show()
        else:
            self._log.warning('no quaternion')
        if self._usfs.gotAccelerometer():
            self._ax, self._ay, self._az = self._usfs.readAccelerometer()
        if self._usfs.gotGyrometer():
            self._gx, self._gy, self._gz = self._usfs.readGyrometer()
        if self._usfs.gotBarometer():
            self._pressure, self._temperature = self._usfs.readBarometer()
            self._altitude = (1.0 - math.pow(self._pressure / 1013.25, 0.190295)) * 44330
        return math.degrees(self._corrected_yaw)

    def show_info(self):
        '''
        Displays the RDoFs along with the active trim adjustment.
        '''
        _info =  Fore.RED   + 'roll: {:6.2f}; '.format(self.roll)
        _info += Fore.GREEN + 'pitch: {:6.2f}; '.format(self.pitch)
        _info += Fore.BLUE  + 'yaw: {:6.2f}; '.format(self.yaw)
        if self._adjust_rdof:
            if self._adjust_rdof == RDoF.YAW:
                _info += Fore.CYAN + Style.DIM + 'yaw trim: fxd={:7.4f} / adj={:7.4f}'.format(
                    math.degrees(self._fixed_yaw_trim) if self._fixed_yaw_trim else 0.0,
                    math.degrees(self._trim_adjust))
            elif self._adjust_rdof == RDoF.PITCH:
                _info += Fore.CYAN + Style.DIM + 'pitch trim: fxd={:7.4f} / adj={:7.4f}'.format(
                    math.degrees(self._pitch_trim), math.degrees(self._trim_adjust))
            elif self._adjust_rdof == RDoF.ROLL:
                _info += Fore.CYAN + Style.DIM + 'roll trim: fxd={:7.4f} / adj={:7.4f}'.format(
                    math.degrees(self._roll_trim), math.degrees(self._trim_adjust))
        self._log.info(_info)

    def _circular_stdev(self, angles):
        '''
        calculate standard deviation for circular data (angles in radians).
        returns value in radians.
        '''
        if len(angles) < 2:
            return float('inf')
        sin_sum = sum(math.sin(a) for a in angles)
        cos_sum = sum(math.cos(a) for a in angles)
        n = len(angles)
        sin_mean = sin_sum / n
        cos_mean = cos_sum / n
        r = math.sqrt(sin_mean**2 + cos_mean**2)
        if r > 0.999999: 
            return 0.0
        if r < 0.0001:
            return math.pi
        return math.sqrt(-2 * math.log(r))

    def clear_queue(self):
        '''
        Clears the statistic queue.
        '''
        self._queue.clear()

    def enable(self):
        if not self.closed:
            if self.enabled:
                self._log.warning('USFS already enabled.')
            else:
                super().enable()

    def disable(self):
        super().disable()

    def close(self):
        '''
        Closes the USFS, calling disable.
        '''
        super().close()

#EOF
