#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the K-Series Robot Operating System (KROS) project, released under the MIT
# License. Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-12-11
# modified: 2025-12-11
#
# This is a conversion from one of the Pimoroni BME680 example scripts into a
# Python class with logging.

import time
import traceback
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

import bme690

from core.component import Component
from core.logger import Logger, Level

class AirQualityMonitor(Component):
    NAME = 'aqm'
    '''
    An air quality monitor that writes its log to a CSV file.

    The delay time between polls is set for 600 seconds (every
    ten minutes), 144 per day.
    '''
    def __init__(self, use_data_logger=False, level=Level.INFO):
        self._log = Logger(AirQualityMonitor.NAME, log_to_file=use_data_logger, level=level)
        self._data_log = Logger(AirQualityMonitor.NAME, log_to_file=True, data_logger=True, level=level) \
                if use_data_logger \
                else None
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        try:
            self._sensor = bme690.BME690(bme690.I2C_ADDR_PRIMARY)
        except (RuntimeError, IOError):
            self._sensor = bme690.BME690(bme690.I2C_ADDR_SECONDARY)
        # These oversampling settings can be tweaked to change the
        # balance between accuracy and noise in the data.
        self._sensor.set_humidity_oversample(bme690.OS_2X)
        self._sensor.set_pressure_oversample(bme690.OS_4X)
        self._sensor.set_temperature_oversample(bme690.OS_8X)
        self._sensor.set_filter(bme690.FILTER_SIZE_3)
        self._sensor.set_gas_status(bme690.ENABLE_GAS_MEAS)
        self._sensor.set_gas_heater_temperature(320)
        self._sensor.set_gas_heater_duration(150)
        self._sensor.select_gas_heater_profile(0)
        self._burn_in_time_sec = 300 # 5 minutes
        self._poll_time_sec = 60
        # calibration variables
        self._gas_baseline = None
        self._humidity_baseline = None
        self._humidity_weighting = None
        self._log.info('ready.')

    def enable(self):
        if not self.enabled:
            super().enable()
            self._log.info('enabled.')
        else:
            self._log.warning('already enabled.')

    def disable(self):
        if not self.disabled:
            super().disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

    def calibrate(self):
        if not self.enabled:
            raise Exception('not enabled.')
        self._log.info(Fore.WHITE + '''

        Estimates indoor air quality.

        Runs the sensor for a burn-in period, then uses a
        combination of relative humidity and gas resistance
        to estimate indoor air quality as a percentage.

        Press Ctrl+C to exit.
        ''')
        start_time   = time.time()
        curr_time    = time.time()
        burn_in_time = self._burn_in_time_sec
        burn_in_data = []
        try:
            # collect gas resistance burn-in values, then use the average
            # of the last 50 values to set the upper limit for calculating
            # gas_baseline.
            self._log.info('collecting gas resistance burn-in data for {} minutes…'.format(round(burn_in_time / 60)))
            while self.enabled and curr_time - start_time < burn_in_time:
                curr_time = time.time()
                if self._sensor.get_sensor_data() and self._sensor.data.heat_stable:
                    gas = self._sensor.data.gas_resistance
                    burn_in_data.append(gas)
                    self._log.info('gas: {0}Ω\t'.format(gas)
                            + Fore.BLACK
                            + '{:>3}s elapsed; burn-in: {}s'.format(int(curr_time - start_time), burn_in_time))
                    time.sleep(1)
            self._gas_baseline = sum(burn_in_data[-50:]) / 50.0
            # set the humidity baseline to 40%, an optimal indoor humidity.
            self._humidity_baseline = 40.0
            # sets the balance between humidity and gas reading in the
            # calculation of air_quality_score (25:75, humidity:gas)
            self._humidity_weighting = 0.25
            self._log.info('gas baseline: {0}Ω, humidity baseline: {1:.2f}% RH'.format(self._gas_baseline, self._humidity_baseline))
        except KeyboardInterrupt:
            pass

    def monitor(self):
        if not self.enabled:
            raise Exception('not enabled.')
        self._log.info(Fore.GREEN + 'starting air quality monitor…')
        if self._data_log:
            self._data_log.data('START')
        _start_time  = dt.now()
        try:
            if self._data_log:
                self._data_log.data('gas (Ω)', 'temperature (C)', 'humidity (%RH)', 'air-quality')
            while self.enabled:
                if self._sensor.get_sensor_data() and self._sensor.data.heat_stable:
                    gas = self._sensor.data.gas_resistance
                    gas_offset = self._gas_baseline - gas
                    humidity = self._sensor.data.humidity
                    hum_offset = humidity - self._humidity_baseline
                    # calculate humidity_score as the distance from the humidity_baseline.
                    if hum_offset > 0:
                        humidity_score = (100 - self._humidity_baseline - hum_offset)
                        humidity_score /= (100 - self._humidity_baseline)
                        humidity_score *= (self._humidity_weighting * 100)
                    else:
                        humidity_score = (self._humidity_baseline + hum_offset)
                        humidity_score /= self._humidity_baseline
                        humidity_score *= (self._humidity_weighting * 100)
                    # calculate gas_score as the distance from the gas_baseline.
                    if gas_offset > 0:
                        gas_score = (gas / self._gas_baseline)
                        gas_score *= (100 - (self._humidity_weighting * 100))
                    else:
                        gas_score = 100 - (self._humidity_weighting * 100)
                    # calculate air_quality_score.
                    air_quality_score = humidity_score + gas_score
                    temperature = self._sensor.data.temperature
                    self._log.info('gas: {:.2f}Ω; temp: {:2.1f}C; humidity: {:.2f}% RH; air quality: {:.2f}'.format(
                            gas_score, temperature, humidity, air_quality_score))
                    if self._data_log:
                        self._data_log.data('{:.2f}'.format(gas_score),
                                       '{:2.1f}'.format(temperature),
                                       '{:.2f}'.format(humidity),
                                       '{:.2f}'.format(air_quality_score))
                    time.sleep(self._poll_time_sec)
        except KeyboardInterrupt:
            self._log.info('Ctrl-C caught; exiting…')
        except Exception as e:
            self._log.error('{} raised in monitor loop: {}\n{}'.format(type(e), e, traceback.format_exc()))
        finally:
            if self._data_log:
                self._data_log.data('END')
                self._data_log.close()
            if self._log:
                _end_time = dt.now()
                _elapsed_time = _end_time - _start_time
                self._log.info(Fore.YELLOW + 'complete: elapsed: {}'.format(_elapsed_time))
                self._log.close()
            self._log.info('complete.')

#EOF
