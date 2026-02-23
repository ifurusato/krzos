#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License.
#
# author:   Ichiro Furusato 
# created:  2026-02-17
# modified: 2026-02-17

import sys
import time

from colors import *
from logger import Logger, Level
from configure import Configure
from controller import Controller
from message_util import pack_message
from ringcontroller import RingController
from radiozoa_sensor import RadiozoaSensor
from radiozoa_config import RadiozoaConfig

class RadiozoaController(RingController):

    def __init__(self, config):
        self._log = Logger('ctrl', level=Level.INFO)
        super().__init__(config)
        self._autostart_delay_ms = 9000
        self._sensor   = None
        self._radiozoa = None
        self._config   = Configure()
        self._PACKED_CARDINAL = pack_message("N    NE   E    SE   S    SW   W    NW") # order of returned data
        self._log.info('ready.')

    def _start_services(self):
        '''
        This method is called upon startup, after a preset delay. It can be overridden
        as necessary to start any application-level services.
        '''
        super()._start_services()
        self._radiozoa_start()

    @property
    def radiozoa(self):
        return self._radiozoa

    def _radiozoa_init(self, force=False):
        '''
        Initialise the Radiozoa sensor.
        '''
        try:
#           if self._config is None:
#               self._config = Configure()
            if self._config.configure(force=force):
                from sensor import Sensor
    
                if self._radiozoa is None:
                    self._radiozoa = RadiozoaSensor()
                if self._sensor is None:
                    self._sensor = Sensor(controller=self)
            else: 
                raise Exception('radiozoa init fail.')
        except Exception as e:
            self._log.error("{} raised by radiozoa_init: {}".format(type(e), e))
            sys.print_exception(e)

    def _radiozoa_reset(self):
        if not self._radiozoa:
            self._log.error('no radiozoa available.')
            return Controller._PACKED_ERR, COLOR_RED
        if self._radiozoa.is_ranging:
            self._radiozoa.stop_ranging()
            self._radiozoa.dump()
            self._radiozoa = None
            time.sleep(0.5)
        try:
            if self._radiozoa is None:
                self._radiozoa = RadiozoaSensor()
            self._log.info('reset radiozoa…')
            if self._sensor:
                self._sensor.disable()
            self._radiozoa_init(force=True)
            self._log.info('radiozoa reset.')
            return Controller._PACKED_ACK, COLOR_DARK_GREEN
        except Exception as e:
            self._log.error("{} raised during Radiozoa reset: {}".format(type(e), e))
            return Controller._PACKED_ERR, COLOR_RED

    def _radiozoa_start(self):
        self._log.info('starting radiozoa…')
        if not self._radiozoa:
            self._radiozoa_init()
            time.sleep_ms(250)
        if self._radiozoa:
            # disable heartbeat
            self._enable_heartbeat(False)
            self._radiozoa.start_ranging()
            if not self._sensor.enabled:
                self._sensor.enable()
        else:   
            self._log.error('failed to start: radiozoa not configured.')
            
    def _radiozoa_stop(self):
        self._log.info('stopping radiozoa…')
        if self._radiozoa:
            self._sensor.disable()
            self._radiozoa.stop_ranging()
            # enable heartbeat
            self._enable_heartbeat(True)
        else:
            self._log.error('failed to stop: radiozoa not configured.')

    def print_help(self):
        super().print_help()
        print('''    radiozoa init | start | stop | reset    # radiozoa control
    cardinal                                # return legend of distances
    distances                               # return eight ToF distances
    poll [<n>]                              # set the polling rate in Hz (no value sets to default)
''')

    def pre_process(self, cmd, arg0, arg1, arg2, arg3, arg4):
        '''
        Pre-process the arguments, returning a response and color if a match occurs.
        Such a match precludes further processing.
        '''
#       self._log.info("radiozoa: pre-process command '{}' with arg0: '{}'; arg1: '{}'; arg2: '{}'; arg3: '{}'; arg4: '{}'".format(cmd, arg0, arg1, arg2, arg3, arg4))
        parts = cmd.split()
        
        if arg0 == "__extend_here__":
            return None, None

        elif arg0 == "scan":
            if self._config:
                self._config.i2cdetect()
                return Controller._PACKED_ACK, COLOR_DARK_GREEN
            else:
                self._log.error("no configure available; use 'radiozoa start' first.")
                return Controller._PACKED_ERR, COLOR_RED

        elif arg0 == "radiozoa":
#           self._log.info('arg0: {}; arg1: {}'.format(arg0, arg1))
            if arg1 == "init":
                self._radiozoa_init()
                return Controller._PACKED_ACK, COLOR_DARK_GREEN

            elif arg1 == "start":
                self._radiozoa_start()
                return Controller._PACKED_ACK, COLOR_DARK_GREEN

            elif arg1 == "stop":
                self._radiozoa_stop()
                self.reset_ring()
                return Controller._PACKED_ACK, COLOR_DARK_GREEN

            elif arg1 == "reset":
                _response, _color = self._radiozoa_reset()
                return _response, _color

            else:
                self._log.error("radiozoa: unrecognised command '{}' with arg0: '{}'; arg1: '{}'; arg2: '{}'; arg3: '{}'; arg4: '{}'".format(cmd, arg0, arg1, arg2, arg3, arg4))
                return Controller._PACKED_ERR, COLOR_RED

        elif arg0 == "cardinal":
                return self._PACKED_CARDINAL, COLOR_DARK_GREEN

        elif arg0 == "distances":
            if self._sensor:
                return self._sensor.distances_packed, COLOR_FUCHSIA
            else:
                self._log.error('no sensor.')
                return Controller._PACKED_ERR, COLOR_RED

        elif arg0 == "poll":
            if self._sensor:
                try:
                    rate_hz = int(arg1) if arg1 is not None else self._sensor.poll_rate_hz
                    self._sensor.set_poll_rate_hz(rate_hz)
                    return Controller._PACKED_ACK, COLOR_DARK_GREEN
                except ValueError:
                    self._log.error("invalid poll rate: '{}'".format(arg1))
                    return Controller._PACKED_ERR, COLOR_RED
            else:
                self._log.error('no sensor.')
                return Controller._PACKED_ERR, COLOR_RED

        else:
            return super().pre_process(cmd, arg0, arg1, arg2, arg3, arg4)

#EOF
