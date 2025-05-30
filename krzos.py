#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2019-12-23
# modified: 2024-10-31
#
# The KRZ03 Robot Operating System (KRZOS), including its command line
# interface (CLI).
#
#        1         2         3         4         5         6         7         8         9         C
#234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
#

import os, sys, signal, time, traceback
import argparse
import itertools
from pathlib import Path
from colorama import init, Fore, Style
init()

import core.globals as globals
globals.init()

from core.logger import Logger, Level
from core.event import Event, Group
from core.component import Component
from core.fsm import FiniteStateMachine, State
from core.orientation import Orientation
from core.util import Util
from core.message import Message, Payload
from core.message_bus import MessageBus
from core.message_factory import MessageFactory
from core.config_loader import ConfigLoader
from core.config_error import ConfigurationError
from core.controller import Controller

from core.publisher import Publisher
from core.queue_publisher import QueuePublisher
from hardware.system_publisher import SystemPublisher

from core.subscriber import Subscriber, GarbageCollector
from hardware.system_subscriber import SystemSubscriber
from hardware.distance_sensors import DistanceSensors
from hardware.distance_sensors_subscriber import DistanceSensorsSubscriber
from hardware.sound_subscriber import SoundSubscriber

from hardware.pigpiod_util import PigpiodUtility
from hardware.system import System
from hardware.rtof import RangingToF
from hardware.rgbmatrix import RgbMatrix # used for ICM20948
from hardware.icm20948 import Icm20948
from hardware.imu import IMU
from hardware.task_selector import TaskSelector
from hardware.i2c_scanner import I2CScanner
from hardware.distance_sensors_publisher import DistanceSensorsPublisher
from hardware.button import Button
from hardware.eyeballs import Eyeballs
from hardware.sound import Sound
from hardware.player import Player
from hardware.differential_drive import DifferentialDrive # subclass of MotorController

from behave.behaviour_manager import BehaviourManager
from behave.macro_processor import MacroProcessor
from behave.avoid import Avoid
from behave.roam import Roam
from behave.idle import Idle
#from behave.swerve import Swerve
#from behave.moth import Moth
#from behave.sniff import Sniff

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class KRZOS(Component, FiniteStateMachine):
    '''
    Extends Component and Finite State Machine (FSM) as a basis of a K-Series
    Robot Operating System (KRZOS) or behaviour-based system (BBS), including
    spawning the various tasks and starting up the Subsumption Architecture,
    used for communication between Components over a common message bus.

    The MessageBus receives Event-containing messages from sensors and other
    message sources, which are passed on to the Arbitrator, whose job it is
    to determine the highest priority action to execute for that task cycle,
    by passing it on to the Controller.

    There is also a krzosd linux daemon, which can be used to start, enable and
    disable krzos.
    '''
    def __init__(self, level=Level.INFO):
        '''
        This initialises KRZOS and calls the YAML configurer.
        '''
        _name = 'krzos'
        self._level = level
        self._log = Logger(_name, self._level)
        self._print_banner()
        self._log.info('…')
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        FiniteStateMachine.__init__(self, self._log, _name)
        self._system                      = System(self, level)
        self._system.set_nice()
        # configuration…
        self._config                      = None
        self._component_registry          = None
        self._controller                  = None
        self._message_bus                 = None
        self._gamepad_publisher           = None
        self._queue_publisher             = None
        self._rtof_publisher              = None
        self._system_publisher            = None
        self._system_subscriber           = None
        self._distance_sensors            = None
        self._distance_sensors_publisher  = None
        self._distance_sensors_subscriber = None
        self._sound_subscriber            = None
        self._task_selector               = None
        self._rgbmatrix                   = None
        self._icm20948                    = None
        self._imu                         = None
        self._behaviour_mgr               = None
        self._macro_processor             = None
        self._avoid                       = None
        self._roam                        = None
#       self._gamepad_controller          = None
        self._motor_controller            = None
        self._tinyfx                      = None
        self._pushbutton                  = None
        self._eyeballs                    = None
        self._killswitch                  = None
        self._started                     = False
        self._closing                     = False
        self._log.info('oid: {}'.format(id(self)))
        self._log.info('initialised.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def configure(self, arguments):
        '''
        Provided with a set of configuration arguments, configures KRZOS based on
        both MR01 hardware as well as optional features, the latter based on
        devices showing up (by address) on the I²C bus. Optional devices are only
        enabled at startup time via registration of their feature availability.
        '''
        self._log.heading('configuration', 'configuring krzos…',
            '[1/2]' if arguments.start else '[1/1]')
        self._log.info('application log level: {}'.format(self._log.level.name))

        # read YAML configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _loader = ConfigLoader(self._level)
        _config_filename = arguments.config_file
        _filename = _config_filename if _config_filename is not None else 'config.yaml'
        self._config = _loader.configure(_filename)
        self._is_raspberry_pi = self._system.is_raspberry_pi()

        _i2c_scanner = I2CScanner(self._config, level=Level.INFO)

        # configuration from command line arguments ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        _args = self._config['krzos'].get('arguments')
        # copy argument-based configuration over to _config (changing the names!)

        _args['log_enabled']    = arguments.log
        self._log.info('write log enabled:    {}'.format(_args['log_enabled']))

        _args['motors_enabled'] = arguments.motors
        self._log.info('😃 motors enabled:      {}'.format(_args['motors_enabled']))

        _args['json_dump_enabled'] = arguments.json
        self._log.info('json enabled:         {}'.format(_args['json_dump_enabled']))

#       self._log.info('argument gamepad:     {}'.format(arguments.gamepad))
        _args['gamepad_enabled'] = arguments.gamepad and self._is_raspberry_pi
        self._log.info('gamepad enabled:      {}'.format(_args['gamepad_enabled']))

        # print remaining arguments
        self._log.info('argument config-file: {}'.format(arguments.config_file))
        self._log.info('argument level:       {}'.format(arguments.level))

        # establish basic subsumption components ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        self._log.info('configure subsumption components…')

        self._message_bus = MessageBus(self._config, self._level)
        self._message_factory = MessageFactory(self._message_bus, self._level)

        self._controller = Controller(self._message_bus, self._level)

        # JSON configuration dump ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if _args['json_dump_enabled']:
            print('exporting JSON configuration.')
            # TODO

        # create components ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        _cfg = self._config['krzos'].get('component')
        self._component_registry = globals.get('component-registry')

        # basic hardware ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        # create subscribers ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        _subs = arguments.subs if arguments.subs else ''

        if _cfg.get('enable_system_subscriber') or 's' in _subs:
            self._system_subscriber = SystemSubscriber(self._config, self, self._message_bus, level=self._level)

#       if _cfg.get('enable_omni_subscriber') or 'o' in _subs:
#           self._omni_subscriber = OmniSubscriber(self._config, self._message_bus, level=self._level) # reacts to IR sensors

        if _cfg.get('enable_distance_subscriber'):
            self._distance_sensors_subscriber = DistanceSensorsSubscriber(self._config, self._message_bus, level=self._level) # reacts to IR sensors

        if _cfg.get('enable_sound_subscriber'):
            self._sound_subscriber = SoundSubscriber(self._config, self._message_bus, level=self._level)

        # create publishers  ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        _pubs = arguments.pubs if arguments.pubs else ''

        if _cfg.get('enable_queue_publisher') or 'q' in _pubs:
            self._queue_publisher = QueuePublisher(self._config, self._message_bus, self._message_factory, self._level)

        _enable_system_publisher = _cfg.get('enable_system_publisher')
        if _enable_system_publisher:
            self._system_publisher = SystemPublisher(self._config, self._message_bus, self._message_factory, self._system, level=self._level)

        _enable_distance_publisher = _cfg.get('enable_distance_publisher')
        if _enable_distance_publisher:
            self._distance_sensors = DistanceSensors(self._config, level=self._level)
            self._distance_sensors_publisher = DistanceSensorsPublisher(self._config, self._message_bus, self._message_factory, self._distance_sensors, level=self._level)

        # optionally used by ICM20948
        self._rgbmatrix = RgbMatrix(enable_port=True, enable_stbd=True, level=self._level)

        _enable_imu_publisher = _cfg.get('enable_imu_publisher')
        if _enable_imu_publisher:
            if _i2c_scanner.has_hex_address(['0x69']):
                self._icm20948 = Icm20948(self._config, self._rgbmatrix, level=self._level)
                self._imu = IMU(self._config, self._icm20948, self._message_bus, self._message_factory, level=self._level)
            else:
                self._log.warning('no IMU available.')

        _enable_rtof_publisher = _cfg.get('enable_rtof_publisher')
        if _enable_rtof_publisher:
            if _i2c_scanner.has_hex_address(['0x29']):
                self._rtof_publisher = RangingToF(self._config, self._message_bus, self._message_factory, level=self._level)
            else:
                self._log.warning('rtof disabled: no VL53L5CX found.')

        _enable_tinyfx_controller = _cfg.get('enable_tinyfx_controller')
        if _enable_tinyfx_controller:
            if not _i2c_scanner.has_hex_address(['0x45']):
                raise Exception('tinyfx not available on I2C bus.')

            from hardware.tinyfx_controller import TinyFxController

            self._log.info('configure tinyfx controller…')
            if self._component_registry.has(TinyFxController.NAME):
                self._tinyfx = self._component_registry.get(TinyFxController.NAME)
            else:
                self._tinyfx = TinyFxController(self._config)
                self._tinyfx.enable()
            self._log.info('instantiate sound player…')
            Player.instance(self._tinyfx)

        _enable_pushbutton= _cfg.get('enable_pushbutton')
        if _enable_pushbutton:
            self._pushbutton = Button(self._config, name='trig', pin=17, momentary=True)
            self._pushbutton.add_callback(self._await_start, 500)
            if _enable_tinyfx_controller:
                Player.play(Sound.BEEP)

        _enable_killswitch= _cfg.get('enable_killswitch') or 'k' in _pubs
        if _enable_killswitch:
            self._killswitch = Button(self._config, name='kill', pin=18, momentary=False)
            self._killswitch.add_callback(self.shutdown, 500)

        # GPIO 21 is reserved for krzosd

        _enable_eyeballs = _cfg.get('enable_eyeballs')
        if _enable_eyeballs:
            self._eyeballs = Eyeballs()

        # add task selector ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        # and finally, the garbage collector:
        self._garbage_collector = GarbageCollector(self._config, self._message_bus, level=self._level)

        # create motor controller ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        _enable_motor_controller = _cfg.get('enable_motor_controller')
        if _args['motors_enabled'] and _enable_motor_controller:
            if not _i2c_scanner.has_hex_address(['0x44']):
                raise Exception('motor 2040 not available on I2C bus.')
            self._motor_controller = DifferentialDrive(self._config, level=self._level)
            # before disabling the message bus, first stop the motors on system shutdown
            self._message_bus.add_callback_on_stop(self._motor_controller.stop)

        # create behaviours ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        _behaviour_cfg = self._config['krzos'].get('behaviour')
        if _behaviour_cfg.get('enable_macro_processor'):
            self._macro_processor = MacroProcessor(self._config, self._motor_controller, self._eyeballs, level=self._level)

        _enable_behaviours = _behaviour_cfg.get('enable_behaviours') or Util.is_true(arguments.behave)
        if _enable_behaviours:
            self._behaviour_mgr = BehaviourManager(self._config, self._message_bus, self._message_factory, self._level) # a specialised subscriber
            self._log.info('behaviour manager enabled.')

            if _behaviour_cfg.get('enable_avoid_behaviour'):
                self._avoid = Avoid(self._config, self._message_bus, self._message_factory, level=self._level)
            if _behaviour_cfg.get('enable_roam_behaviour'):
                self._roam  = Roam(self._config, self._message_bus, self._message_factory, self._motor_controller, self._distance_sensors)

            _unused = '''
            _bcfg = self._config['krzos'].get('behaviour')
            # create and register behaviours (listed in priority order)

            if _bcfg.get('enable_swerve_behaviour'):
                self._swerve = Swerve(self._config, self._message_bus, self._message_factory, self._motor_controller,
                        self._external_clock, level=self._level)
            if _bcfg.get('enable_moth_behaviour'):
                self._moth   = Moth(self._config, self._message_bus, self._message_factory, self._motor_controller, self._level)
            if _bcfg.get('enable_sniff_behaviour'):
                self._sniff  = Sniff(self._config, self._message_bus, self._message_factory, self._motor_controller, self._level)
            '''
            if _behaviour_cfg.get('enable_idle_behaviour'):
                self._idle   = Idle(self._config, self._message_bus, self._message_factory, self._level)

        if _args['gamepad_enabled'] or _cfg.get('enable_gamepad_publisher') or 'g' in _pubs:
            from hardware.gamepad_publisher import GamepadPublisher
            from hardware.gamepad_controller import GamepadController

            self._gamepad_publisher = GamepadPublisher(self._config, self._message_bus, self._message_factory, exit_on_complete=True, level=self._level)
#           self._gamepad_controller = GamepadController(self._message_bus, self._level)

        # finish up ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        self._export_config = False
        if self._export_config:
            self.export_config()
        self._log.info('configured.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def has_await_pushbutton(self):
        return self._pushbutton != None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _await_start(self, arg=None):
        self._log.info('await start callback triggered…')
        self._pushbutton.close()
        self._pushbutton = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def start(self, dummy=None):
        '''
        This first disables the Pi's status LEDs, establishes the message bus,
        arbitrator, controller, enables the set of features, then starts the main
        OS loop.
        '''
        if self._started:
            self._log.warning('already started.')
            # could toggle callback on pushbutton?
            return
        self._log.heading('starting', 'starting k-series robot operating system (krzos)…', '[2/2]' )
        FiniteStateMachine.start(self)

        if self._system_subscriber:
            self._log.info('enabling system subscriber…')
            self._system_subscriber.enable()

        if self._tinyfx: # turn on running lights
            _cfg = self._config.get('krzos').get('hardware').get('tinyfx-controller')
            _enable_mast_light = _cfg.get('enable_mast_light')
            if _enable_mast_light:
                self._tinyfx.channel_on(Orientation.MAST)
            _enable_nav_light = _cfg.get('enable_nav_lights')
            if _enable_nav_light:
                self._tinyfx.channel_on(Orientation.PORT)
                time.sleep(0.1)
                self._tinyfx.channel_on(Orientation.STBD)

        PigpiodUtility.ensure_pigpiod_is_running()

        # begin main loop ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        self._log.notice('Press Ctrl-C to exit.')
        self._log.info('begin main os loop.\r')

        if self._motor_controller:
            self._motor_controller.enable()

        # we enable ourself if we get this far successfully
        Component.enable(self)
        FiniteStateMachine.enable(self)

        # print registry of components
        self._component_registry.print_registry()

        if self._tinyfx:
            # instantiate singleton with existing TinyFX
            Player.play(Sound.BEEP)

        # ════════════════════════════════════════════════════════════════════
        # now in main application loop until quit or Ctrl-C…
        self._started = True
        self._log.info('enabling message bus…')
        self._message_bus.enable()
        # that blocks so we never get here until the end…
#       self._log.info('main loop closed.')

        # end main loop ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_config(self):
        '''
        Returns the application configuration.
        '''
        return self._config

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_component_registry(self):
        '''
        Return the registry of all instantiated Components.
        '''
        return self._component_registry

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_logger(self):
        '''
        Returns the application-level logger.
        '''
        return self._log

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_level(self):
        '''
        Returns the log level of the application.
        '''
        return self._level

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_message_bus(self):
        '''
        Returns the MessageBus.
        '''
        return self._message_bus

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_message_factory(self):
        '''
        Returns the MessageFactory.
        '''
        return self._message_factory

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_behaviour_manager(self):
        '''
        Returns the BehaviourManager, None if not used.
        '''
        return self._behaviour_mgr

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_gamepad_publisher(self):
        '''
        Returns the GamepadPublisher, None if not used.
        '''
        return self._gamepad_publisher

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_queue_publisher(self):
        '''
        Returns the QueuePublisher, None if not used.
        '''
        return self._queue_publisher

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_distance_sensors(self):
        '''
        Returns the DistanceSensors, None if not used.
        '''
        return self._distance_sensors

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_motor_controller(self):
        '''
        Returns the motor controller.
        '''
        return self._motor_controller

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _set_pi_leds(self, enable):
        '''
        Enables or disables the Raspberry Pi's board LEDs.
        '''
        sudo_name = self._config['pi'].get('sudo_name')
        _led_0_path = self._config['pi'].get('led_0_path')
        _led_0 = Path(_led_0_path)
        _led_1_path = self._config['pi'].get('led_1_path')
        _led_1 = Path(_led_1_path)
        if _led_0.is_file() and _led_0.is_file():
            if enable:
                self._log.info('re-enabling LEDs…')
                os.system('echo 1 | {} tee {}'.format(sudo_name,_led_0_path))
                os.system('echo 1 | {} tee {}'.format(sudo_name,_led_1_path))
            else:
                self._log.debug('disabling LEDs…')
                os.system('echo 0 | {} tee {}'.format(sudo_name,_led_0_path))
                os.system('echo 0 | {} tee {}'.format(sudo_name,_led_1_path))
        else:
            self._log.warning('could not change state of LEDs: does not appear to be a Raspberry Pi.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def shutdown(self, arg=None):
        '''
        This halts any motor activity, demands a sudden halt of all tasks,
        then shuts down the OS.
        '''
        if self._pushbutton:
            self._pushbutton.cancel()
            self._pushbutton = None
        self._log.info(Fore.MAGENTA + 'shutting down…')
        self.close()
        # we never get here if we shut down properly
        self._log.error('shutdown error.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        This permanently disables the KRZOS.
        '''
        if self.closed:
            self._log.warning('already closed.')
        elif self._closing:
            self._log.warning('already closing.')
        elif self.enabled:
            if self._eyeballs:
                self._eyeballs.disable()
            if self._tinyfx:
                Player.play(Sound.HZAH)
            self._log.info('disabling…')
            if self._task_selector:
                self._task_selector.close()
            if self._queue_publisher:
                self._queue_publisher.disable()
#           PigpiodUtility.stop_pigpiod()
            Component.disable(self)
            FiniteStateMachine.disable(self)
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')
        return True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def closing(self):
        return self._closing

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        This closes KRZOS and sets the robot to a passive, stable state
        following a session.
        '''
        if self.closed:
            self._log.warning('already closed.')
        elif self.closing:
            self._log.warning('already closing.')
        else:
            try:
                self._log.info('closing…')
#               Component.disable(self)
                self._closing = True

                self._log.info(Fore.MAGENTA + Style.BRIGHT + '🌸 a. closing behaviours…')
                self._behaviour_mgr.close_all_behaviours()

                self._log.info(Fore.MAGENTA + Style.BRIGHT + '🌸 b. closing subscribers and publisher…')
                # closes all components that are not a publisher, subscriber, the message bus or krzos itself…
                for _component in list(self._component_registry):
                    if not isinstance(_component, Publisher) and not isinstance(_component, Subscriber) \
                            and _component != self and _component != self._message_bus:
                        self._log.info(Style.BRIGHT + 'closing component \'{}\' ({})…'.format(_component.name, _component.classname))
                        _component.close()
                        self._component_registry.remove_component(_component)
                time.sleep(0.1)

                self._log.info(Fore.MAGENTA + Style.BRIGHT + '🌸 c. closing other components…')
                # closes any remaining non-message bus or krzos…
                for _component in list(self._component_registry):
                    if _component != self and _component != self._message_bus:
                        self._log.info(Style.BRIGHT + 'closing component \'{}\' ({})…'.format(_component.name, _component.classname))
                        _component.close()
                        self._component_registry.remove_component(_component)

                self._log.info(Fore.MAGENTA + Style.BRIGHT + '🌸 d. waiting for components to close…; {} are still open.'.format(self._component_registry.count_open_components()))
                self.wait_for_components_to_close()
                self._log.info(Fore.MAGENTA + Style.BRIGHT + '🌸 e. finished waiting for components to close; {} are still open.'.format(self._component_registry.count_open_components()))

                self._log.info(Fore.MAGENTA + Style.BRIGHT + '🌸 f. closing krzos…')
                Component.close(self) # will call disable()
                FiniteStateMachine.close(self)

                self._log.info(Fore.MAGENTA + Style.BRIGHT + '🌸 g. closing the message bus…')
                self._log.info(Fore.MAGENTA + Style.BRIGHT + 'closing message bus…')
                if self._message_bus and not self._message_bus.closed:
                    self._log.info(Fore.MAGENTA + Style.BRIGHT + 'closing message bus from krzos…')
                    self._message_bus.close()
                    self._log.info(Fore.MAGENTA + Style.BRIGHT + 'message bus closed.')

                # stop using logger here
                print(Fore.MAGENTA + '\n🌸 h. application closed.\n' + Style.RESET_ALL)

            except Exception as e:
                print(Fore.RED + 'error closing application: {}\n{}'.format(e, traceback.format_exc()) + Style.RESET_ALL)
            finally:
                PigpiodUtility.wait_for_daemon_to_stop()
                self._log.close()
                self._closing = False
                if REPORT_REMAINING_FRAMES:
                    _threads = sys._current_frames().items()
                    if len(_threads) > 1:
                        try:
                            print(Fore.WHITE + '{} threads remain upon closing.'.format(len(_threads)) + Style.RESET_ALL)
                            frames = sys._current_frames()
                            for thread_id, frame in frames.items():
                                print(Fore.WHITE + '    remaining frame: ' + Fore.YELLOW + "Thread ID: {}, Frame: {}".format(thread_id, frame) + Style.RESET_ALL)
                        except Exception as e:
                            print('error showing frames: {}\n{}'.format(e, traceback.format_exc()))
                        finally:
                            print('\n')
                    else:
                        print(Fore.WHITE + 'no threads remain upon closing.' + Style.RESET_ALL)
                sys.exit(0)

    def wait_for_components_to_close(self, check_interval=0.1, timeout=3.0):
        """
        Waits until all non-Publisher, non-Subscriber components (excluding
        the application and the message bus) report that they are closed.
        
        :param check_interval:     Seconds between checks.
        :param timeout:            Optional timeout in seconds. None means wait indefinitely.
        :raises TimeoutError:      If timeout is exceeded before all components close.
        """
        start_time = time.time()
        while True:
            remaining = [
                c for c in self._component_registry
                if not isinstance(c, Publisher)
                and not isinstance(c, Subscriber)
                and c != self
                and c != self._message_bus
                and not c.closed()
            ]
            if not remaining:
                self._log.info("all relevant components have closed.")
                break
            if timeout is not None and (time.time() - start_time) > timeout:
                names = ', '.join(c.name for c in remaining)
                raise TimeoutError(f"timeout waiting for components to close: {names}")
            time.sleep(check_interval)
            print('🌸 ping')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def export_config(self):
        '''
        Exports the current configuration to a YAML file named ".config.yaml".
        '''
        self._log.info('exporting configuration to file…')
        _loader.export(self._config, comments=[ \
            '┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈', \
            '      YAML configuration for K-Series Robot Operating System (KRZOS)          ', \
            '┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈', \
            '', \
            'exported: {}'.format(Util.get_timestamp()) ])

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _print_banner(self):
        '''
        Display banner on console.
        '''
        self._log.info(' ')
        self._log.info(' ')
        self._log.info('      ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓ ')
        self._log.info('      ┃                                                                 ┃ ')
        self._log.info('      ┃    █▒▒    █▒▒  █▒▒▒▒▒▒▒    █▒▒▒▒▒▒▒   █▒▒▒▒▒▒    █▒▒▒▒▒▒   █▒▒  ┃ ')
        self._log.info('      ┃    █▒▒  █▒▒    █▒▒   █▒▒       █▒▒   █▒▒   █▒▒  █▒▒        █▒▒  ┃ ')
        self._log.info('      ┃    █▒▒▒▒▒▒     █▒▒▒▒▒▒▒      █▒▒     █▒▒   █▒▒   █▒▒▒▒▒▒   █▒▒  ┃ ')
        self._log.info('      ┃    █▒▒  █▒▒    █▒▒   █▒▒    █▒▒      █▒▒   █▒▒        █▒▒       ┃ ')
        self._log.info('      ┃    █▒▒    █▒▒  █▒▒    █▒▒  █▒▒▒▒▒▒▒   █▒▒▒▒▒▒    █▒▒▒▒▒▒   █▒▒  ┃ ')
        self._log.info('      ┃                                                                 ┃ ')
        self._log.info('      ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛ ')
        self._log.info(' ')
        self._log.info(' ')

    # end of KRZOS class  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━


# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def print_documentation(console=True):
    '''
    Print the extended documentation as imported from the help.txt file. If
    'console' is false, just return its contents.
    '''
    _help_file = Path("help.txt")
    if _help_file.is_file():
        with open(_help_file) as f:
            _content = f.read()
            if console:
                return _content
            else:
                print(_content)
    else:
        if console:
            return 'help file not found.'
        else:
            print('{} not found.'.format(_help_file))

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def parse_args(passed_args=None):
    '''
    Parses the command line arguments and return the resulting args object.
    Help is available via '--help', '-h', or '--docs', '-d' (for extended help),
    or calling the script with no arguments.

    This optionally permits arguments to be passed in as a list, overriding
    sys.argv.
    '''
    _log = Logger('parse-args', Level.INFO)
    _log.debug('parsing…')
#   formatter = lambda prog: argparse.HelpFormatter(prog,max_help_position=60)
    formatter = lambda prog: argparse.RawTextHelpFormatter(prog)
    parser = argparse.ArgumentParser(formatter_class=formatter,
            description='Provides command line control of the K-Series Robot OS application.',
            epilog='This script may be executed by krzosd (krzos daemon) or run directly from the command line.')

    parser.add_argument('--docs',         '-d', action='store_true', help='show the documentation message and exit')
    parser.add_argument('--configure',    '-c', action='store_true', help='run configuration (included by -s)')
    parser.add_argument('--start',        '-s', action='store_true', help='start krzos')
    parser.add_argument('--motors',       '-m', action='store_true', help='enable motors')
    parser.add_argument('--json',         '-j', action='store_true', help='dump YAML configuration as JSON file')
    parser.add_argument('--gamepad',      '-g', action='store_true', help='enable bluetooth gamepad control')
    parser.add_argument('--pubs',         '-P', help='enable publishers as identified by first character')
    parser.add_argument('--subs',         '-S', help='enable subscribers as identified by first character')
    parser.add_argument('--behave',       '-b', help='override behaviour configuration (1, y, yes or true, otherwise false)')
    parser.add_argument('--config-file',  '-f', help='use alternative configuration file')
    parser.add_argument('--log',          '-L', action='store_true', help='write log to timestamped file')
    parser.add_argument('--level',        '-l', help='specify logging level \'DEBUG\'|\'INFO\'|\'WARN\'|\'ERROR\' (default: \'INFO\')')

    try:
        print('')
        args = parser.parse_args() if passed_args is None else parser.parse_args(passed_args)
        if args.docs:
            print(Fore.CYAN + '{}\n{}'.format(parser.format_help(), print_documentation(True)) + Style.RESET_ALL)
            return -1
        elif not args.configure and not args.start:
            print(Fore.CYAN + '{}'.format(parser.format_help()) + Style.RESET_ALL)
            return -1
        else:
            globals.put('log-to-file', args.log)
            return args


    except NotImplementedError as nie:
        _log.error('unrecognised log level \'{}\': {}'.format(args.level, nie))
        _log.error('exit on error.')
        sys.exit(1)
    except Exception as e:
        _log.error('error parsing command line arguments: {}\n{}'.format(e, traceback.format_exc()))
        _log.error('exit on error.')
        sys.exit(1)

# main ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

REPORT_REMAINING_FRAMES = False # debugging

def main(argv):

    _krzos = None

    _log = Logger("main", Level.INFO)
    _suppress = False
    try:
        _args = parse_args()
        if _args == None:
            print('')
            _log.info('arguments: no action.')
        elif _args == -1:
            _suppress = True # help or docs
        else:
            _level = Level.from_string(_args.level) if _args.level != None else Level.INFO
            _log.level = _level
            _log.debug('arguments: {}'.format(_args))
            _krzos = KRZOS(level=_level)
            if _args.configure or _args.start:
                _krzos.configure(_args)
                if not _args.start:
                    _log.info('configure only: ' + Fore.YELLOW + 'specify the -s argument to start krzos.')
            if _args.start:
                _counter = itertools.count() 
                if _krzos.has_await_pushbutton():
                    while _krzos.has_await_pushbutton():
                        if next(_counter) % 20 == 0:
#                           _log.info(Fore.YELLOW + 'waiting for pushbutton…')
                            _log.info(Fore.YELLOW + 'waiting for pushbutton on pin {}…'.format(_krzos._pushbutton.pin))
                        time.sleep(0.1)
                    match _krzos.state:
                        case State.INITIAL: # expected state
                            _krzos.start()
                            # does not return here til we close down
                        case State.CLOSED:
                            _log.info(Fore.WHITE + 'closed before starting.')
                        case _:
                            _log.warning('closed with invalid state: {}'.format(_krzos.state))
                else:
                    _krzos.start()
#               _log.info(Fore.WHITE + 'returned from main loop.')
                    # krzos is now running…
    except KeyboardInterrupt:
        print('\n')
        print(Fore.MAGENTA + Style.BRIGHT + 'caught Ctrl-C; exiting…' + Style.RESET_ALL)
    except RuntimeError as rte:
        _log.error('runtime error starting krzos: {}'.format(rte))
    except Exception:
        _log.error('error starting krzos: {}'.format(traceback.format_exc()))
    finally:
        if _krzos and not _krzos.closed:
            _krzos.close()

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
if __name__== "__main__":
    main(sys.argv[1:])

#EOF
