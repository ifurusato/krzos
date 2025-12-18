#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2019-12-23
# modified: 2025-12-18
#
# The KRZ04 Robot Operating System (KROS), including its command line
# interface (CLI).
#
#        1         2         3         4         5         6         7         8         9         C
#234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890

import os, sys
import signal
import subprocess
import time
import traceback
import argparse
import itertools
import atexit
from pathlib import Path
from colorama import init, Fore, Style
init()

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
from core.subscriber import Subscriber, GarbageCollector
from core.controller import Controller
from core.publisher import Publisher
from core.queue_publisher import QueuePublisher

from gamepad.gamepad_monitor import GamepadMonitor
from gamepad.gamepad_publisher import GamepadPublisher
from gamepad.gamepad_controller import GamepadController

from hardware.button import Button
from hardware.i2c_scanner import I2CScanner
from hardware.irq_clock import IrqClock
from hardware.compass_encoder import CompassEncoder
from hardware.digital_pot import DigitalPotentiometer
from hardware.motor_controller import MotorController
from hardware.rotation_controller import RotationController, RotationPhase
from hardware.player import Player
from hardware.system import System
from hardware.usfs import Usfs
from hardware.icm20948 import Icm20948
from hardware.vl53l5cx_sensor import Vl53l5cxSensor
from hardware.toggle_config import ToggleConfig
#from hardware.system_publisher import SystemPublisher
#from hardware.sound import Sound
#from hardware.system_subscriber import SystemSubscriber
from hardware.tinyfx_controller import TinyFxController

from behave.behaviour_manager import BehaviourManager

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class KROS(Component, FiniteStateMachine):
    NAME = 'kros'
    '''
    Extends Component and Finite State Machine (FSM) as a basis of a K-Series
    Robot Operating System (KROS) or behaviour-based system (BBS), including
    spawning the various tasks and starting up the Subsumption Architecture,
    used for communication between Components over a common message bus.

    The MessageBus receives Event-containing messages from sensors and other
    message sources, which are passed on to the Arbitrator, whose job it is
    to determine the highest priority action to execute for that task cycle,
    by passing it on to the Controller.

    There is also a krosd linux daemon, which can be used to start, enable and
    disable kros.
    '''
    def __init__(self, level=Level.INFO):
        '''
        This initialises KROS and calls the YAML configurer.
        '''
        self._level = level
        self._log = Logger(KROS.NAME, log_to_file=True, level=self._level)
        self._print_banner()
        self._log.info('…')
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        FiniteStateMachine.__init__(self, self._log, KROS.NAME)
        # components…
        self._config              = None
        self._component_registry  = None
        self._system              = None
        self._message_bus         = None
        self._message_factory     = None
        self._controller          = None
        self._queue_publisher     = None
        self._irq_clock           = None
        self._button              = None
        self._digital_pot         = None
        self._compass_encoder     = None
        self._usfs                = None
        self._icm20948            = None
        self._vl53_sensor         = None
        self._radiozoa_sensor     = None
        self._tinyfx              = None
        self._eyeballs            = None
        self._motor_controller    = None
        self._rotation_controller = None
        self._system_publisher    = None
        self._system_subscriber   = None
        self._behaviour_manager   = None
        self._data_log            = None
        # configuration…
        self._data_logging        = False
        self._gamepad_enabled     = False
        self._started             = False
        self._closing             = False
        self._log.info('oid: {}'.format(id(self)))
        self._log.info('initialised.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def configure(self, arguments):
        '''
        Provided with a set of configuration arguments, configures KROS based on
        both MR01 hardware as well as optional features, the latter based on
        devices showing up (by address) on the I²C bus. Optional devices are only
        enabled at startup time via registration of their feature availability.
        '''
        self._log.heading('configuration', 'configuring kros…',
            '[1/2]' if arguments.start else '[1/1]')
        self._log.info('application log level: {}'.format(self._log.level.name))

        # read YAML configuration

        _loader = ConfigLoader(self._level)
        _config_filename = arguments.config_file
        _filename = _config_filename if _config_filename is not None else 'config.yaml'
        self._config = _loader.configure(_filename)
        if not isinstance(self._config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(name)))

        _i2c_scanner = I2CScanner(self._config, level=Level.INFO)

        # kros application configuration

        _app_cfg = self._config['kros'].get('application')
        self._data_logging = _app_cfg.get('data_logging')

        # toggle switch configuration

        _toggle_config = ToggleConfig(self._config, level=Level.INFO)
        _toggle3 = _toggle_config.toggle(3) # assigned to roam
        _toggle4 = _toggle_config.toggle(4) # unassigned

        _gamepad_enabled_by_toggle = _toggle_config.is_enabled('gamepad')
        _behaviour_manager_enabled_by_toggle = _toggle_config.is_enabled('behaviour_manager')
        _roam_released_by_toggle  = _toggle_config.is_enabled('roam')
        _avoid_released_by_toggle = _toggle_config.is_enabled('avoid')

        # configuration from command line arguments ┈┈┈┈┈┈┈┈

        _args = self._config['kros'].get('arguments')
        # copy argument-based configuration over to _config (changing the names!)

        _args['log_enabled']    = arguments.log
        self._log.info('write log enabled:    {}'.format(_args['log_enabled']))

        _args['motors_enabled'] = arguments.motors
        self._log.info('motors enabled:      {}'.format(_args['motors_enabled']))

        _args['json_dump_enabled'] = arguments.json
        self._log.info('json enabled:         {}'.format(_args['json_dump_enabled']))

#       self._log.info('argument gamepad:     {}'.format(arguments.gamepad))
        _args['gamepad_enabled'] = arguments.gamepad and self._is_raspberry_pi
        self._gamepad_enabled = _args['gamepad_enabled']
        if not self._gamepad_enabled and _gamepad_enabled_by_toggle:
            self._log.info(Fore.GREEN + 'gamepad enabled from toggle 1.')
            self._gamepad_enabled = True

        self._log.info('gamepad enabled:      {}'.format(self._gamepad_enabled))

        # print remaining arguments
        self._log.info('argument config-file: {}'.format(arguments.config_file))
        self._log.info('argument level:       {}'.format(arguments.level))

        # establish basic subsumption components

        self._log.info('configure subsumption components…')

        self._message_bus = MessageBus(self._config, self._level)
        self._message_factory = MessageFactory(self._message_bus, self._level)
        self._controller = Controller(self._message_bus, self._level)

        # less noisy on startup
        self._message_bus.verbose = False

        # JSON configuration dump
        if _args['json_dump_enabled']:
            print('exporting JSON configuration.')
            # TODO

        # create components

        self._component_registry = Component.get_registry()
        if self._component_registry is None:
            raise ValueError('no component registry available.')
        _cfg = self._config['kros'].get('component')

        if _cfg.get('enable_queue_publisher'): # or 'q' in _pubs:
            self._queue_publisher = QueuePublisher(self._config, self._message_bus, self._message_factory, self._level)

        # basic hardware

        # create system monitor
        self._system = System(config=self._config, kros=self, level=self._level)
        self._system.set_nice() # set KROS as high priority process
        self._is_raspberry_pi = self._system.is_raspberry_pi()

        self._use_external_clock = _cfg.get('enable_external_clock')
        if self._use_external_clock:
            self._log.info('creating external clock…')
            self._irq_clock = IrqClock(self._config, level=self._level)
        else:
            self._irq_clock = None

        self._await_pushbutton = _app_cfg.get('await_pushbutton')
        self._log.info('await pushbutton: {}'.format(self._await_pushbutton))
        _enable_pushbutton = _cfg.get('enable_pushbutton')
        if _enable_pushbutton:
            self._button = Button(config=self._config, name='button', level=self._level)
            if self._await_pushbutton:
                self._button.add_callback(self._await_start)
            else:
                # even if we don't await the start we still set up kill switch
                self._button.add_callback(self.shutdown)

        self._digital_pot = DigitalPotentiometer(self._config, level=self._level)
        self._digital_pot.set_output_range(-1.0, 1.0)

        self._compass_encoder = CompassEncoder(self._config)

#       _enable_vl53l5cx = _cfg.get('enable_vl53l5cx')
#       if _enable_vl53l5cx:
#           self._log.info('creating VL53L5CX sensor…')
#           self._vl53_sensor = Vl53l5cxSensor(self._config, level=self._level)
#           self._vl53_sensor = Vl53l5cxSensor(self._config, skip=('skip' in sys.argv or True in sys.argv), level=self._level)
#           self._vl53_sensor.enable()

        _radiozoa_cfg = self._config['kros'].get('behaviour').get('radiozoa')
        _enable_radiozoa = _radiozoa_cfg.get('enable')
        if _enable_radiozoa:
            from hardware.radiozoa_sensor import RadiozoaSensor

            self._radiozoa_sensor = RadiozoaSensor(self._config)
            self._radiozoa_sensor.enable()
            timeout_seconds = 5
            start_time = time.monotonic()
            while not self._radiozoa_sensor.enabled:
                if time.monotonic() - start_time > timeout_seconds:
                    raise TimeoutError("radiozoa failed to enable within timeout")
                time.sleep(0.1)
            self._radiozoa_sensor.set_visualisation(False)
            self._radiozoa_sensor.start_ranging()

        _enable_tinyfx_controller = _cfg.get('enable_tinyfx_controller')
        if _enable_tinyfx_controller:
            self._log.info('configure tinyfx controller…')
            self._tinyfx = TinyFxController(self._config, level=self._level)
            self._tinyfx.enable()

        # create subscribers

        # create publishers

        # gamepad support

        if _args['gamepad_enabled'] or _cfg.get('enable_gamepad'): # or 'g' in _pubs:
            self._gamepad_publisher = GamepadPublisher(self._config, self._message_bus, self._message_factory, exit_on_complete=True, level=self._level)
#           self._gamepad_controller = GamepadController(self._message_bus, self._level)

        # add garbage collector

        self._garbage_collector = GarbageCollector(self._config, self._message_bus, level=self._level)

        # create motor controller

        self._log.info('creating motor controller…')
        self._motor_controller = MotorController(self._config, external_clock=self._irq_clock, level=self._level)

        self._log.info('creating rotation controller…')
        self._rotation_controller = RotationController(self._config, self._motor_controller, level=Level.INFO)

        # create IMUs

        if _cfg.get('enable_usfs'):
            self._usfs = Usfs(self._config, matrix11x7=None, trim_pot=None, level=self._level)
            # fixed trim determined via observation
            self._usfs.set_fixed_yaw_trim(-72.5) # TODO config
            self._usfs.set_verbose(False)

        if _cfg.get('enable_icm20948'):
            self._icm20948 = Icm20948(self._config, level=Level.INFO)
            self._icm20948.include_accel_gyro(True)

        # create behaviours

        _enable_behaviours = _cfg.get('enable_behaviours')
        if not _enable_behaviours and _behaviour_manager_enabled_by_toggle:
            self._log.info(Fore.GREEN + 'behaviour manager enabled from toggle 2.')
            _enable_behaviours = True

        if _enable_behaviours:
            self._log.info('creating behaviour manager…')
            self._behaviour_manager = BehaviourManager(self._config, self._message_bus, self._message_factory, self._level)

        # finish up

        self._export_config = False
        if self._export_config:
            self.export_config()
        self._log.info('configured.')

    def should_await_pushbutton(self):
        return self._await_pushbutton and self._button != None

    def _await_start(self, arg=None):
        if self._data_log:
            self._data_log.data('STARTED')
        self._log.info('await start callback triggered…')
        self._button.clear_callbacks()
        self._button.add_callback(self.shutdown)
#       self._button.close()
        self._button = None

    def start(self, dummy=None):
        '''
        This first disables the Pi's status LEDs, establishes the message bus,
        arbitrator, controller, enables the set of components such as the motor
        controller, rotation controller, IMUs, then starts the main OS loop.
        '''
        if self._started:
            self._log.warning('already started.')
            # could toggle callback on pushbutton?
            return
        if self._motor_controller:
            if self._motor_controller.check_clock_active(timeout_sec=1.0):
                self._log.info('external clock active.')
            else:
                Player.play('buzz')
                self._log.error('cannot start: external clock is not active.')
                self.shutdown()
                return

        self._log.heading('starting', 'starting k-series robot operating system (kros)…', '[2/2]' )
        FiniteStateMachine.start(self)

#       if self._system_subscriber:
#           self._log.info('enabling system subscriber…')
#           self._system_subscriber.enable()

        if self._irq_clock:
            self._log.info('enabling external clock…')
            self._irq_clock.enable()

        _motor_ctrl_cfg = self._config.get('kros').get('motor_controller')
        if _motor_ctrl_cfg.get('enable'):
            self._log.info('enable motor controller…')
            self._motor_controller.enable()
            if self._rotation_controller:
                self._log.info('enable rotation controller…')
                self._rotation_controller.enable()
        else:
            self._log.warning('motor controller disabled.')
        if self._icm20948:
            self._icm20948.enable()

        if self._tinyfx: # turn on running lights
            if not self._tinyfx.enabled:
                self._tinyfx.enable()

        # begin main loop
        self._log.info(Fore.WHITE + Style.BRIGHT + 'Press Ctrl-C to exit.')

        # we enable ourself if we get this far successfully
        self._log.info('beginning main os loop…')
        super().enable()
        FiniteStateMachine.enable(self)
        # now in main application loop until quit or Ctrl-C…
        self._message_bus.add_callback_on_start(self.started)
        self._started = True
        self._log.info('enabling message bus…')
        self._message_bus.enable()
        # that blocks so we never get here until the end…
        self._log.info('main loop closed.')
        # end main loop

    def started(self):
        '''
        A callback executed upon starting the MessageBus.
        '''
        self._log.info(Fore.MAGENTA + 'kros started.')
        if self._queue_publisher and not self._queue_publisher.enabled:
            self._queue_publisher.enable()

        # OLD LOCATION OF MOTOR CONTROLLER / ROTATION CONTROLLER / ICM20948

        if self._data_logging:
            self._data_log = Logger('kros', log_to_file=True, data_logger=True, level=Level.INFO)
            self._data_log.data('START')
        if self._behaviour_manager:
            self._behaviour_manager.enable()

        Player.play('yippee')

        # print registry of components
        self._component_registry.print_registry()
#       self._message_bus.verbose = False # TEMP
        # register callback on close
        atexit.register(lambda: self._post_cleanup())

    def get_config(self):
        '''
        Returns the application configuration.
        '''
        return self._config

    def get_logger(self):
        '''
        Returns the application-level logger.
        '''
        return self._log

    def get_level(self):
        '''
        Returns the log level of the application.
        '''
        return self._level

    def get_message_bus(self):
        '''
        Returns the MessageBus.
        '''
        return self._message_bus

    def get_message_factory(self):
        '''
        Returns the MessageFactory.
        '''
        return self._message_factory

    def shutdown(self, arg=None):
        '''
        This halts any motor activity, demands a sudden halt of all tasks,
        then shuts down the OS.
        '''
        try:
            Player.play('woow')
            if self._button:
                self._button.close()
                self._button = None
            self._log.info(Fore.WHITE + Style.BRIGHT + 'shutting down…')
            if self._data_log:
                self._data_log.data('SHUTDOWN')
            self.close()
            self._fake_logger_info('shutdown complete.', color=Fore.WHITE+Style.BRIGHT)
        except Exception as e:
            self._fake_logger_info('ERROR: {} raised shutting down: {}'.format(type(e), e), color=Fore.RED)

    def disable(self):
        '''
        This permanently disables the KROS.
        '''
        if self.closed:
            self._log.warning('already closed.')
        if self.enabled:
            if self._closing:
                self._log.info('disabling while closing…')
            else:
                self._log.info('disabling…')
            if self._irq_clock and not self._irq_clock.disabled:
                self._irq_clock.disable()
            super().disable()
            FiniteStateMachine.disable(self)
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')
        return True

    def _post_cleanup(self):
        '''
        Called by atexit after the close of the application.
        '''
        if self._tinyfx: # turn off all lights
            if self._tinyfx.enabled:
                self._log.info(Fore.YELLOW + 'tinyfx signing off…')
                self._tinyfx.play('cricket')
                time.sleep(0.334)
            else:
                self._log.warning('tinyfx already disabled.')
            self._tinyfx.off()
        self._log.info('closing log…')
        # close all logging now
        if self._data_log:
            self._data_log.data('END')
        self._log.close()
        self._report_remaining_frames()
        # print final message using the Logger formatting
        self._fake_logger_info(Fore.MAGENTA + 'application closed.')

    @property
    def closing(self):
        return self._closing

    def close(self):
        '''
        This closes KROS and sets the robot to a passive, stable state
        following a session.
        '''
        if self.closed:
            self._log.warning('already closed.')
        elif self.closing:
            self._log.warning('already closing.')
        else:
            try:
                self._closing = True
                self._log.info('closing…')
                if self._behaviour_manager:
                    self._behaviour_manager.close()
                if self._gamepad_enabled:
                    _gamepad_monitor = self._component_registry.get(GamepadMonitor.NAME)
                    if _gamepad_monitor:
                        _gamepad_monitor.close()
                    else:
                        self._log.warning("could not find gamepad monitor! ('{}') in registry.".format(GamepadMonitor.NAME))
                if self._motor_controller:
                    self._motor_controller.close()
                self._log.info(Fore.MAGENTA + 'closing subscribers and publishers…')
                # closes all remaining components that are not a publisher, subscriber, the message bus or kros itself…
                for _name in self._component_registry.names:
                    _component = self._component_registry.get(_name)
                    if _component is None:
                        # already gone
                        self._log.info(Style.DIM + 'component \'{}\' not found in registry.'.format(_name))
                    elif not isinstance(_component, Publisher) \
                            and not isinstance(_component, Subscriber) \
                            and not isinstance(_component, TinyFxController) \
                            and not isinstance(_component, Player) \
                            and _component != self \
                            and _component != self._message_bus:
                        if not _component.closed:
                            self._log.debug(Style.DIM + 'closing component \'{}\' ({})…'.format(_component.name, _component.classname))
                            _component.close()
                if self._irq_clock and not self._irq_clock.closed:
                    self._irq_clock.close()
#               time.sleep(0.1)
                self._log.info(Fore.MAGENTA + 'closing other components…')
                # closes any remaining non-message bus or kros…
                for _name in self._component_registry.names:
                    _component = self._component_registry.get(_name)
                    if _component is None:
                        # already gone (e.g., behaviours)
                        self._log.debug('component \'{}\' not found in registry.'.format(_name))
                    elif _component != self \
                            and not isinstance(_component, TinyFxController) \
                            and not isinstance(_component, Player) \
                            and _component != self._message_bus:
                        if not _component.closed:
                            self._log.debug(Style.DIM + 'closing component \'{}\' ({})…'.format(_component.name, _component.classname))
                            _component.close()
                            self._component_registry.deregister(_component)
                _open_count = self._component_registry.count_open_components()
                if _open_count > 4: # we expect kros, message bus, tinyfx and player to still be open
                    self._log.warning('waiting for components to close…; {} are still open.'.format(_open_count))
                    self._component_registry.print_registry()
                    _open_count = self._component_registry.count_open_components()
                    if _open_count > 4: # we expect player and tinyfx controller to remain open
                        self.wait_for_components_to_close()
                        self._log.info('finished waiting for components to close; {} remain open:'.format(_open_count))
                        for _name in self._component_registry.names:
                            self._log.info('    {}'.format(_name))
#               if self._tinyfx:
#                   self._tinyfx.off()
                self._log.info(Fore.MAGENTA + 'closing kros…')
                super().close() # will call disable()
                FiniteStateMachine.close(self)
                self._log.info('1/3. kros close.')
                self._log.info('closing the message bus…')
                if self._message_bus and not self._message_bus.closed:
                    self._log.info(Fore.MAGENTA + 'closing message bus from kros…')
                    self._message_bus.close()
                    self._log.info(Fore.MAGENTA + 'message bus closed.')
                else:
                    self._log.warning('message bus not closed.')
                self._log.info('2/3. kros close.')
            except Exception as e:
                print(Fore.RED + 'error closing application: {}\n{}'.format(e, traceback.format_exc()) + Style.RESET_ALL)
            finally:
                self._fake_logger_info('3/3. kros close.')
                self._closing = False

    def _report_remaining_frames(self):
        if REPORT_REMAINING_FRAMES:
            _threads = sys._current_frames().items()
            _thread_count = len(_threads) # not likely Egyptian
            if _thread_count > 1:
                try:
                    self._fake_logger_info(Fore.WHITE + '{} thread{} remain{} upon closing:'.format(
                            _thread_count, 's' if _thread_count > 1 else '', '' if _thread_count > 1 else 's'))
                    frames = sys._current_frames()
                    for thread_id, frame in frames.items():
                        print(Fore.WHITE + '  remaining frame: ' + Fore.YELLOW + "Thread ID: {}, Frame: {}".format(thread_id, frame) + Style.RESET_ALL)
                except Exception as e:
                    print('error showing frames: {}\n{}'.format(e, traceback.format_exc()))
                finally:
                    print('')
            else:
                self._fake_logger_info(Fore.WHITE + 'no threads remain upon closing.')

    def _fake_logger_info(self, message, color=Fore.CYAN):
        '''
        Used when logger is no longer available, print message using the Logger formatting.
        '''
        print(Logger.timestamp_now() + Fore.RESET + ' {:<16} : '.format('kros')
                + color + Style.NORMAL + 'INFO  : {}\n'.format(message) + Style.RESET_ALL)

    def wait_for_components_to_close(self, check_interval=0.01, timeout=3.0):
        '''
        Waits until all non-Publisher, non-Subscriber components (excluding
        the application and the message bus) report that they are closed.

        :param check_interval:     Seconds between checks.
        :param timeout:            Optional timeout in seconds. None means wait indefinitely.
        :raises TimeoutError:      If timeout is exceeded before all components close.
        '''
        start_time = time.time()
        while True:
            remaining = [
                c for c in self._component_registry
                if not isinstance(c, Publisher)
                and not isinstance(c, Subscriber)
                and c != self
                and c != self._message_bus
                and not c.closed
            ]
            if not remaining:
                self._log.info("all relevant components have closed.")
                break
            if timeout is not None and (time.time() - start_time) > timeout:
                names = ', '.join(c.name for c in remaining)
                raise TimeoutError(f"timeout waiting for components to close: {names}")
            time.sleep(check_interval)

    def export_config(self):
        '''
        Exports the current configuration to a YAML file named ".config.yaml".
        '''
        self._log.info('exporting configuration to file…')
        _loader.export(self._config, comments=[ \
            '┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈', \
            '      YAML configuration for K-Series Robot Operating System (KROS)           ', \
            '┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈', \
            '', \
            'exported: {}'.format(Util.get_timestamp()) ])

    def _print_banner(self):
        '''
        Display banner on console.
        '''
        self._log.info(' ')
        self._log.info(' ')
        self._log.info('      ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓ ')
        self._log.info('      ┃                                                       ┃ ')
        self._log.info('      ┃    █▒▒    █▒▒  █▒▒▒▒▒▒▒     █▒▒▒▒▒▒    █▒▒▒▒▒▒   █▒▒  ┃ ')
        self._log.info('      ┃    █▒▒  █▒▒    █▒▒   █▒▒   █▒▒   █▒▒  █▒▒        █▒▒  ┃ ')
        self._log.info('      ┃    █▒▒▒▒▒▒     █▒▒▒▒▒▒▒    █▒▒   █▒▒   █▒▒▒▒▒▒   █▒▒  ┃ ')
        self._log.info('      ┃    █▒▒  █▒▒    █▒▒   █▒▒   █▒▒   █▒▒        █▒▒       ┃ ')
        self._log.info('      ┃    █▒▒    █▒▒  █▒▒    █▒▒   █▒▒▒▒▒▒    █▒▒▒▒▒▒   █▒▒  ┃ ')
        self._log.info('      ┃                                                       ┃ ')
        self._log.info('      ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛ ')
        self._log.info(' ')
        self._log.info(' ')

    # end of KROS class  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

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
            epilog='This script may be executed by krosd (kros daemon) or run directly from the command line.')

    parser.add_argument('--docs',         '-d', action='store_true', help='show the documentation message and exit')
    parser.add_argument('--tests',        '-t', action='store_true', help='execute diagnostic tests prior to starting')
    parser.add_argument('--configure',    '-c', action='store_true', help='run configuration (included by -s)')
    parser.add_argument('--start',        '-s', action='store_true', help='start kros')
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
            return args

    except NotImplementedError as nie:
        _log.error('unrecognised log level \'{}\': {}'.format(args.level, nie))
        _log.error('exit on error.')
#       sys.exit(1)
    except Exception as e:
        _log.error('error parsing command line arguments: {}\n{}'.format(e, traceback.format_exc()))
        _log.error('exit on error.')
#       sys.exit(1)

# main ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

REPORT_REMAINING_FRAMES = True # debugging

def main(argv):

    # path to diagnostics script
    DIAGNOSTICS_SCRIPT = "./diagnostics.py"

    _kros = None
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
            _execute_diagnostics = _args.tests
            if _execute_diagnostics:
                # execute the diagnostics and check the return code
                # the 'check=True' flag raises a CalledProcessError if the script fails
                _log.info("executing diagnostics…")
                subprocess.run([sys.executable, DIAGNOSTICS_SCRIPT, '--run-tests'], check=True)
                _log.info("all pre-startup diagnostics successful, proceeding with KROS…")
            else:
                _log.info("diagnostics inactive.")
            # otherwise proceed
            _kros = KROS(level=_level)
            if _args.configure or _args.start:
                _kros.configure(_args)
                if not _args.start:
                    _log.info('configure only: ' + Fore.YELLOW + 'specify the -s argument to start kros.')
            if _args.start:
                _counter = itertools.count()
                if _kros.should_await_pushbutton():
                    while _kros.should_await_pushbutton():
                        if next(_counter) % 20 == 0:
                            _log.info(Fore.YELLOW + 'waiting for pushbutton…')
                        time.sleep(0.1)
                    match _kros.state:
                        case State.INITIAL: # expected state
                            _kros.start()
                            # does not return here til we close down
                        case State.CLOSED:
                            _log.info(Fore.WHITE + 'closed before starting.')
                        case _:
                            _log.warning('closed with invalid state: {}'.format(_kros.state))
                else:
                    _kros.start()
                    # kros is now running…
    except KeyboardInterrupt:
        print('\n')
        print(Fore.MAGENTA + Style.BRIGHT + 'caught Ctrl-C; exiting…' + Style.RESET_ALL)
    except RuntimeError as rte:
        _log.error('runtime error starting kros: {}'.format(rte))
    except subprocess.CalledProcessError as e:
        _log.error("pre-startup diagnostics failed with error: {}".format(e))
        _log.error("aborting KROS startup.")
#       sys.exit(1)
    except Exception:
        _log.error('error starting kros: {}'.format(traceback.format_exc()))
    finally:
        Logger.close_data_handler()
        if _kros and not _kros.closed:
            _kros.close()

if __name__== "__main__":
    main(sys.argv[1:])

#EOF
