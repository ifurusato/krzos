#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-20
# modified: 2026-03-12

import sys, time, traceback
from math import degrees
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.config_loader import ConfigLoader
from core.direction import Direction
from core.orientation import Orientation
from core.rotation import Rotation
from hardware.motor_controller import MotorController
from hardware.rotation_controller import RotationController
from hardware.movement_controller import MovementController, MovementPhase
from hardware.rotation_controller import RotationPhase

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('test', Level.INFO)

_odometer            = None
_motor_controller    = None
_rotation_controller = None
_movement_controller = None

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def phase_changed(*args, **kwargs):
    prior_phase = args[0]
    new_phase   = args[1]
    print(Fore.CYAN + '🔹 phase: ' + Fore.GREEN + 'prior: {}; new: {}; '.format(prior_phase.name, new_phase.name) + Style.RESET_ALL)
    print_pose()

def print_pose():
    global _odometer
    pose  = _odometer.pose
    x     = pose[0]
    y     = pose[1]
    theta = degrees(pose[2])
    print(Fore.CYAN + '🔹 pose: ' + Fore.GREEN + 'x: {:.2f}mm; y: {:.2f}mm; theta: {:.1f}°'.format(x, y, theta) + Style.RESET_ALL)

def main():
    global _odometer, _motor_controller, _rotation_controller, _movement_controller

    try:

        time.sleep(3)

        _log.info('loading configuration…')
        _loader = ConfigLoader(Level.INFO)
        _config = _loader.configure()

        _log.info('creating motor controller…')
        _motor_controller = MotorController(_config, level=Level.INFO)

        _log.info('creating rotation controller…')
        _rotation_controller = RotationController(_config, _motor_controller, level=Level.INFO)

        _log.info('creating movement controller…')
        _movement_controller = MovementController(_config, _motor_controller, _rotation_controller, level=Level.INFO)

        _log.info('enabling controllers…')
        _motor_controller.enable()
        _odometer = _motor_controller.odometer
        _rotation_controller.enable()
        _movement_controller.enable()

        _log.info(Fore.GREEN + Style.BRIGHT + 'all controllers ready.\n')

        _movement_controller.add_phase_change_callback(phase_changed)

        _log.info(Fore.YELLOW + 'moving ahead 2000mm…')

#       _movement_controller.run(2000.0, Direction.AHEAD)
#       _movement_controller.run(2000.0, Direction.STBD)
#       _movement_controller.run(360.0,  Direction.CLOCKWISE)

        _movement_controller.run_displacement(1030.0, 1460.0, degrees=0.0)
#       _movement_controller.run_displacement(1030.0, 0.0, degrees=0.0)
#       _movement_controller.run_displacement(0.0, 1460.0, degrees=0.0)

        _log.info(Fore.GREEN + 'movement complete.')

        print_pose()
        _odometer.print_pose(force=True, title='CYAN final pose')
        print(Fore.CYAN + '🔹 ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈' + Style.RESET_ALL)

        mc = _motor_controller
        print('🔹 final step counts: pfwd={}, sfwd={}, paft={}, saft={}'.format(
            mc.get_motor(Orientation.PFWD).steps,
            mc.get_motor(Orientation.SFWD).steps,
            mc.get_motor(Orientation.PAFT).steps,
            mc.get_motor(Orientation.SAFT).steps))

        _odometer.console('CYAN complete.')
        _log.info('waiting 1 second before shutdown…')
        time.sleep(1.0)

    except KeyboardInterrupt:
        _log.info('Ctrl-C caught, exiting…')
    except Exception as e:
        _log.error('error in main: {}\n{}'.format(e, traceback.format_exc()))
    finally:
        _log.info('cleaning up…')
        if _movement_controller:
            _log.info('disabling movement controller…')
            _movement_controller.disable()
            _movement_controller.close()
        if _rotation_controller:
            _log.info('disabling rotation controller…')
            _rotation_controller.disable()
            _rotation_controller.close()
        if _motor_controller:
            _log.info('disabling motor controller…')
            _motor_controller.disable()
            _motor_controller.close()
        _log.info(Fore.GREEN + 'test complete.')

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

if __name__ == "__main__":
    main()

#EOF
