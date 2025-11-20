#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-20
# modified: 2025-11-20
#
# Test script for MovementController - drives a 100cm square pattern.

import sys, time, traceback
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.config_loader import ConfigLoader
from core.direction import Direction
from core.rotation import Rotation
from hardware.motor_controller import MotorController
from hardware.rotation_controller import RotationController
from hardware.movement_controller import MovementController, MovementPhase

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
_log = Logger('test', Level.INFO)

_motor_controller    = None
_rotation_controller = None
_movement_controller = None

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def wait_for_movement_complete(movement_ctrl, poll_rate_hz=20):
    '''
    Poll the movement controller until movement is complete.
    Handles all phases: accel, move, decel.
    
    Args:
        movement_ctrl: MovementController instance
        poll_rate_hz: Polling frequency in Hz
    '''
    poll_delay = 1.0 / poll_rate_hz
    
    while movement_ctrl.is_moving:
        current_time, elapsed, accumulated = movement_ctrl.poll()
        phase = movement_ctrl.movement_phase
        
        if phase == MovementPhase.ACCEL:
            movement_ctrl.handle_accel_phase(elapsed, accumulated, current_time)
        elif phase == MovementPhase.MOVE:
            movement_ctrl.handle_move_phase(accumulated, current_time)
        elif phase == MovementPhase.DECEL:
            if movement_ctrl.handle_decel_phase(elapsed, accumulated):
                break  # deceleration complete
        
        time.sleep(poll_delay)

def wait_for_rotation_complete(rotation_ctrl, poll_rate_hz=20):
    '''
    Poll the rotation controller until rotation is complete.
    Handles all phases: accel, rotate, decel.
    
    Args:
        rotation_ctrl: RotationController instance
        poll_rate_hz: Polling frequency in Hz
    '''
    poll_delay = 1.0 / poll_rate_hz
    
    while rotation_ctrl.is_rotating:
        current_time, elapsed, accumulated = rotation_ctrl.poll()
        phase = rotation_ctrl.rotation_phase
        
        if phase.name == 'accel':
            rotation_ctrl.handle_accel_phase(elapsed, accumulated, current_time)
        elif phase.name == 'rotate':
            rotation_ctrl.handle_rotate_phase(accumulated, current_time)
        elif phase.name == 'decel':
            if rotation_ctrl.handle_decel_phase(elapsed, accumulated):
                break  # deceleration complete
        
        time.sleep(poll_delay)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def drive_square(movement_ctrl, side_length_cm=100.0):
    '''
    Drive a square pattern with the specified side length.
    
    Args:
        movement_ctrl: MovementController instance
        side_length_cm: Length of each side in centimeters
    '''
    _log.info(Fore.CYAN + Style.BRIGHT + '━' * 70)
    _log.info(Fore.CYAN + Style.BRIGHT + 'Starting {}cm square pattern'.format(side_length_cm))
    _log.info(Fore.CYAN + Style.BRIGHT + '━' * 70)
    
    for i in range(4):
        _log.info(Fore.GREEN + Style.BRIGHT + '\n▶ Side {} of 4'.format(i + 1))
        
        # Move forward
        _log.info(Fore.YELLOW + '  Moving forward {}cm...'.format(side_length_cm))
        movement_ctrl.move(side_length_cm, Direction.AHEAD)
        wait_for_movement_complete(movement_ctrl)
        _log.info(Fore.GREEN + '  ✓ Forward movement complete')
        
        # Pause briefly
        time.sleep(0.5)
        
        # Rotate 90° clockwise (if not the last side)
        if i < 3:
            _log.info(Fore.YELLOW + '  Rotating 90° clockwise...')
            movement_ctrl.rotate(90.0, Rotation.CLOCKWISE)
            
            # Wait for rotation via movement controller's ROTATING phase
            while movement_ctrl.movement_phase == MovementPhase.ROTATING:
                wait_for_rotation_complete(movement_ctrl._rotation_controller)
                time.sleep(0.05)
            
            _log.info(Fore.GREEN + '  ✓ Rotation complete')
            time.sleep(0.5)
    
    _log.info(Fore.CYAN + Style.BRIGHT + '\n━' * 70)
    _log.info(Fore.CYAN + Style.BRIGHT + 'Square pattern complete!')
    _log.info(Fore.CYAN + Style.BRIGHT + '━' * 70)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():
    global _motor_controller, _rotation_controller, _movement_controller
    
    try:
        # load configuration
        _log.info('loading configuration…')
        _loader = ConfigLoader(Level.INFO)
        _config = _loader.configure()
        # create motor controller
        _log.info('creating motor controller…')
        _motor_controller = MotorController(_config, level=Level.INFO)
        # create rotation controller
        _log.info('creating rotation controller…')
        _rotation_controller = RotationController(_config, _motor_controller, level=Level.INFO)
        # create movement controller
        _log.info('creating movement controller…')
        _movement_controller = MovementController(_config, _motor_controller, _rotation_controller, level=Level.INFO)
        # enable controllers
        _log.info('enabling controllers…')
        _motor_controller.enable()
        _rotation_controller.enable()
        _movement_controller.enable()
        
        _log.info(Fore.GREEN + Style.BRIGHT + 'All controllers ready!\n')
        
        # wait a moment before starting
        time.sleep(2.0)
        
        # execute square pattern
        drive_square(_movement_controller, side_length_cm=80.0)
        
        # wait before shutdown
        _log.info('\nWaiting 2 seconds before shutdown…')
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        _log.info('Ctrl-C caught, exiting…')
    except Exception as e:
        _log.error('error in main: {}\n{}'.format(e, traceback.format_exc()))
    finally:
        # cleanup
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
        
        _log.info(Fore.GREEN + 'Test complete.')

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
if __name__== "__main__":
    main()

#EOF
