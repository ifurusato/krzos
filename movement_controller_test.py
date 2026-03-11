#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-20
# modified: 2025-11-20
#
# Test script for MovementController - drives an 80cm square pattern.

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

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
_log = Logger('test', Level.INFO)

_odometer            = None
_motor_controller    = None
_rotation_controller = None
_movement_controller = None

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def wait_for_movement_complete(movement_ctrl, poll_rate_hz=20):
    '''
    Poll movement controller until linear movement completes and returns to IDLE.
    Does NOT handle rotation phase.
    '''
    poll_delay = 1.0 / poll_rate_hz
    while movement_ctrl.movement_phase in (MovementPhase.ACCEL, MovementPhase.MOVE, MovementPhase.DECEL):
        current_time, elapsed, accumulated = movement_ctrl.poll()
        phase = movement_ctrl.movement_phase
        if phase == MovementPhase.ACCEL:
            movement_ctrl.handle_accel_phase(elapsed, accumulated, current_time)
        elif phase == MovementPhase.MOVE:
            movement_ctrl.handle_move_phase(accumulated, current_time)
        elif phase == MovementPhase.DECEL:
            movement_ctrl.handle_decel_phase(elapsed, accumulated)
        time.sleep(poll_delay)
    while not _motor_controller.all_motors_are_stopped:
        time.sleep(0.05)

def x_wait_for_movement_complete(movement_ctrl, poll_rate_hz=20):
    '''
    Poll movement controller until linear movement completes and returns to IDLE.
    Does NOT handle rotation phase.
    '''
    poll_delay = 1.0 / poll_rate_hz

    _enable_wait_loop = True
    def wait_enable():
        nonlocal _enable_wait_loop
#       print(Fore.MAGENTA + 'WAIT ENABLED CALLED.' + Style.RESET_ALL)
        _enable_wait_loop = False
    
    while movement_ctrl.movement_phase in (MovementPhase.ACCEL, MovementPhase.MOVE, MovementPhase.DECEL):
        current_time, elapsed, accumulated = movement_ctrl.poll()
        phase = movement_ctrl.movement_phase
        if phase == MovementPhase.ACCEL:
#           print(Fore.GREEN + 'ACCEL' + Style.RESET_ALL)
            movement_ctrl.handle_accel_phase(elapsed, accumulated, current_time)
            _motor_controller.add_state_change_callback(lambda: wait_enable())
        elif phase == MovementPhase.MOVE:
#           print(Fore.YELLOW + 'MOVE' + Style.RESET_ALL)
            movement_ctrl.handle_move_phase(accumulated, current_time)
        elif phase == MovementPhase.DECEL:
#           print(Fore.RED + 'DECEL' + Style.RESET_ALL)
            movement_ctrl.handle_decel_phase(elapsed, accumulated)
#       current_time, elapsed, accumulated = movement_ctrl.poll()
#       print('acc={:.1f}mm phase={}'.format(accumulated, movement_ctrl.movement_phase.name))
        time.sleep(poll_delay)
    while _enable_wait_loop:
        print('waiting to stop... ')
        time.sleep(0.1)
    print('END ')
    time.sleep(5)

def wait_for_rotation_complete(movement_ctrl, rotation_ctrl, poll_rate_hz=20):
    '''
    Poll rotation controller until rotation completes.
    Also monitors movement controller to detect when it returns to IDLE.
    '''
    poll_delay = 1.0 / poll_rate_hz
    
    while rotation_ctrl.is_rotating:
        current_time, elapsed, accumulated = rotation_ctrl.poll()
        phase = rotation_ctrl.rotation_phase
        
        if phase == RotationPhase.ACCEL:
            rotation_ctrl.handle_accel_phase(elapsed, accumulated, current_time)
        elif phase == RotationPhase.ROTATE:
            rotation_ctrl.handle_rotate_phase(accumulated, current_time)
        elif phase == RotationPhase.DECEL:
            rotation_ctrl.handle_decel_phase(elapsed, accumulated)
        
        time.sleep(poll_delay)
    
    # Wait for movement controller to transition back to IDLE
    while movement_ctrl.movement_phase == MovementPhase.ROTATING:
        movement_ctrl.poll()  # This checks rotation completion and transitions to IDLE
        time.sleep(poll_delay)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def drive_2m_forward(movement_ctrl, distance=2000.0):

    print('move ahead 2m...')
    # STEP 1: Move ahead 2m
    _log.info(Fore.YELLOW + 'moving ahead {}cm...'.format(distance))
    movement_ctrl.move(distance, Direction.AHEAD)
    wait_for_movement_complete(movement_ctrl)
    _log.info(Fore.GREEN + 'movement complete, robot STOPPED.')

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

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
        _odometer = _motor_controller.odometer
        _rotation_controller.enable()
        _movement_controller.enable()
        
        _log.info(Fore.GREEN + Style.BRIGHT + 'All controllers ready!\n')
        
        # wait a moment before starting
#       time.sleep(2.0)
        
        _movement_controller.add_phase_change_callback(phase_changed)

        drive_2m_forward(_movement_controller, distance=2000.0)

        print_pose()
        
        _odometer.print_pose(force=True, title='CYAN final pose')
        print(Fore.CYAN + '🔹 ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈' + Style.RESET_ALL)

        mc = _motor_controller
        print('🔹 final step counts: pfwd={}, sfwd={}, paft={}, saft={}'.format(
            mc.get_motor(Orientation.PFWD).steps,
            mc.get_motor(Orientation.SFWD).steps,
            mc.get_motor(Orientation.PAFT).steps,
            mc.get_motor(Orientation.SAFT).steps))


        _odometer.console('CYAN complete.')
        # wait before shutdown
        _log.info('\nWaiting 1 second before shutdown…')
        time.sleep(1.0)
        
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
