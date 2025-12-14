#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-07
# modified: 2025-11-07
#
# Tests the RotationController independently.
#

import sys
import time
import asyncio
import traceback
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from core.component import Component
from core.rotation import Rotation
from hardware.i2c_scanner import I2CScanner, DeviceNotFound
from hardware.motor_controller import MotorController
from hardware.rotation_controller import RotationController, RotationPhase

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
_log = Logger('test', Level.INFO)
_component_registry = Component.get_registry()

# globals for shutdown
_shutting_down = False
_motor_controller = None
_rotation_controller = None
_irq_clock = None

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def shutdown():
    global _shutting_down, _motor_controller, _rotation_controller, _irq_clock
    try:
        _log.info(Fore.WHITE + 'shutting down!')
        if _shutting_down:
            return  # not reentrant
        _shutting_down = True
        
        # close RotationController before MotorController
        if _rotation_controller and not _rotation_controller.closed:
            _log.debug('closing rotation controller (before motor controller)…')
            _rotation_controller.close()
        if _motor_controller and not _motor_controller.closed:
            _log.debug('closing motor controller…')
            _motor_controller.close()
        # close any remaining components
        clean_close()
        
        if _irq_clock and not _irq_clock.closed:
            _irq_clock.close()
        
#       _component_registry.print_registry()
    finally:
        pass
    _log.info('shutdown complete.')
    sys.exit(0)

def clean_close():
    # closes all components that are not already closed
    for _name in _component_registry.names:
        if _name in ['rotation-controller', 'motor-ctrl']:
            continue  # already closed explicitly
        _component = _component_registry.get(_name)
        if _component is not None and not _component.closed:
            _log.debug('closing component \'{}\' ({})…'.format(_component.name, _component.classname))
            _component.close()
    time.sleep(0.1)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
async def run_rotation_test():
    '''
    Main test sequence that performs exact rotation tests.
    '''
    global _motor_controller, _rotation_controller, _irq_clock
    
    _log.info(Fore.CYAN + Style.BRIGHT + '\n' + '='*80)
    _log.info(Fore.CYAN + 'ROTATION CONTROLLER TEST')
    _log.info(Fore.CYAN + '='*80 + '\n')
    
    try:
        # load configuration
        _loader = ConfigLoader(Level.INFO)
        _config = _loader.configure()
        
        # create motor controller
        _log.info('creating motor controller…')
        _motor_controller = MotorController(_config, level=Level.INFO)
        _motor_controller.enable()
        
        # create rotation controller
        _log.info('creating rotation controller…')
        _rotation_controller = RotationController(_config, _motor_controller, level=Level.INFO)
        _rotation_controller.enable()
        
        _log.info(Fore.GREEN + 'setup complete\n')
        
        # wait for user
        input(Fore.YELLOW + 'Press ENTER to begin 360° clockwise rotation test...')
        
        # test rotation
        _degrees = 360.0
        _rotation_controller.rotate(_degrees, Rotation.COUNTER_CLOCKWISE)
        
        # poll loop
        _count = 0
        while _rotation_controller.is_rotating:
            current_time, elapsed, accumulated_rotation = _rotation_controller.poll()
            
            # handle phases
            if _rotation_controller.rotation_phase == RotationPhase.ACCEL:
                _rotation_controller.handle_accel_phase(elapsed, accumulated_rotation, current_time)
                
            elif _rotation_controller.rotation_phase == RotationPhase.ROTATE:
                rotate_complete = _rotation_controller.handle_rotate_phase(accumulated_rotation, current_time)
                
                # display progress
                if _count % 20 == 0:
                    _log.info('🔄 rotating at {:.1f}°'.format(accumulated_rotation))
                
            elif _rotation_controller.rotation_phase == RotationPhase.DECEL:
                decel_complete = _rotation_controller.handle_decel_phase(elapsed, accumulated_rotation)
                if decel_complete:
                    _log.info(Fore.GREEN + 'rotation complete!')
                    break
            
            _count += 1
            await asyncio.sleep(0.05)  # 20Hz polling
        
        _log.info(Fore.GREEN + Style.BRIGHT + '\n' + '='*80)
        _log.info(Fore.GREEN + 'TEST COMPLETE')
        _log.info(Fore.GREEN + 'Final cumulative heading: {:.1f}°'.format(
            _rotation_controller.get_current_heading()))
        _log.info(Fore.GREEN + '='*80 + '\n')
        
    except KeyboardInterrupt:
        _log.info(Fore.YELLOW + '\nInterrupted by user')
    except Exception as e:
        _log.error('Error during test: {}\n{}'.format(e, traceback.format_exc()))
    finally:
        shutdown()

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
if __name__ == '__main__':
    asyncio.run(run_rotation_test())

#EOF
