#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Test script for rotating the robot to sequential headings using HeadingTask,
# with dynamic IMU trim (via DigitalPotentiometer),
# and manual intervention via a hardware button to end each movement.
#
# Copyright 2025 by Murray Altheim. All rights reserved.
#
# author:   Murray Altheim
# created:  2025-10-01
# modified: 2025-10-02

import sys
import time
import traceback
from colorama import init, Fore, Style
init()

from core.orientation import Orientation
from core.rate import Rate
from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.motor_controller import MotorController
from hardware.digital_pot import DigitalPotentiometer
from matrix11x7 import Matrix11x7
from hardware.em7180 import Em7180
from hardware.heading_task import HeadingTask
from hardware.button import Button
from hardware.irq_clock import IrqClock

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

FIXED_TRIM = 0

_log = Logger('test-heading-task', Level.INFO)

# global references
_motor_controller = None
_em7180 = None
_heading_task = None
_level = Level.INFO
_button = None
_irq_clock = None

def disable_heading_task_callback():
    global _heading_task
    if _heading_task and _heading_task.enabled:
        _log.info(Fore.YELLOW + "Button pressed: Disabling current HeadingTask.")
        _heading_task.close()
        time.sleep(0.2) # crude debounce

def set_target_heading(target_heading, persistent=True,
                      kp=0.02, deadband=2.0, hysteresis=4.0, max_speed=0.4,
                      settle_cycles=6):
    '''
    Starts a HeadingTask to rotate to the target_heading.
    If persistent is False, the task will auto-disable after settling.
    If persistent is True, manual intervention (button) is needed to end the task.
    '''
    global _heading_task, _motor_controller, _em7180, _level, _irq_clock, _config
    # Stop and cleanup previous heading_task, if any
    if _heading_task and _heading_task.enabled:
        _log.info(Fore.YELLOW + "Disabling previous HeadingTask before setting new target.")
        _heading_task.disable()
        time.sleep(0.5)
    _log.info(Fore.CYAN + f"Setting target heading to {target_heading}° (persistent={persistent})...")
    _heading_task = HeadingTask(
        _config,
        _motor_controller,
        _irq_clock,
        _em7180,
        target_heading,
        kp=kp,
        deadband=deadband,
        hysteresis=hysteresis,
        max_speed=max_speed,
        settle_cycles=settle_cycles,
        persistent=persistent,
        level=_level
    )
    _heading_task.enable()

def show_heading_live(imu, rate=None):
    '''
    Display live heading, corrected_yaw, and trim info for the current HeadingTask.
    '''
    while _heading_task and _heading_task.enabled and not _heading_task.closed:
        imu.poll()
        _yaw = imu.yaw
        _corrected_yaw = imu.corrected_yaw
        _yaw_trim = imu.yaw_trim
        _log.info('Yaw: {:+6.2f} '.format(_yaw)
                  + Fore.WHITE + 'Corrected Yaw: {:+6.2f} '.format(_corrected_yaw)
                  + Style.DIM + 'with trim: {:+6.2f}'.format(_yaw_trim))
        if rate:
            rate.wait()
        else:
            time.sleep(0.05)

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

if __name__ == "__main__":
    try:
        _level = Level.INFO
        _config = ConfigLoader(_level).configure()

        # Digital Potentiometer as IMU trim
        _trim_pot = DigitalPotentiometer(_config, level=_level)
        _trim_pot.set_output_range(-180.0, 180.0)

        # LED matrix for heading display
        _matrix11x7 = Matrix11x7()
        _matrix11x7.set_brightness(0.25)

        # IMU with dynamic trim
        _em7180 = Em7180(_config, matrix11x7=_matrix11x7, trim_pot=_trim_pot, level=_level)
        if FIXED_TRIM != 0:
            _em7180.set_fixed_yaw_trim(FIXED_TRIM)
        _em7180.set_verbose(False)

        # External IRQ clock for PID timing
        _log.info('creating IRQ clock…')
        _irq_clock = IrqClock(_config, level=Level.INFO)
        _irq_clock.enable()

        # hardware button setup for manual HeadingTask disable
        _button = Button(config=_config, level=_level)
        _button.add_callback(disable_heading_task_callback)
        _log.info(Fore.CYAN + "Hardware button ready for HeadingTask disable.")

        # Motor Controller
        _motor_controller = MotorController(_config, external_clock=_irq_clock, level=_level)
        _motor_controller.enable()

        _rate = Rate(2, Level.ERROR) # 2Hz

        # Example usage: head North, then East, then West, then South
        headings = [0, 90, 180, 270]

        for hdg in headings:
            set_target_heading(hdg, persistent=True) # use persistent mode -- button disables after settling
            _log.info(Fore.CYAN + f"Moving to heading {hdg}°. Press button to stop and proceed to next.")
            show_heading_live(_em7180, rate=_rate)
            time.sleep(1.0)

        _log.info(Fore.GREEN + "HeadingTask sequence completed.")

    except KeyboardInterrupt:
        _log.info('Ctrl-C caught; disabling HeadingTask and exiting…')
        try:
            if _heading_task:
                _heading_task.disable()
        except Exception:
            pass
    except Exception as e:
        _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
    finally:
        if _motor_controller:
            _motor_controller.stop()
            _motor_controller.close()
        if _trim_pot:
            _trim_pot.close()
        if _em7180:
            _em7180.close()
        if _irq_clock:
            _irq_clock.disable()

#EOF
