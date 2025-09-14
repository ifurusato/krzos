#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-22
# modified: 2025-06-22
#
# Just stops all the motors.
#

from logger import Logger, Level
from config_loader import ConfigLoader
from motor_controller import MotorController

def main():
    motor_ctrl = None
    try:
        _config = ConfigLoader.configure('config.yaml')
        if _config is None:
            raise ValueError('failed to import configuration.')
        motor_ctrl = MotorController(config=_config, motors_enabled=(True, True, False, False), level=Level.INFO)
        motor_ctrl.stop()
        print("all motors stopped.")
    finally:
        if motor_ctrl:
            motor_ctrl.close()
        print("complete.")

if __name__ == "__main__":
    main()

#EOF
