
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2020-10-26
# modified: 2026-02-05

import time
import threading
from pmw3901 import BG_CS_FRONT_BCM, PAA5100, PMW3901
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

class OpticalFlowSensor(Component):
    '''
    A wrapper for the PAA5100JE or PMW3901 optical flow sensor, that uses a
    low-resolution camera and some clever algorithms to detect motion across
    surfaces. The PAA5100JE sensor has a range of 15-35mm, a frame rate of 
    242 FPS, FoV of 42°, and a maximum speed of 1.14 metres per second (with 
    sensor 25mm away from surface).

    By contrast, the PMW3901 has a range from 80mm to infinity, and is more
    suited for tall robots or flying drones. Currently the class has not been
    calibrated for the PMW3901.

    In configuration there are X and Y trim values as multipliers on the values
    read by the sensor.

    Connections:

    * 3-5V to any 5V or 3V pin
    * CS to BCM 7
    * SCK to BCM 11
    * MOSI to BCM 10
    * MISO to BCM 9
    * INT to BCM 19
    * GND to any ground pin

    Values are kept internally as floats, returned as ints.

    Note: This was calibrated on a bamboo cutting board, and on other surfaces
    the tracking will vary.

    :param config:          the application configuration
    :param level:           the log level
    '''
    def __init__(self, config, level=Level.INFO):
        self._log = Logger('ofs', level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._log.info('initialising ofs…')
        _cfg = config['kros'].get('hardware').get('ofs')
        _rotation    = _cfg.get('rotation') # options: 0, 90, 180, 270; rotation of sensor in degrees
        _slot        = BG_CS_FRONT_BCM # front SPI slot
        # the class for the specified breakout (PWM3901 or PAA5100)
        _near        = _cfg.get('near')
        SensorClass  = PAA5100 if _near else PMW3901
        self._log.info(Fore.GREEN + 'sensor class: {}'.format(SensorClass))
        self._nofs   = SensorClass(spi_port=0, spi_cs=_slot)
        self._nofs.set_rotation(_rotation)
        self._x_trim = _cfg.get('x_trim') # percentage X trim as a multiplier
        self._y_trim = _cfg.get('y_trim') # percentage Y trim as a multiplier
        self._x      = 0.0
        self._y      = 0.0
        self._tx     = 0.0
        self._ty     = 0.0
        self._spmm   = 6.43 # steps per millimeter, calibrated on a bamboo cutting board
        self._poll_interval  = 0.2   # interval to poll the sensor
        self._poll_thread    = None
        self._running        = False
        self._lock           = threading.Lock()
        self._log.info('ready.')

    def start(self):
        '''
        Start the polling thread.
        '''
        if self._running:
            self._log.warning('polling thread already running.')
            return
        self._log.info('starting polling thread…')
        self._running = True
        self._poll_thread = threading.Thread(target=self._poll_task_fn, daemon=True)
        self._poll_thread.start()

    def stop(self):
        '''
        Stop the polling thread.
        '''
        if not self._running:
            self._log.warning('polling thread not running.')
            return
        self._log.info('stopping polling thread…')
        self._running = False
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=2.0)
            self._poll_thread = None
        self._log.info('polling thread stopped.')

    def set_led(self, value):
        '''
        turn the white LEDs on or off.
        '''
        if value is True:
            time.sleep(0.2)
            self._nofs._write(0x7f, 0x14)
            self._nofs._write(0x6f, 0x1c) # DEC 28
            self._nofs._write(0x7f, 0x00)
        else:
            time.sleep(0.2)
            self._nofs._write(0x7f, 0x14)
            self._nofs._write(0x6f, 0x00) 
            self._nofs._write(0x7f, 0x00)

    def relative(self):
        '''
        return the relative positional change since the previous poll.
        '''
        if not self._running:
            self._poll_on_demand()
        with self._lock:
            return self._x, self._y

    def absolute(self):
        '''
        return the absolute (cumulative) positional change since the
        previous poll.
        '''
        if not self._running:
            self._poll_on_demand()
        with self._lock:
            return int(self._tx), int(self._ty)

    def millimeters(self):
        '''
        return the absolute (cumulative) positional change in millimeters
        since the previous poll.
        '''
        _abs_x, _abs_y = self.absolute()
        _dist_x = int(_abs_x / self._spmm)
        _dist_y = int(_abs_y / self._spmm)
        return _dist_x, _dist_y

    def x_variance(self):
        '''
        Return the lateral variance as a percentage of longitudinal movement.

        Shows the inverse of y_variance(): how much lateral drift (X) there 
        is relative to forward/backward movement (Y).

        In practical terms:

            y_variance() would be useful when you're trying to move in the 
                X direction and want to know how much you're drifting in Y.
            x_variance() would be useful when you're trying to move in the 
                Y direction (forward/backward) and want to know how much 
                you're drifting sideways in X.

        For a robot moving forward, x_variance() would tell you if it's veering 
        to port or starboard instead of going straight. For a robot strafing 
        sideways, y_variance() would tell you if it's drifting forward or aft 
        instead of going purely lateral.

        So whether you use x_variance() depends on whether you care about lateral 
        drift during forward/backward movement.
        '''
        with self._lock:
            return 0 if self._ty == 0 else int( self._tx / self._ty * 100.0 )


    def y_variance(self):
        '''
        The y_variance() method returns the ratio of _ty to _tx as a percentage.

        Specifically:

            If _tx is 0, it returns 0 (to avoid division by zero)
            Otherwise, it returns int(_ty / _tx * 100.0)

        So if the sensor has moved 100 units in X and 50 units in Y, it would 
        return 50 (meaning Y movement is 50% of X movement).

        If Y and X movement are equal, it returns 100. If there's more Y movement 
        than X, it returns a value greater than 100.
        '''
        with self._lock:
            return 0 if self._tx == 0 else int( self._ty / self._tx * 100.0 )

    def reset(self):
        with self._lock:
            self._tx = 0.0
            self._ty = 0.0

    def close(self):
        '''
        closes the sensor.
        '''
        self.set_led(False)
        self.stop()
        Component.close(self) # calls disable


    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _poll_task_fn(self):
        '''
        thread function to poll the sensor for motion data.
        '''
        while self._running:
            try:
                x, y = self._nofs.get_motion()
                with self._lock:
                    self._x = x * self._x_trim
                    self._y = y * self._y_trim
                    self._tx += self._x
                    self._ty += self._y
            except Exception as e:
                self._log.error('error polling sensor: {}'.format(e))
            time.sleep(self._poll_interval)

    def _poll_on_demand(self):
        '''
        poll the sensor once and update values.
        '''
        try:
            x, y = self._nofs.get_motion()
            with self._lock:
                self._x = x * self._x_trim
                self._y = y * self._y_trim
                self._tx += self._x
                self._ty += self._y
        except Exception as e:
            self._log.error('error polling sensor: {}'.format(e))

#EOF
