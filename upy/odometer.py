#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-12-09
# modified: 2025-12-09

import time
from machine import Pin
from logger import Logger, Level
#from pmw3901_rp2040 import create_paa5100

class Odometer:
    RESOLUTION_FACTOR = 0.009734
    '''
    MicroPython Odometer that computes velocity and position using a PAA5100JE
    near optical flow sensor.

    Supports both polling mode and interrupt-driven mode for efficiency.

    Measures translation only - rotation must come from encoders or IMU.

    Modes:
    - Polling mode:  Call update() regularly in your main loop
    - IRQ mode:      Attach IRQ pin, updates happen automatically on motion events

    - velocity:      Returns (vx, vy) where vx = lateral cm/s, vy = forward cm/s
    - position:      Returns (x, y) where x = lateral cm, y = forward cm (body frame)
    - reset():       Resets cumulative position and timing
    - calibrate():   Adjusts resolution factor based on known vs measured distance
    '''
    def __init__(self, sensor=None, irq_pin=None, resolution_factor=None, timeout=1.0):
        '''
        Initialize the optical flow odometer.

        Args:
            sensor:             PAA5100 instance (optional or already configured with correct orientation)
            irq_pin:            Optional Pin instance for interrupt-driven updates
            resolution_factor:  Scaling factor for sensor readings (adjust via calibration)
            timeout:            Sensor read timeout in seconds
        '''
        self._log = Logger('odometer', level=Level.INFO)
        if sensor:
            self._nofs = sensor
            self._irq_pin = irq_pin
        else:
            # use defaults for the RP2040
            from pmw3901_rp2040 import create_paa5100

            self._nofs, self._irq_pin = create_paa5100()
            self._nofs.set_orientation(invert_x=True, invert_y=True, swap_xy=False)
        self.set_resolution_factor(resolution_factor)
        self._timeout = timeout
        # internal state
        self._last_time    = None  # float (seconds from time.ticks_ms())
        self._x            = 0.0   # lateral position (cm) - positive = starboard
        self._y            = 0.0   # forward position (cm) - positive = forward
        self._vx           = 0.0   # lateral velocity (cm/s)
        self._vy           = 0.0   # forward velocity (cm/s)
        # calibration tracking
        self._cumulative_x = 0.0
        self._cumulative_y = 0.0
        self._callbacks    = []
        self._enabled      = True
        self._closed       = False
        # IRQ setup
        self._irq_mode = False
        if irq_pin is not None:
            self._setup_irq(irq_pin)
        self._log.info("mode: {}".format("IRQ-driven" if self._irq_mode else "polling"))
        self._log.info('ready.')

    def _setup_irq(self, irq_pin):
        '''
        Set up interrupt-driven mode.
        '''
        try:
            # Attach IRQ handler to pin (rising edge = motion detected)
            irq_pin.irq(trigger=Pin.IRQ_RISING, handler=self._irq_handler)
            self._irq_mode = True
            self._log.info("  IRQ attached to pin {}".format(irq_pin))
        except Exception as e:
            self._log.info("  Warning: Could not setup IRQ:  {}".format(e))
            self._log.info("  Falling back to polling mode.")
            self._irq_mode = False

    def _irq_handler(self, pin):
        '''
        IRQ callback - called when sensor detects motion.
        '''
        if self._enabled and not self._closed:
            # Call update from IRQ context
            self.update()

    @property
    def enabled(self):
        return self._enabled

    @property
    def closed(self):
        return self._closed

    @property
    def irq_mode(self):
        '''
        Returns True if using interrupt-driven updates.
        '''
        return self._irq_mode

    @property
    def position(self):
        '''
        Returns (x, y): x (lateral cm), y (forward cm) in body frame.
        '''
        return self._x, self._y

    @property
    def velocity(self):
        '''
        Returns (vx, vy): vx (lateral cm/s), vy (forward cm/s) in body frame.
        '''
        return self._vx, self._vy

    def set_position(self, x, y):
        '''
        Set the robot's position to specific coordinates.
        Used for initialization or resetting odometry.

        Args:
            x:  lateral position in cm
            y: forward position in cm
        '''
        self._x = x
        self._y = y
        self._log.info("position set to:  x={:.2f}cm, y={:.2f}cm".format(x, y))

    def reset(self):
        '''
        Resets the odometer cumulative position and timing state.
        '''
        self._last_time = None
        self._x = 0.0
        self._y = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._cumulative_x = 0.0
        self._cumulative_y = 0.0
        self._log.info("reset.")

    def update(self):
        '''
        Read sensor and update velocity and position.

        In polling mode:  Call this at regular intervals (e.g., 10-50Hz) in your main loop.
        In IRQ mode: Called automatically when sensor detects motion (you can still call manually).

        Automatically handles timing and integration.
        '''
        if not self._enabled:
            return
        current_time = time.ticks_ms() / 1000.0  # convert to seconds
        try:
            # read sensor - returns (x, y) in sensor units
            # with your orientation:  x = lateral, y = forward
            raw_x, raw_y = self._nofs.get_motion(timeout=self._timeout)
            # apply resolution factor to convert sensor units to cm
            dx_cm = raw_x * self._resolution_factor
            dy_cm = raw_y * self._resolution_factor
            if self._last_time is not None:
                dt = current_time - self._last_time
                if dt > 0.0:
                    # calculate velocities in body frame
                    self._vx = dx_cm / dt  # lateral velocity
                    self._vy = dy_cm / dt  # forward velocity
                    # integrate position in body frame
                    self._x += dx_cm
                    self._y += dy_cm
                    # track cumulative movement for calibration
                    self._cumulative_x += abs(dx_cm)
                    self._cumulative_y += abs(dy_cm)
                    # trigger callbacks if moving
                    if self.is_moving():
                        self._trigger_callbacks()
            self._last_time = current_time
        except RuntimeError:
            # sensor timeout or no motion detected
            # set velocities to zero but don't update position
            self._vx = 0.0
            self._vy = 0.0
            # keep last_time to maintain timing continuity

    def calibrate(self, known_distance_cm, measured_distance_cm=None):
        '''
        Calibrate the resolution factor based on a known movement.

        Usage:
        1. Call reset() to zero odometry
        2. Move robot a known distance (e.g., 100cm forward)
        3. Call calibrate(known_distance_cm=100.0)

        Or manually specify measured distance:
            calibrate(known_distance_cm=100.0, measured_distance_cm=98.5)

        Args:
            known_distance_cm:     Actual distance traveled (ground truth)
            measured_distance_cm:  Distance measured by sensor (if None, uses current cumulative)
        '''
        self._log.debug("cumulative_x={:.2f}, cumulative_y={:.2f}".format(self._cumulative_x, self._cumulative_y))
        if measured_distance_cm is None:
            # use cumulative distance traveled
            measured_distance_cm = (self._cumulative_x**2 + self._cumulative_y**2)**0.5
            self._log.debug("calculated measured_distance={:.2f}cm".format(measured_distance_cm))
        if measured_distance_cm == 0.0:
            self._log.error("no movement detected for calibration.")
            return
        # calculate correction factor
        correction = known_distance_cm / measured_distance_cm
        old_factor = self._resolution_factor
        self._log.debug("correction factor = {:.2f} / {:.2f} = {:.6f}".format(
            known_distance_cm, measured_distance_cm, correction))
        self._resolution_factor *= correction
        self._log.info("\ncalibration complete:")
        self._log.info("  known distance:       {:.2f}cm".format(known_distance_cm))
        self._log.info("  measured distance:    {:.2f}cm".format(measured_distance_cm))
        self._log.info("  old factor:           {:.6f}".format(old_factor))
        self._log.info("  new factor:           {:.6f}".format(self._resolution_factor))
        self._log.info("  correction applied:   {:.4f}x".format(correction))
        if abs(correction - 1.0) < 0.01:
            self._log.warning("correction is very close to 1.0 - no change needed!")

    def get_resolution_factor(self):
        '''
        Returns the current resolution factor for saving to config.
        '''
        return self._resolution_factor

    def set_resolution_factor(self, resolution_factor):
        '''
        Set the resolution factor manually. If the argument is None, sets to the hard-coded default.
        '''
        self._resolution_factor = resolution_factor if resolution_factor is not None else Odometer.RESOLUTION_FACTOR
        self._log.info("resolution factor: {:.6f}".format(self._resolution_factor))

    def set_sensor_led(self, enable):
        '''
        Turns the NOFS's illumination on or off.
        '''
        if enable:
            self._nofs.enable_sensor_led()
        else:
            self._nofs.disable_sensor_led()

    def is_moving(self, threshold=0.1):
        '''
        Returns True if velocity magnitude exceeds threshold.

        Args:
            threshold:  Minimum velocity in cm/s to consider "moving"
        '''
        velocity_magnitude = (self._vx**2 + self._vy**2)**0.5
        return velocity_magnitude > threshold

    def get_distance_traveled(self):
        '''
        Returns total distance traveled (magnitude) since last reset.
        '''
        return (self._x**2 + self._y**2)**0.5

    def get_cumulative_distance(self):
        '''
        Returns cumulative distance (sum of all movements, not straight-line).
        Useful for calibration.
        '''
        return (self._cumulative_x**2 + self._cumulative_y**2)**0.5

    def print_info(self):
        '''
        Prints the current velocity vector and position.
        '''
        self._log.info("velocity: ({:.2f}, {:.2f}); position: ({:.2f}cm, {:.2f}cm)".format(
            self._vx, self._vy, self._x, self._y))

    def add_callback(self, callback):
        '''
        Adds a callback to be triggered when the robot is moving.

        Args:
            callback: Function to call (no arguments)
        '''
        if not callable(callback):
            raise ValueError("callback must be callable")
        if callback not in self._callbacks:
            self._callbacks.append(callback)
            self._log.info("added callback: {}".format(callback))

    def remove_callback(self, callback):
        '''
        Removes a callback from the list.
        '''
        if callback in self._callbacks:
            self._callbacks.remove(callback)

    def _trigger_callbacks(self):
        '''
        Execute all registered callbacks.
        '''
        for callback in self._callbacks:
            try:
                callback()
            except Exception as e:
                self._log.error("{} raised in callback: {}".format(type(e), e))

    def enable(self):
        '''
        Enable the odometer.
        '''
        if not self._enabled:
            self._enabled = True
            self._log.info("enabled.")
        else:
            self._log.warning("already enabled.")

    def disable(self):
        '''
        Disable the odometer.
        '''
        if self._enabled:
            self._enabled = False
            self._log.info("disabled.")
        else:
            self._log.warning("disabled.")

    def close(self):
        '''
        Close the odometer and clean up.
        '''
        if not self._closed:
            self._closed = True
            self._enabled = False
            # Detach IRQ if present
            if self._irq_pin is not None:
                try:
                    self._irq_pin.irq(handler=None)
                except:
                    pass
            self._log.info("closed.")
        else:
            self._log.warning("already closed.")

#EOF
