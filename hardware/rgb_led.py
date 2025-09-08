#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-08-25
# modified: 2025-08-26

import RPi.GPIO as GPIO
from hardware.color import Color

class RGBLED(object):
    '''
    Controls an RGB LED by managing its color and brightness. This is specifically
    designed for the GPIO pins used by the RGB LED on the Pimoroni Display HAT Mini.

    :param brightness_percentage: A float from 0.0 to 100.0 representing the brightness.
    '''
    LED_R = 17
    LED_G = 27
    LED_B = 22
    def __init__(self, brightness_percentage=100.0):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.LED_R, GPIO.OUT)
        GPIO.setup(self.LED_G, GPIO.OUT)
        GPIO.setup(self.LED_B, GPIO.OUT)
        self.led_r_pwm = GPIO.PWM(self.LED_R, 2000)
        self.led_g_pwm = GPIO.PWM(self.LED_G, 2000)
        self.led_b_pwm = GPIO.PWM(self.LED_B, 2000)
        self.led_r_pwm.start(100)
        self.led_g_pwm.start(100)
        self.led_b_pwm.start(100)
        self.set_brightness(brightness_percentage)

    def set_brightness(self, brightness_percentage):
        '''
        Sets the brightness of the RGB LED.

        :param brightness_percentage: A float from 0.0 to 100.0 representing the brightness.
        '''
        if brightness_percentage < 0.0 or brightness_percentage > 100.0:
            raise ValueError("Brightness must be between 0.0 and 100.0.")
        self._brightness = brightness_percentage

    def set_rgb(self, rgb_tuple):
        '''
        Sets the RGB LED color directly from a tuple of normalized RGB values.

        :param rgb_tuple: A tuple (r, g, b) where each value is between 0.0 and 1.0.
        '''
        red   = rgb_tuple[0]
        green = rgb_tuple[1]
        blue  = rgb_tuple[2]
        self.led_r_pwm.ChangeDutyCycle((1.0 - red) * 100)
        self.led_g_pwm.ChangeDutyCycle((1.0 - green) * 100)
        self.led_b_pwm.ChangeDutyCycle((1.0 - blue) * 100)

    def set_color(self, color):
        '''
        Sets the color of the RGB LED using a Color enum instance.

        :param color: An instance of the Color enum.
        '''
        brightness_factor = self._brightness / 100.0
        red   = (color.value[1] / 255.0) * brightness_factor
        green = (color.value[2] / 255.0) * brightness_factor
        blue  = (color.value[3] / 255.0) * brightness_factor
        self.set_rgb((red, green, blue))

    def cleanup(self):
        '''
        Sets the RGBLED off and cleans up the GPIO pins used by the class.
        '''
        self.set_color(Color.BLACK)
        self.led_r_pwm.stop()
        self.led_g_pwm.stop()
        self.led_b_pwm.stop()
        GPIO.cleanup()

#EOF
