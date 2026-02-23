#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-02-17
# modified: 2026-02-17
#
# changes: general cleanup; added methods for compatibility with existing API;
# unused constants have been removed; root-level constants now underscored.

###############################################################################
#
# COPYRIGHT(c) 2017 STMicroelectronics International N.V. All rights reserved.
#
# This file is part of VL53L1 Core and is dual licensed,
# either 'STMicroelectronics Proprietary license'
# or 'BSD 3-clause "New" or "Revised" License', at your option.
#===============================================================================
#
# License terms: BSD 3-clause "New" or "Revised" License.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software
# without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#===============================================================================

# load necessary modules:
#-----------------------------------------------------------
import time
from exceptions import TimeoutError

# From vL53l1x_class.h Header File
_VL53L1_I2C_SLAVE__DEVICE_ADDRESS =                   0x0001
_VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND =       0x0008
_ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS =     0x0016
_ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS = 0x0018
_ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS = 0x001A
_ALGO__PART_TO_PART_RANGE_OFFSET_MM =                 0x001E
_MM_CONFIG__INNER_OFFSET_MM =                         0x0020
_MM_CONFIG__OUTER_OFFSET_MM =                         0x0022
_GPIO_HV_MUX__CTRL =                                  0x0030
_GPIO__TIO_HV_STATUS =                                0x0031
_SYSTEM__INTERRUPT_CONFIG_GPIO =                      0x0046
_PHASECAL_CONFIG__TIMEOUT_MACROP =                    0x004B
_RANGE_CONFIG__TIMEOUT_MACROP_A_HI =                  0x005E
_RANGE_CONFIG__VCSEL_PERIOD_A =                       0x0060
_RANGE_CONFIG__VCSEL_PERIOD_B =                       0x0063
_RANGE_CONFIG__TIMEOUT_MACROP_B_HI =                  0x0061
_RANGE_CONFIG__SIGMA_THRESH =                         0x0064
_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS =        0x0066
_RANGE_CONFIG__VALID_PHASE_HIGH =                     0x0069
_VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD =             0x006C
_SYSTEM__THRESH_HIGH =                                0x0072
_SYSTEM__THRESH_LOW =                                 0x0074
_SD_CONFIG__WOI_SD0 =                                 0x0078
_SD_CONFIG__INITIAL_PHASE_SD0 =                       0x007A
_ROI_CONFIG__USER_ROI_CENTRE_SPAD =                   0x007F
_ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE =      0x0080
_SYSTEM__INTERRUPT_CLEAR =                            0x0086
_SYSTEM__MODE_START =                                 0x0087
_VL53L1_RESULT__RANGE_STATUS =                        0x0089
_VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0 =      0x008C
_RESULT__AMBIENT_COUNT_RATE_MCPS_SD =                 0x0090
_VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = 0x0096
_VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 = 0x0098
_VL53L1_RESULT__OSC_CALIBRATE_VAL =                   0x00DE
_VL53L1_FIRMWARE__SYSTEM_STATUS =                     0x00E5
_VL53L1_IDENTIFICATION__MODEL_ID =                    0x010F
_DISTANCE_MODE_SHORT =                                1
_DISTANCE_MODE_LONG =                                 2

_VL51L1X_DEFAULT_CONFIGURATION = [
0x00, 0x01, 0x01, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08,
0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x0F,
0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02,
0x0a, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8,
0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01,
0xdb, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x80, 0x08,
0xb8, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x89, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00,
0x00
]

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
# VL53L1_define_Error_group Error and Warning code returned by API
# The following DEFINE are used to identify the PAL ERROR

_VL53L1_ERROR_NONE =       0
_VL53L1_ERROR_TIME_OUT =  -7

# class representing a VL53L1 sensor component
class VL53L1X:
    DEFAULT_DEVICE_ADDRESS = 0x29
    # software version information
    VL53L1X_IMPLEMENTATION_VER_MAJOR =       1
    VL53L1X_IMPLEMENTATION_VER_MINOR =       0
    VL53L1X_IMPLEMENTATION_VER_SUB =         1
    VL53L1X_IMPLEMENTATION_VER_REVISION = 0000

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __init__(self, i2c, address=0x29, distance_mode=2, timing_budget_ms=100, debug=False):
        '''
        Initialize VL53L1X sensor (compatible with VL53L0X API).
        
        Args:
            i2c: MicroPython I2C bus instance
            address: I2C address (default 0x29)
            distance_mode: 1=short (max 1.3m), 2=long (max 4m, default)
            timing_budget_ms: timing budget in ms - options: 15, 20, 33, 50, 100 (default), 200, 500
            debug: enable debug output (default False)
        '''
        if i2c is None:
            raise ValueError("no I2C driver provided.")
        self._i2c = i2c
        self._address = address
        self._debug = debug
        self._distance_mode = distance_mode
        self._timing_budget_ms = timing_budget_ms
        self._status = 0
        time.sleep_ms(100)
        self.init()
        self._started = False

    def init(self):
        '''
        initialize the sensor with default values.
        '''
        self._status = 0
        sensorState = 0
        # wait for boot
        timeout_count = 0
        while not sensorState and not self._status:
            sensorState = self.boot_state()
            time.sleep_ms(2)
            timeout_count += 1
            if timeout_count > 100:
                raise TimeoutError()
        if not self._status:
            self._status = self.sensor_init()
        # set distance mode
        if self._distance_mode in [1, 2]:
            self.set_distance_mode(self._distance_mode)
        # set timing budget
        if self._timing_budget_ms:
            self.set_timing_budget_in_ms(self._timing_budget_ms)
        return self._status

    # VL53L0X-compatible API ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def start(self, period=0):
        '''
        start continuous ranging (compatible with VL53L0X API).
        '''
        self.start_ranging()
        self._started = True
        return self._status

    def stop(self):
        '''
        stop ranging (compatible with VL53L0X API).
        '''
        self.stop_ranging()
        self._started = False
        return self._status

    def read(self):
        '''
        read distance in mm (compatible with VL53L0X API).
        waits for data to be ready. auto-starts ranging if not already started.
        
        Returns:
            int: distance in millimeters
        '''
        if not self._started:
            # single-shot mode
            self.start_ranging()
            timeout = 0
            while not self.check_for_data_ready():
                time.sleep_ms(1)
                timeout += 1
                if timeout > 1000:
                    raise TimeoutError()
            distance = self.get_distance()
            self.clear_interrupt()
            self.stop_ranging()
        else:
            # continuous mode - wait for data ready
            timeout = 0
            while not self.check_for_data_ready():
                time.sleep_ms(1)
                timeout += 1
                if timeout > 1000:
                    raise TimeoutError()
            distance = self.get_distance()
            self.clear_interrupt()
        
        return distance

    # VL53L1X native methods ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def get_distance(self):
        '''
        This function returns the distance measured by the sensor in mm.

        @return Integer Distance measured by the sensor in mm
        '''
        self._status = 0
        distance = self.__i2cRead(self._address, _VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, 2)
        return distance

    # VL53L1X_api.h functions ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def get_sw_version(self):
        '''
        This function returns the SW driver version.

        @return List [major, minor, build, revision] numbers
        '''
        self._status = 0
        major = self.VL53L1X_IMPLEMENTATION_VER_MAJOR
        minor = self.VL53L1X_IMPLEMENTATION_VER_MINOR
        build = self.VL53L1X_IMPLEMENTATION_VER_SUB
        revision = self.VL53L1X_IMPLEMENTATION_VER_REVISION
        return [major, minor, build, revision]

    def set_i2c_address(self, new_address):
        '''
        This function sets the sensor I2C address used in case multiple devices.
        application, default address 0x29 (0x52 >> 1)

        @param new_address: I2C address to change device to
        '''
        self._status = 0
        self._status = self.__i2cWrite(self._address, _VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_address, 1)
        self._address = new_address
        return self._status

    def sensor_init(self):
        '''
        This function loads the 135 bytes default values to initialize the sensor.

        @return Integer 0 on success or error code
        '''
        self._status = 0
        Addr = 0x00
        tmp = 0
        timeout = 0
        for Addr in range(0x2D, 0x87 + 1):
            self._status = self.__i2cWrite(self._address, Addr, _VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D], 1)
        self._status = self.start_ranging()
#       while(tmp == 0):
#           tmp = self.check_for_data_ready()
#           timeout = timeout + 1
#           if (timeout > 50):
#               raise TimeoutError()
        while(tmp == 0):
            tmp = self.check_for_data_ready()
            timeout = timeout + 1
            if (timeout > 50):
                self.status = _VL53L1_ERROR_TIME_OUT                      
                return self.status
        tmp = 0
        self._status = self.clear_interrupt()
        self._status = self.stop_ranging()
        self._status = self.__i2cWrite(self._address, _VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09, 1) #  two bounds VHV
        self._status = self.__i2cWrite(self._address, 0x0B, 0, 1) #  start VHV from the previous temperature
        return self._status

    def clear_interrupt(self):
        '''
        This function clears the interrupt, to be called after a ranging data reading 
        to arm the interrupt for the next data ready event.
        '''
        self._status = 0
        self._status = self.__i2cWrite(self._address, _SYSTEM__INTERRUPT_CLEAR, 0x01, 1)
        return self._status

    def set_interrupt_polarity(self, NewPolarity):
        '''
        This function programs the interrupt polarity.

        @param Integer NewPolarity: 1 = active high (default), 0 = active low
        '''
        self._status = 0
        Temp = self.__i2cRead(self._address, _GPIO_HV_MUX__CTRL, 1)
        Temp = Temp & 0xEF
        self._status = self.__i2cWrite(self._address, _GPIO_HV_MUX__CTRL, Temp | (not (NewPolarity & 1)) << 4, 1)
        return self._status

    def get_interrupt_polarity(self):
        '''
        This function returns the current interrupt polarity.

        @return Integer 1 = active high (default), 0 = active low
        '''
        self._status = 0
        Temp = self.__i2cRead(self._address, _GPIO_HV_MUX__CTRL, 1)
        Temp = Temp & 0x10
        pInterruptPolarity = not (Temp >> 4)
        return pInterruptPolarity

    def start_ranging(self):
        '''
        This function starts the ranging distance operation.

        The ranging operation is continuous. The clear interrupt has to be done after each
        get data to allow the interrupt to raise when the next data is ready
        1=active high (default), 0=active low, use set_interrupt_polarity() to change
        the interrupt polarity if required.
        '''
        self._status = 0
        self._status = self.__i2cWrite(self._address, _SYSTEM__MODE_START, 0x40, 1) # enable VL53L1X
        return self._status

    def stop_ranging(self):
        '''
        This function stops the ranging.
        '''
        self._status = 0
        self._status = self.__i2cWrite(self._address, _SYSTEM__MODE_START, 0x00, 1) # disable VL53L1X
        return self._status

    def check_for_data_ready(self):
        '''
        This function checks if the new ranging data is available by polling the dedicated register.

        @return Integer 1 if new data is ready, 0 if not
        '''
        self._status = 0
        IntPol = self.get_interrupt_polarity()
        Temp = self.__i2cRead(self._address, _GPIO__TIO_HV_STATUS, 1)
        # read in the register to check if a new value is available
        if (self._status == 0):
            if ((Temp & 1) == IntPol):
                isDataReady = 1
            else:
                isDataReady = 0
        return isDataReady

    def set_timing_budget_in_ms(self, TimingBudgetInMs):
        '''
        This function programs the timing budget in ms.

        @param TimingBudgetInMs: Predefined values = 15, 20, 33, 50, 100 (default), 200, 500.
        '''
        self._status = 0
        DM = self.get_distance_mode()
        if (DM == 0):
            return 1
        elif (DM == 1):    # Short DistanceMode
            if TimingBudgetInMs == 15: # only available in short distance mode
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01D, 2)
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0027, 2)
            elif TimingBudgetInMs == 20:
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0051, 2)
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E, 2)
            elif TimingBudgetInMs == 33:
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00D6, 2)
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E, 2)
            elif TimingBudgetInMs == 50:
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x1AE, 2)
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01E8, 2)
            elif TimingBudgetInMs == 100:
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02E1, 2)
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0388, 2)
            elif TimingBudgetInMs == 200:
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x03E1, 2)
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0496, 2)
            elif TimingBudgetInMs == 500:
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0591, 2)
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x05C1, 2)
            else:
                self._status = 1
        else:
            if TimingBudgetInMs == 20:
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001E, 2)
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0022, 2)
            elif TimingBudgetInMs == 33:
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0060, 2)
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E, 2)
            elif TimingBudgetInMs == 50:
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00AD, 2)
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x00C6, 2)
            elif TimingBudgetInMs == 100:
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01CC, 2)
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01EA, 2)
            elif TimingBudgetInMs == 200:
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02D9, 2)
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x02F8, 2)
            elif TimingBudgetInMs == 500:
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x048F, 2)
                self.__i2cWrite(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x04A4, 2)
            else:
                self._status = 1
        return self._status

    def get_timing_budget_in_ms(self):
        '''
        This function returns the current timing budget in ms.
        '''
        self._status = 0
        Temp = self.__i2cRead(self._address, _RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 2)
        def get_timing_budget_in_ms_switch(var):
            switcher = {
                0x001D:15,
                0x0051:20,
                0x001E:20,
                0x00D6:33,
                0x0060:33,
                0x1AE :50,
                0x00AD:50,
                0x02E1:100,
                0x01CC:100,
                0x03E1:200,
                0x02D9:200,
                0x0591:500,
                0x048F:500}
            return switcher.get(var,0)
        pTimingBudget = get_timing_budget_in_ms_switch(Temp)
        return pTimingBudget

    def set_distance_mode(self, DM):
        '''
        This function programs the distance mode (1=short, 2=long(default)).

        @param Integer DM: Distance Mode
            * 1- Short mode max distance is limited to 1.3 m but better ambient immunity
            * 2- Long mode can range up to 4 m in the dark with 200 ms timing budget (default)
        '''
        self._status = 0
        TB = self.get_timing_budget_in_ms()
        if DM == 1:
            self._status = self.__i2cWrite(self._address, _PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14, 1)
            self._status = self.__i2cWrite(self._address, _RANGE_CONFIG__VCSEL_PERIOD_A, 0x07, 1)
            self._status = self.__i2cWrite(self._address, _RANGE_CONFIG__VCSEL_PERIOD_B, 0x05, 1)
            self._status = self.__i2cWrite(self._address, _RANGE_CONFIG__VALID_PHASE_HIGH, 0x38, 1)
            self._status = self.__i2cWrite(self._address, _SD_CONFIG__WOI_SD0, 0x0705, 2)
            self._status = self.__i2cWrite(self._address, _SD_CONFIG__INITIAL_PHASE_SD0, 0x0606, 2)
        elif DM == 2:
            self._status = self.__i2cWrite(self._address, _PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A, 1)
            self._status = self.__i2cWrite(self._address, _RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F, 1)
            self._status = self.__i2cWrite(self._address, _RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D, 1)
            self._status = self.__i2cWrite(self._address, _RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8, 1)
            self._status = self.__i2cWrite(self._address, _SD_CONFIG__WOI_SD0, 0x0F0D, 2)
            self._status = self.__i2cWrite(self._address, _SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E, 2)
        else:
            if self._debug == 1:
                print("Invalid DIstance Mode")
        self._status = self.set_timing_budget_in_ms(TB)
        return self._status

    def get_distance_mode(self):
        '''
        This function returns the current distance mode (1=short, 2=long).

        @return Integer Distance Mode
            * 1- Short mode max distance is limited to 1.3 m but better ambient immunity
            * 2- Long mode can range up to 4 m in the dark with 200 ms timing budget (default)
        '''
        self._status = 0
        TempDM = self.__i2cRead(self._address,_PHASECAL_CONFIG__TIMEOUT_MACROP, 1)
        if TempDM is None:
            print("WARNING: read returned null distance mode value.")
        elif (TempDM == 0x14):
            return _DISTANCE_MODE_SHORT
        elif (TempDM == 0x0A):
            return _DISTANCE_MODE_LONG
        else: # '74' (0x4A)
            print("WARNING: read returned unrecognised distance mode value of '{}' (0x{:02X})".format(TempDM, TempDM))
        # default
        return _DISTANCE_MODE_LONG

    def set_inter_measurement_in_ms(self, InterMeasMs):
        '''
        This function programs the Intermeasurement period in ms.

        @param InterMeasMs: Intermeasurement period must be >/= timing budget. This condition
                            is not checked by the API, the customer has the duty to check the
                            condition. Default = 100 ms
        '''
        self._status = 0
        ClockPLL = self.__i2cRead(self._address, _VL53L1_RESULT__OSC_CALIBRATE_VAL, 2)
        ClockPLL = ClockPLL&0x3FF
        self._status = self.__i2cWrite(self._address, _VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD,
                int(ClockPLL * InterMeasMs * 1.075), 4)
        return self._status

    def get_inter_measurement_in_ms(self):
        '''
        This function returns the Intermeasurement period in ms.

        @return Integer Intermeasurement period in ms
        '''
        tmp = 0
        ClockPLL = 0
        pIM = 0
        tmp = self.__i2cRead(self._address, _VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, 4)
        ClockPLL = self.__i2cRead(self._address, _VL53L1_RESULT__OSC_CALIBRATE_VAL, 2)
        ClockPLL = ClockPLL&0x3FF
        pIM= (tmp/(ClockPLL*1.065))
        return pIM

    def boot_state(self):
        '''
        This function returns the boot state of the device (1:booted, 0:not booted).

        @return Integer Boot state
            * 1- booted
            * 0- not booted
        '''
        self._status = 0
        state = 0
        state = self.__i2cRead(self._address,_VL53L1_FIRMWARE__SYSTEM_STATUS, 1)
        return state

    def get_sensor_id(self):
        '''
        This function returns the sensor id, sensor Id must be 0xEEAC.

        @return Integer Sensor ID
        '''
        self._status = 0
        sensorId = 0
        sensorId = self.__i2cRead(self._address, _VL53L1_IDENTIFICATION__MODEL_ID, 2)
        return sensorId

    def get_signal_per_spad(self):
        '''
        This function returns the returned signal per SPAD in kcps/SPAD.
        (kcps stands for Kilo Count Per Second).

        @return     Signal per SPAD (Kilo Count Per Second/SPAD).
        '''
        self._status = 0
        SpNb=1
        signal = self.__i2cRead(self._address, _VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, 2)
        SpNb = self.__i2cRead(self._address, _VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, 2)
        signalRate = (2000.0*signal/SpNb)
        return signalRate

    def get_ambient_per_spad(self):
        '''
        This function returns the ambient per SPAD in kcps/SPAD.

        @return     Ambient per SPAD

        '''
        self._status = 0
        SpNb=1
        AmbientRate = self.__i2cRead(self._address, _RESULT__AMBIENT_COUNT_RATE_MCPS_SD, 2)
        SpNb = self.__i2cRead(self._address, _VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, 2)
        ambPerSp=(2000.0 * AmbientRate / SpNb)
        return ambPerSp

    def get_signal_rate(self):
        '''
        This function returns the returned signal in kcps.

        @return     signal in kcps
        '''
        self._status = 0
        tmp = self.__i2cRead(self._address, _VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, 2)
        signal = tmp*8
        return signal

    def get_spad_nb(self):
        '''
        This function returns the current number of enabled SPADs.

        @return     Number of enabled SPADs
        '''
        self._status = 0
        tmp = self.__i2cRead(self._address, _VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, 2)
        spNb = tmp >> 8
        return spNb

    def get_ambient_rate(self):
        '''
        This function returns the ambient rate in kcps.

        @return     Ambient rate in kcps
        '''
        self._status = 0
        tmp = self.__i2cRead(self._address, _RESULT__AMBIENT_COUNT_RATE_MCPS_SD, 2)
        ambRate = tmp*8
        return ambRate

    def get_range_status(self):
        '''
        This function returns the ranging status error.

        @return     Ranging status error
                        * 0- no error
                        * 1- sigma failed
                        * 2- signal failed
                        * 7- wrap-around
        '''
        self._status = 0
        RgSt = self.__i2cRead(self._address, _VL53L1_RESULT__RANGE_STATUS, 1)
        RgSt = RgSt&0x1F

        def get_range_status_switch(var):
            switcher = { 9:0, 6:1, 4:2, 8:3, 5:4, 3:5, 19:6, 7:7, 12:9, 18:10, 22:11, 23:12, 13:13}
            return switcher.get(var,255)
        rangeStatus = get_range_status_switch(RgSt)
        return rangeStatus

    def set_offset(self, OffsetValue):
        '''
        This function programs the offset correction in mm.

        @param OffsetValue: The offset correction value to program in mm
        '''
        self._status = 0
        Temp = OffsetValue*4
        self.__i2cWrite(self._address, _ALGO__PART_TO_PART_RANGE_OFFSET_MM, Temp, 2)
        self.__i2cWrite(self._address, _MM_CONFIG__INNER_OFFSET_MM, 0x0, 2)
        self.__i2cWrite(self._address, _MM_CONFIG__OUTER_OFFSET_MM, 0x0, 2)
        return self._status

    def get_offset(self):
        '''
        This function returns the programmed offset correction value in mm.

        @return Integer Offset correction value in mm
        '''
        self._status = 0
        Temp = self.__i2cRead(self._address,_ALGO__PART_TO_PART_RANGE_OFFSET_MM, 2)
        Temp = Temp << 3
        Temp = Temp >> 5
        offset = Temp
        return offset

    def set_xtalk(self, XtalkValue):
        '''
        This function programs the xtalk correction value in cps (Count Per Second).
        This is the number of photons reflected back from the cover glass in cps.

        @param XTalkValue: xtalk correction value in count per second to avoid float type
        '''
        self._status = 0
        self._status = self.__i2cWrite(self._address, _ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS, 0x0000, 2)
        self._status = self.__i2cWrite(self._address, _ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS, 0x0000, 2)
        self._status = self.__i2cWrite(self._address, _ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, (XtalkValue << 9)/1000, 2) # << 9 (7.9 format) and /1000 to convert cps to kpcs
        return self._status

    def get_xtalk(self):
        '''
        This function returns the current programmed xtalk correction value in cps.

        @return     xtalk correction value in cps
        '''
        self._status = 0
        tmp = self.__i2cRead(self._address,_ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, 2)
        xtalk = (tmp*1000) >> 9 # 1000 to convert kcps to cps and >> 9 (7.9 format)
        return xtalk

    def set_distance_threshold(self,
                    ThreshLow,
                    ThreshHigh, Window,
                    IntOnNoTarget):
        '''
        This function programs the threshold detection mode.

        @param mm ThreshLow: The threshold under which one the device raises an interrupt if Window = 0
        @param mm ThreshHigh: The threshold above which one the device raises an interrupt if Window = 1
        @param Window: Window detection mode:
                                        * 0- below
                                        * 1- above
                                        * 2- out
                                        * 3- in
        @param IntOnNoTarget: 1 (No longer used - just set to 1)

        Example:
            * self.set_distance_threshold(100,300,0,1): Below 100
            * self.set_distance_threshold(100,300,1,1): Above 300
            * self.set_distance_threshold(100,300,2,1): Out of window
            * self.set_distance_threshold(100,300,3,1): In window
        '''
        self._status = 0
        Temp = 0
        Temp = self.__i2cRead(self._address, _SYSTEM__INTERRUPT_CONFIG_GPIO, 1)
        Temp = Temp & 0x47
        if (IntOnNoTarget == 0):
            self._status = self.__i2cWrite(self._address, _SYSTEM__INTERRUPT_CONFIG_GPIO,
                    (Temp | (Window & 0x07)), 1)
        else:
            self._status = self.__i2cWrite(self._address, _SYSTEM__INTERRUPT_CONFIG_GPIO,
                    ((Temp | (Window & 0x07)) | 0x40), 1)

        self._status = self.__i2cWrite(self._address, _SYSTEM__THRESH_HIGH, ThreshHigh, 2)
        self._status = self.__i2cWrite(self._address, _SYSTEM__THRESH_LOW, ThreshLow, 2)
        return self._status

    def get_distance_threshold_window(self):
        '''
        This function returns the window detection mode (0=below 1=above 2=out 3=in).

        @return Integer Window detection mode
            * 0- below
            * 1- above
            * 2- out
            * 3- in
        '''
        self._status = 0
        tmp = self.__i2cRead(self._address,_SYSTEM__INTERRUPT_CONFIG_GPIO, 1)
        window = (tmp & 0x7)
        return window

    def get_distance_threshold_low(self):
        '''
        This function returns the low threshold in mm.

        @return Integer Low threshold in mm
        '''
        self._status = 0
        low = self.__i2cRead(self._address,_SYSTEM__THRESH_LOW, 2)
        return low

    def get_distance_threshold_high(self):
        '''
        This function returns the high threshold in mm.

        @return Integer High threshold in mm
        '''
        self._status = 0
        high = self.__i2cRead(self._address,_SYSTEM__THRESH_HIGH, 2)
        return high

    def set_roi(self, X, Y, OpticalCenter = 199):
        '''
        This function programs the ROI (Region of Interest). The height and width of the ROI (X, Y)
        are set in SPADs; the smallest acceptable ROI size = 4 (4 x 4). The optical center is set
        based on table below.

        To set the center, use the pad that is to the right and above (i.e. upper right of) the
        exact center of the region you'd like to measure as your optical center.

        Table of Optical Centers:

            128,136,144,152,160,168,176,184,  192,200,208,216,224,232,240,248
            129,137,145,153,161,169,177,185,  193,201,209,217,225,233,241,249
            130,138,146,154,162,170,178,186,  194,202,210,218,226,234,242,250
            131,139,147,155,163,171,179,187,  195,203,211,219,227,235,243,251
            132,140,148,156,164,172,180,188,  196,204,212,220,228,236,244,252
            133,141,149,157,165,173,181,189,  197,205,213,221,229,237,245,253
            134,142,150,158,166,174,182,190,  198,206,214,222,230,238,246,254
            135,143,151,159,167,175,183,191,  199,207,215,223,231,239,247,255

            127,119,111,103,095,087,079,071,  063,055,047,039,031,023,015,007
            126,118,110,102,094,086,078,070,  062,054,046,038,030,022,014,006
            125,117,109,101,093,085,077,069,  061,053,045,037,029,021,013,005
            124,116,108,100,092,084,076,068,  060,052,044,036,028,020,012,004
            123,115,107,099,091,083,075,067,  059,051,043,035,027,019,011,003
            122,114,106,098,090,082,074,066,  058,050,042,034,026,018,010,002
            121,113,105,097,089,081,073,065,  057,049,041,033,025,017,009,001
            120,112,104,096,088,080,072,064,  056,048,040,032,024,016,008,0 Pin 1

            (Each SPAD has a number which is not obvious.)

        @param X: ROI Width
        @param Y: ROI Height
        @param OpticalCenter: The pad that is to the upper right of the exact center of the ROI (see table above). (default=199)
        '''
        self._status = 0
        if (X > 16):
            X = 16
        if (Y > 16):
            Y = 16
        if (X > 10 or Y > 10):
            OpticalCenter = 199
        self._status = self.__i2cWrite(self._address, _ROI_CONFIG__USER_ROI_CENTRE_SPAD, OpticalCenter, 1)
        self._status = self.__i2cWrite(self._address, _ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, (Y - 1) << 4 | (X - 1), 1)
        return self._status

    def get_roi_xy(self):
        '''
        This function returns width X and height Y.

        @return List Region of Interest Width (X) and Height (Y)
        '''
        self._status = 0
        tmp = self.__i2cRead(self._address,_ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, 1)
        ROI_X = (tmp & 0x0F) + 1
        ROI_Y = ((tmp & 0xF0) >> 4) + 1
        return [ROI_X, ROI_Y]

    def set_signal_threshold(self, Signal):
        '''
        This function programs a new signal threshold in kcps (default=1024 kcps).

        @param Signal: Signal threshold in kcps (default=1024 kcps)
        '''
        self._status = 0
        self.__i2cWrite(self._address,_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,Signal >> 3, 2)
        return self._status

    def get_signal_threshold(self):
        '''
        This function returns the current signal threshold in kcps.

        @return     Signal threshold in kcps
        '''
        self._status = 0
        tmp = self.__i2cRead(self._address, _RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 2)
        signal = tmp << 3
        return signal

    def set_sigma_threshold(self, Sigma):
        '''
        This function programs a new sigma threshold in mm (default=15 mm).

        @param Sigma: Sigma threshold in mm (default=15 mm)
        '''
        self._status = 0
        if(Sigma>(0xFFFF >> 2)):
            return 1
        # 16 bits register 14.2 format
        self._status = self.__i2cWrite(self._address,_RANGE_CONFIG__SIGMA_THRESH,Sigma << 2, 2)
        return self._status

    def get_sigma_threshold(self):
        '''
        This function returns the current sigma threshold in mm.

        @return Integer Sigma threshold in mm
        '''
        self._status = 0
        tmp = self.__i2cRead(self._address,_RANGE_CONFIG__SIGMA_THRESH, 2)
        sigma = tmp >> 2
        return sigma

    def start_temperature_update(self):
        '''
        This function performs the temperature calibration.
        It is recommended to call this function any time the temperature might have changed by more than 8 deg C
        without sensor ranging activity for an extended period.
        '''
        self._status = 0
        tmp=0
        self._status = self.__i2cWrite(self._address,_VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,0x81, 1) #  full VHV
        self._status = self.__i2cWrite(self._address,0x0B,0x92, 1)
        self._status = self.start_ranging()
        while(tmp==0):
            tmp = self.check_for_data_ready()
        tmp = 0
        self._status = self.clear_interrupt()
        self._status = self.stop_ranging()
        self._status = self.__i2cWrite(self._address, _VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09, 1) #  two bounds VHV
        self._status = self.__i2cWrite(self._address, 0x0B, 0, 1) #  start VHV from the previous temperature
        return self._status

    # VL53L1X_calibration.h functions ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def calibrate_offset(self, TargetDistInMm):
        '''
        This function performs the offset calibration.
        The function returns the offset value found and programs the offset compensation into the device.

        @param TargetDistInMm: * Target distance in mm, ST recommended 100 mm
                                * Target reflectance = grey17%

        @return int status: 0 if no error, other if error
        '''
        tmp = 0
        AverageDistance = 0
        self._status = 0
        self._status = self.__i2cWrite(self._address, _ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0x0, 2)
        self._status = self.__i2cWrite(self._address, _MM_CONFIG__INNER_OFFSET_MM, 0x0, 2)
        self._status = self.__i2cWrite(self._address, _MM_CONFIG__OUTER_OFFSET_MM, 0x0, 2)
        self._status = self.start_ranging() # enable VL53L1X sensor
        for i in range(0, 50):
            while (tmp == 0):
                tmp = self.check_for_data_ready()
            tmp = 0
            distance = self.get_distance()
            self._status = self.clear_interrupt()
            AverageDistance = AverageDistance + distance
        self._status = self.stop_ranging()
        AverageDistance = AverageDistance / 50
        offset = TargetDistInMm - AverageDistance
        self._status = self.__i2cWrite(self._address, _ALGO__PART_TO_PART_RANGE_OFFSET_MM, offset*4, 2)
        return self._status

    def calibrate_xtalk(self, TargetDistInMm):
        '''
        This function performs the xtalk calibration.
        The function returns the xtalk value found and programs the xtalk compensation to the device

        @param TargetDistInMm: Target distance in mm
        * The target distance is the distance where the sensor start to "under range" due to the influence
          of the photons reflected back from the cover glass becoming strong (also called the inflection point).
        * Target reflectance = grey 17%

        @return int status: 0 if no error, other if error
        '''
        tmp = 0
        AverageSignalRate = 0
        AverageDistance = 0
        AverageSpadNb = 0
        distance = 0
        self._status = 0
        self._status = self.__i2cWrite(self._address, 0x0016,0, 2)
        self._status = self.start_ranging()
        for i in range(0, 50):
            while (tmp == 0):
                tmp = self.check_for_data_ready()
            tmp = 0
            sr = self.get_signal_rate()
            distance = self.get_distance()
            self._status = self.clear_interrupt()
            AverageDistance = AverageDistance + distance
            spadNum = self.get_spad_nb()
            AverageSpadNb = AverageSpadNb + spadNum
            AverageSignalRate =    AverageSignalRate + sr
        self._status = self.stop_ranging()
        AverageDistance = AverageDistance / 50
        AverageSpadNb = AverageSpadNb / 50
        AverageSignalRate = AverageSignalRate / 50
        # calculate Xtalk value
        xtalk = (512*(AverageSignalRate*(1-(AverageDistance/TargetDistInMm)))/AverageSpadNb)
        self._status = self.__i2cWrite(self._address, 0x0016, xtalk, 2)
        return self._status

    # protected ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def __get_tick_count(self):
        '''
        returns current tick count in [ms]
        '''
        self._status = _VL53L1_ERROR_NONE
        ptick_count_ms = 0
        return ptick_count_ms

    def __wait_us(self, wait_us):
        time.sleep(wait_us/1000/1000)
        return _VL53L1_ERROR_NONE

    def __wait_ms(self, wait_ms):
        time.sleep(wait_ms/1000)
        return _VL53L1_ERROR_NONE

    def __wait_value_mask_ex(self, timeout_ms, index, value, mask, poll_delay_ms):
        '''
        platform implementation of ```WaitValueMaskEx``` V2WReg script command
        '''
        self._status     = _VL53L1_ERROR_NONE
        start_time_ms   = 0
        current_time_ms = 0
        polling_time_ms = 0
        byte_value      = 0
        found           = 0
        # calculate time limit in absolute time
        start_time_ms = self.__get_tick_count()
        # remember current trace functions and temporarily disable function logging
        # wait until value is found, timeout reached on error occurred
        while ((self._status == _VL53L1_ERROR_NONE) and
            (polling_time_ms < timeout_ms) and
            (found == 0)):
            if (self._status == _VL53L1_ERROR_NONE):
                byte_value = self.__i2cRead(self._address, index, 1)
            if ((byte_value & mask) == value):
                found = 1
            if (self._status == _VL53L1_ERROR_NONE and
                found == 0 and
                poll_delay_ms > 0):
                self._status = self.__wait_ms(poll_delay_ms)
            # Update polling time (Compare difference rather than absolute to negate 32bit wrap around issue)
            current_time_ms = self.__get_tick_count()
            polling_time_ms = current_time_ms - start_time_ms
        if (found == 0 and self._status == _VL53L1_ERROR_NONE):
            self._status = _VL53L1_ERROR_TIME_OUT
        return self._status

    # write and read functions for I2C ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def __i2cWrite(self, address, register, data, nbytes):
        '''
        wrapper for standard MicroPython I2C. Handles 16-bit register addresses.
        '''
        registerMSB = (register >> 8) & 0xFF
        registerLSB = register & 0xFF
        
        buffer = bytearray([registerMSB, registerLSB])
        
        if nbytes == 4:
            buffer.extend([(data >> 24) & 0xFF, (data >> 16) & 0xFF, 
                           (data >> 8) & 0xFF, data & 0xFF])
        elif nbytes == 2:
            buffer.extend([(data >> 8) & 0xFF, data & 0xFF])
        elif nbytes == 1:
            buffer.append(data & 0xFF)
        else:
            if self._debug:
                print("__i2cWrite: invalid nbytes")
            return 1
        
        try:
            self._i2c.writeto(address, buffer)
            self._status = 0
        except Exception as e:
            if self._debug:
                print("I2C write error: {}".format(e))
            self._status = 1
        
        return self._status

    def __i2cRead(self, address, register, nbytes):
        '''
        wrapper for standard MicroPython I2C. Handles 16-bit register addresses.
        '''
        registerMSB = (register >> 8) & 0xFF
        registerLSB = register & 0xFF
        
        if nbytes not in [1, 2, 4]:
            if self._debug:
                print("__i2cRead: invalid nbytes")
            return 0
        
        try:
            # Write register address then read - as separate I2C transactions
            self._i2c.writeto(address, bytearray([registerMSB, registerLSB]))
            data_bytes = self._i2c.readfrom(address, nbytes)
            
            # Convert bytes to integer (big-endian for VL53L1X)
            data = 0
            for byte_val in data_bytes:
                data = (data << 8) | byte_val
            
            return data
        except Exception as e:
            if self._debug:
                print("I2C read error: {}".format(e))
            return 0

#EOF
