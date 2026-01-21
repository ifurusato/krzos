#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved.
# Portions Copyright (c) 2020 Bryan Siepert for Adafruit Industries (MIT License)
#
# author:   Ichiro Furusato
# created:  2026-01-20
# modified: 2026-01-21
#
# BNO085 IMU driver for CPython (refactored from CircuitPython version)
# Base class with minimal dependencies for general use

from __future__ import annotations

import time
from collections import namedtuple
from struct import pack_into, unpack_from
from typing import Any, Optional

try:
    from smbus2 import SMBus, i2c_msg
except ImportError:
    raise ImportError('smbus2 required: pip install smbus2')

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
# Constants and lookup tables

# channel names for debug output
_CHANNEL_NAMES = {
    0x0: 'SHTP_COMMAND',
    0x1: 'EXE',
    0x2: 'CONTROL',
    0x3: 'INPUT_SENSOR_REPORTS',
    0x4: 'WAKE_INPUT_SENSOR_REPORTS',
    0x5: 'GYRO_ROTATION_VECTOR',
}

# report names for debug output
_REPORT_NAMES = {
    0xFB: 'BASE_TIMESTAMP',
    0xF2: 'COMMAND_REQUEST',
    0xF1: 'COMMAND_RESPONSE',
    0xF4: 'FRS_READ_REQUEST',
    0xF3: 'FRS_READ_RESPONSE',
    0xF6: 'FRS_WRITE_DATA',
    0xF7: 'FRS_WRITE_REQUEST',
    0xF5: 'FRS_WRITE_RESPONSE',
    0xFE: 'GET_FEATURE_REQUEST',
    0xFC: 'GET_FEATURE_RESPONSE',
    0xFD: 'SET_FEATURE_COMMAND',
    0xFA: 'TIMESTAMP_REBASE',
    0x01: 'ACCELEROMETER',
    0x29: 'ARVR_STABILIZED_GAME_ROTATION_VECTOR',
    0x28: 'ARVR_STABILIZED_ROTATION_VECTOR',
    0x22: 'CIRCLE_DETECTOR',
    0x1A: 'FLIP_DETECTOR',
    0x08: 'GAME_ROTATION_VECTOR',
    0x09: 'GEOMAGNETIC_ROTATION_VECTOR',
    0x06: 'GRAVITY',
    0x02: 'GYROSCOPE',
    0x04: 'LINEAR_ACCELERATION',
    0x03: 'MAGNETIC_FIELD',
    0x1E: 'PERSONAL_ACTIVITY_CLASSIFIER',
    0x1B: 'PICKUP_DETECTOR',
    0x21: 'POCKET_DETECTOR',
    0xF9: 'PRODUCT_ID_REQUEST',
    0xF8: 'PRODUCT_ID_RESPONSE',
    0x14: 'RAW_ACCELEROMETER',
    0x15: 'RAW_GYROSCOPE',
    0x16: 'RAW_MAGNETOMETER',
    0x05: 'ROTATION_VECTOR',
    0x17: 'SAR',
    0x19: 'SHAKE_DETECTOR',
    0x12: 'SIGNIFICANT_MOTION',
    0x1F: 'SLEEP_DETECTOR',
    0x13: 'STABILITY_CLASSIFIER',
    0x1C: 'STABILITY_DETECTOR',
    0x11: 'STEP_COUNTER',
    0x18: 'STEP_DETECTOR',
    0x10: 'TAP_DETECTOR',
    0x20: 'TILT_DETECTOR',
    0x07: 'UNCALIBRATED_GYROSCOPE',
    0x0F: 'UNCALIBRATED_MAGNETIC_FIELD',
}

# channel constants
_BNO_CHANNEL_SHTP_COMMAND              = 0
_BNO_CHANNEL_EXE                       = 1
_BNO_CHANNEL_CONTROL                   = 2
_BNO_CHANNEL_INPUT_SENSOR_REPORTS      = 3
_BNO_CHANNEL_WAKE_INPUT_SENSOR_REPORTS = 4
_BNO_CHANNEL_GYRO_ROTATION_VECTOR      = 5

# command/report IDs
_GET_FEATURE_REQUEST                   = 0xFE
_SET_FEATURE_COMMAND                   = 0xFD
_GET_FEATURE_RESPONSE                  = 0xFC
_BASE_TIMESTAMP                        = 0xFB
_TIMESTAMP_REBASE                      = 0xFA
_SHTP_REPORT_PRODUCT_ID_RESPONSE       = 0xF8
_SHTP_REPORT_PRODUCT_ID_REQUEST        = 0xF9
_FRS_WRITE_REQUEST                     = 0xF7
_FRS_WRITE_DATA                        = 0xF6
_FRS_WRITE_RESPONSE                    = 0xF5
_FRS_READ_REQUEST                      = 0xF4
_FRS_READ_RESPONSE                     = 0xF3
_COMMAND_REQUEST                       = 0xF2
_COMMAND_RESPONSE                      = 0xF1

# calibration commands
_SAVE_DCD                              = 0x6
_ME_CALIBRATE                          = 0x7
_ME_CAL_CONFIG                         = 0x00
_ME_GET_CAL                            = 0x01

# sensor report IDs
BNO_REPORT_ACCELEROMETER               = 0x01
BNO_REPORT_GYROSCOPE                   = 0x02
BNO_REPORT_MAGNETOMETER                = 0x03
BNO_REPORT_LINEAR_ACCELERATION         = 0x04
BNO_REPORT_ROTATION_VECTOR             = 0x05
BNO_REPORT_GRAVITY                     = 0x06
BNO_REPORT_GAME_ROTATION_VECTOR        = 0x08
BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR = 0x09
BNO_REPORT_STEP_COUNTER                = 0x11
BNO_REPORT_STABILITY_CLASSIFIER        = 0x13
BNO_REPORT_RAW_ACCELEROMETER           = 0x14
BNO_REPORT_RAW_GYROSCOPE               = 0x15
BNO_REPORT_RAW_MAGNETOMETER            = 0x16
BNO_REPORT_SHAKE_DETECTOR              = 0x19
BNO_REPORT_ACTIVITY_CLASSIFIER         = 0x1E
BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR = 0x2A

# timing/config constants
_DEFAULT_REPORT_INTERVAL = 50000  # microseconds = 50ms
_QUAT_READ_TIMEOUT       = 0.500  # seconds
_PACKET_READ_TIMEOUT     = 2.000  # seconds
_FEATURE_ENABLE_TIMEOUT  = 2.0
_DEFAULT_TIMEOUT         = 2.0
_BNO_HEADER_LEN          = 4
_DATA_BUFFER_SIZE        = 512

# Q-point scalars for fixed-point conversion
_Q_POINT_14_SCALAR       = 2 ** (14 * -1)
_Q_POINT_12_SCALAR       = 2 ** (12 * -1)
_Q_POINT_9_SCALAR        = 2 ** (9 * -1)
_Q_POINT_8_SCALAR        = 2 ** (8 * -1)
_Q_POINT_4_SCALAR        = 2 ** (4 * -1)

# report length lookup
_REPORT_LENGTHS = {
    _SHTP_REPORT_PRODUCT_ID_RESPONSE: 16,
    _GET_FEATURE_RESPONSE: 17,
    _COMMAND_RESPONSE: 16,
    _BASE_TIMESTAMP: 5,
    _TIMESTAMP_REBASE: 5,
}

# raw reports require counterpart to be enabled
_RAW_REPORTS = {
    BNO_REPORT_RAW_ACCELEROMETER: BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_RAW_GYROSCOPE: BNO_REPORT_GYROSCOPE,
    BNO_REPORT_RAW_MAGNETOMETER: BNO_REPORT_MAGNETOMETER,
}

# available sensor reports: (scalar, count, length)
_AVAIL_SENSOR_REPORTS = {
    BNO_REPORT_ACCELEROMETER: (_Q_POINT_8_SCALAR, 3, 10),
    BNO_REPORT_GRAVITY: (_Q_POINT_8_SCALAR, 3, 10),
    BNO_REPORT_GYROSCOPE: (_Q_POINT_9_SCALAR, 3, 10),
    BNO_REPORT_MAGNETOMETER: (_Q_POINT_4_SCALAR, 3, 10),
    BNO_REPORT_LINEAR_ACCELERATION: (_Q_POINT_8_SCALAR, 3, 10),
    BNO_REPORT_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 4, 14),
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR: (_Q_POINT_12_SCALAR, 4, 14),
    BNO_REPORT_GAME_ROTATION_VECTOR: (_Q_POINT_14_SCALAR, 4, 12),
    BNO_REPORT_STEP_COUNTER: (1, 1, 12),
    BNO_REPORT_SHAKE_DETECTOR: (1, 1, 6),
    BNO_REPORT_STABILITY_CLASSIFIER: (1, 1, 6),
    BNO_REPORT_ACTIVITY_CLASSIFIER: (1, 1, 16),
    BNO_REPORT_RAW_ACCELEROMETER: (1, 3, 16),
    BNO_REPORT_RAW_GYROSCOPE: (1, 3, 16),
    BNO_REPORT_RAW_MAGNETOMETER: (1, 3, 16),
}

# initial values for reports
_INITIAL_REPORTS = {
    BNO_REPORT_ACTIVITY_CLASSIFIER: {
        'Tilting': -1,
        'most_likely': 'Unknown',
        'OnStairs': -1,
        'On-Foot': -1,
        'Other': -1,
        'On-Bicycle': -1,
        'Still': -1,
        'Walking': -1,
        'Unknown': -1,
        'Running': -1,
        'In-Vehicle': -1,
    },
    BNO_REPORT_STABILITY_CLASSIFIER: 'Unknown',
    BNO_REPORT_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0),
    BNO_REPORT_GAME_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0),
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0),
}

_ENABLED_ACTIVITIES = 0x1FF

_REPORT_ACCURACY_STATUS = [
    'Accuracy Unreliable',
    'Low Accuracy',
    'Medium Accuracy',
    'High Accuracy',
]

PacketHeader = namedtuple(
    'PacketHeader',
    ['channel_number', 'sequence_number', 'data_length', 'packet_byte_count'],
)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
# Helper functions

class PacketError(Exception):
    '''raised when the packet could not be parsed'''
    pass

def _elapsed(start_time: float) -> float:
    return time.monotonic() - start_time

def _parse_sensor_report_data(report_bytes: bytearray) -> tuple[tuple, int]:
    '''parses reports with only 16-bit fields'''
    data_offset = 4
    report_id = report_bytes[0]
    scalar, count, _report_length = _AVAIL_SENSOR_REPORTS[report_id]
    if report_id in _RAW_REPORTS:
        format_str = '<H'  # raw reports are unsigned
    else:
        format_str = '<h'
    results = []
    accuracy = unpack_from('<B', report_bytes, offset=2)[0]
    accuracy &= 0b11

    for _offset_idx in range(count):
        total_offset = data_offset + (_offset_idx * 2)
        raw_data = unpack_from(format_str, report_bytes, offset=total_offset)[0]
        scaled_data = raw_data * scalar
        results.append(scaled_data)
    results_tuple = tuple(results)
    return (results_tuple, accuracy)

def _parse_step_couter_report(report_bytes: bytearray) -> int:
    return unpack_from('<H', report_bytes, offset=8)[0]

def _parse_stability_classifier_report(report_bytes: bytearray) -> str:
    classification_bitfield = unpack_from('<B', report_bytes, offset=4)[0]
    return ['Unknown', 'On Table', 'Stationary', 'Stable', 'In motion'][classification_bitfield]

def _parse_get_feature_response_report(report_bytes: bytearray) -> tuple[Any, ...]:
    return unpack_from('<BBBHIII', report_bytes)

def _parse_activity_classifier_report(report_bytes: bytearray) -> dict[str, str]:
    activities = [
        'Unknown',
        'In-Vehicle',
        'On-Bicycle',
        'On-Foot',
        'Still',
        'Tilting',
        'Walking',
        'Running',
        'OnStairs',
    ]
    end_and_page_number = unpack_from('<B', report_bytes, offset=4)[0]
    page_number = end_and_page_number & 0x7F
    most_likely = unpack_from('<B', report_bytes, offset=5)[0]
    confidences = unpack_from('<BBBBBBBBB', report_bytes, offset=6)

    classification = {}
    classification['most_likely'] = activities[most_likely]
    for idx, raw_confidence in enumerate(confidences):
        confidence = (10 * page_number) + raw_confidence
        activity_string = activities[idx]
        classification[activity_string] = confidence
    return classification

def _parse_shake_report(report_bytes: bytearray) -> bool:
    shake_bitfield = unpack_from('<H', report_bytes, offset=4)[0]
    return (shake_bitfield & 0x111) > 0

def parse_sensor_id(buffer: bytearray) -> tuple[int, ...]:
    '''parse the fields of a product id report'''
    if not buffer[0] == _SHTP_REPORT_PRODUCT_ID_RESPONSE:
        raise AttributeError('wrong report id for sensor id: {}'.format(hex(buffer[0])))
    sw_major = unpack_from('<B', buffer, offset=2)[0]
    sw_minor = unpack_from('<B', buffer, offset=3)[0]
    sw_patch = unpack_from('<H', buffer, offset=12)[0]
    sw_part_number = unpack_from('<I', buffer, offset=4)[0]
    sw_build_number = unpack_from('<I', buffer, offset=8)[0]
    return (sw_part_number, sw_major, sw_minor, sw_patch, sw_build_number)

def _parse_command_response(report_bytes: bytearray) -> tuple[Any, Any]:
    report_body = unpack_from('<BBBBB', report_bytes)
    response_values = unpack_from('<BBBBBBBBBBB', report_bytes, offset=5)
    return (report_body, response_values)

def _insert_command_request_report(
    command: int,
    buffer: bytearray,
    next_sequence_number: int,
    command_params: Optional[list[int]] = None,
) -> None:
    if command_params and len(command_params) > 9:
        raise AttributeError(
            'command request reports can only have up to 9 arguments but {} were given'.format(len(command_params))
        )
    for _i in range(12):
        buffer[_i] = 0
    buffer[0] = _COMMAND_REQUEST
    buffer[1] = next_sequence_number
    buffer[2] = command
    if command_params is None:
        return
    for idx, param in enumerate(command_params):
        buffer[3 + idx] = param

def _report_length(report_id: int) -> int:
    if report_id < 0xF0: # sensor report
        return _AVAIL_SENSOR_REPORTS[report_id][2]
    return _REPORT_LENGTHS[report_id]

def _separate_batch(packet, report_slices: list[Any]) -> None:
    '''get first report id, look up its report length, read that many bytes, parse them'''
    next_byte_index = 0
    while next_byte_index < packet.header.data_length:
        report_id = packet.data[next_byte_index]
        # check if this is a known report type
        if report_id < 0xF0:
            if report_id not in _AVAIL_SENSOR_REPORTS:
                # unknown sensor report - skip remaining bytes to avoid corruption
                raise RuntimeError('unknown sensor report id: 0x{:02X}, skipping {} remaining bytes'.format(
                    report_id, packet.header.data_length - next_byte_index))
        elif report_id not in _REPORT_LENGTHS:
            # unknown control report
            raise RuntimeError('unknown control report id: 0x{:02X}, skipping {} remaining bytes'.format(
                report_id, packet.header.data_length - next_byte_index))
        required_bytes = _report_length(report_id)
        unprocessed_byte_count = packet.header.data_length - next_byte_index
        # handle incomplete remainder
        if unprocessed_byte_count < required_bytes:
            # log the issue but don't crash - this can happen during I2C errors
            raise PacketError('incomplete report: need {} bytes for report 0x{:02X}, only {} available'.format(
                required_bytes, report_id, unprocessed_byte_count))
        # we have enough bytes to read
        report_slice = packet.data[next_byte_index: next_byte_index + required_bytes]
        report_slices.append([report_slice[0], report_slice])
        next_byte_index = next_byte_index + required_bytes

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Packet:
    '''a class representing a Hillcrest Laboratory Sensor Hub Transport packet'''
    def __init__(self, packet_bytes: bytearray) -> None:
        self.header = self.header_from_buffer(packet_bytes)
        data_end_index = self.header.data_length + _BNO_HEADER_LEN
        self.data = packet_bytes[_BNO_HEADER_LEN: data_end_index]

    def __str__(self) -> str:
        '''format packet for debug output'''
        length = self.header.packet_byte_count
        lines = []
        lines.append('\n\t\t********** Packet *************')
        lines.append('HEADER:')
        lines.append('  data len: {}'.format(self.header.data_length))
        _ch_name = _CHANNEL_NAMES.get(self.channel_number, 'UNKNOWN')
        lines.append('  channel: {} ({})'.format(_ch_name, self.channel_number))
        if self.channel_number in {_BNO_CHANNEL_CONTROL, _BNO_CHANNEL_INPUT_SENSOR_REPORTS}:
            if self.report_id in _REPORT_NAMES:
                lines.append('    report type: {} (0x{:02X})'.format(_REPORT_NAMES[self.report_id], self.report_id))
            else:
                lines.append('    ** UNKNOWN report type **: {}'.format(hex(self.report_id)))
            if self.report_id > 0xF0 and len(self.data) >= 6 and self.data[5] in _REPORT_NAMES:
                lines.append('    sensor report type: {} ({})'.format(_REPORT_NAMES[self.data[5]], hex(self.data[5])))
            if self.report_id == 0xFC and len(self.data) >= 6 and self.data[1] in _REPORT_NAMES:
                lines.append('    enabled feature: {} ({})'.format(_REPORT_NAMES[self.data[1]], hex(self.data[5])))
        lines.append('  sequence number: {}'.format(self.header.sequence_number))
        lines.append('')
        lines.append('DATA: ')
        data_lines = []
        for idx, packet_byte in enumerate(self.data[:length]):
            packet_index = idx + 4
            if (packet_index % 4) == 0:
                if data_lines:
                    lines.append('  [0x{:02X}] {}'.format(packet_index - 4, ' '.join(data_lines)))
                    data_lines = []
            data_lines.append('0x{:02X}'.format(packet_byte))
        if data_lines:
            lines.append('  [0x{:02X}] {}'.format((len(self.data[:length]) // 4) * 4, ' '.join(data_lines)))
        lines.append('\t\t*******************************')
        return '\n'.join(lines)

    @property
    def report_id(self) -> int:
        '''the packet's report ID'''
        return self.data[0]

    @property
    def channel_number(self) -> int:
        '''the packet channel'''
        return self.header.channel_number

    @classmethod
    def header_from_buffer(cls, packet_bytes: bytearray) -> PacketHeader:
        '''creates a PacketHeader object from a given buffer'''
        packet_byte_count = unpack_from('<H', packet_bytes)[0]
        packet_byte_count &= ~0x8000
        channel_number = unpack_from('<B', packet_bytes, offset=2)[0]
        sequence_number = unpack_from('<B', packet_bytes, offset=3)[0]
        data_length = max(0, packet_byte_count - 4)
        header = PacketHeader(channel_number, sequence_number, data_length, packet_byte_count)
        return header

    @classmethod
    def is_error(cls, header: PacketHeader) -> bool:
        '''returns True if the header is an error condition'''
        if header.channel_number > 5:
            return True
        if header.packet_byte_count == 0xFFFF and header.sequence_number == 0xFF:
            return True
        return False

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class BNO085:
    '''
    BNO085 9-DoF IMU driver for CPython using SMBus2.

    Minimal base class with no external dependencies beyond smbus2.

    Args:
        i2c_id:      I2C bus number (default: 1)
        i2c_address: I2C device address (default: 0x4A)
    '''
    DEFAULT_I2C_ADDRESS = 0x4A

    def __init__(self, i2c_id=1, i2c_address=0x4A):
        self._i2c_bus_number = i2c_id
        self._i2c_address = i2c_address

        # sensor state
        self._enabled = False
        self._i2c = None
        self._dbuf = None
        self._data_buffer = None
        self._packet_slices = None
        self._command_buffer = bytearray(12)
        self._sequence_number = None
        self._readings = {}
        self._id_read = False

        # calibration state
        self._magnetometer_accuracy = 0
        self._gyro_accuracy = 0
        self._accel_accuracy = 0
        self._me_calibration_started_at = 0.0
        self._dcd_saved_at = 0.0
        self._two_ended_sequence_numbers = {}

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    # Properties

    @property
    def enabled(self):
        '''returns True if sensor is enabled'''
        return self._enabled

    @property
    def magnetic(self) -> Optional[tuple[float, float, float]]:
        '''tuple of current magnetic field measurements on X, Y, Z axes'''
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_MAGNETOMETER]
        except KeyError:
            raise RuntimeError('no magfield report found, is it enabled?') from None

    @property
    def quaternion(self) -> Optional[tuple[float, float, float, float]]:
        '''quaternion representing current rotation vector'''
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_ROTATION_VECTOR]
        except KeyError:
            raise RuntimeError('no quaternion report found, is it enabled?  ') from None

    @property
    def geomagnetic_quaternion(self) -> Optional[tuple[float, float, float, float]]:
        '''quaternion representing current geomagnetic rotation vector'''
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR]
        except KeyError:
            raise RuntimeError('no geomag quaternion report found, is it enabled?  ') from None

    @property
    def game_quaternion(self) -> Optional[tuple[float, float, float, float]]:
        '''
        quaternion representing current rotation vector with no specific reference for heading,
        while roll and pitch are referenced against gravity
        '''
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_GAME_ROTATION_VECTOR]
        except KeyError:
            raise RuntimeError('no game quaternion report found, is it enabled? ') from None

    @property
    def steps(self) -> Optional[int]:
        '''number of steps detected since sensor initialization'''
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_STEP_COUNTER]
        except KeyError:
            raise RuntimeError('no steps report found, is it enabled?') from None

    @property
    def linear_acceleration(self) -> Optional[tuple[float, float, float]]:
        '''tuple of current linear acceleration values on X, Y, Z axes in m/s²'''
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_LINEAR_ACCELERATION]
        except KeyError:
            raise RuntimeError('no lin. accel report found, is it enabled?  ') from None

    @property
    def acceleration(self) -> Optional[tuple[float, float, float]]:
        '''tuple of acceleration measurements on X, Y, Z axes in m/s²'''
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_ACCELEROMETER]
        except KeyError:
            raise RuntimeError('no accel report found, is it enabled?  ') from None

    @property
    def gravity(self) -> Optional[tuple[float, float, float]]:
        '''tuple of gravity vector in X, Y, Z components in m/s²'''
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_GRAVITY]
        except KeyError:
            raise RuntimeError('no gravity report found, is it enabled? ') from None

    @property
    def gyro(self) -> Optional[tuple[float, float, float]]:
        '''tuple of gyro rotation measurements on X, Y, Z axes in rad/s'''
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_GYROSCOPE]
        except KeyError:
            raise RuntimeError('no gyro report found, is it enabled?') from None

    @property
    def shake(self) -> Optional[bool]:
        '''
        True if shake detected on any axis since last check.
        latching behavior - clears on read
        '''
        self._process_available_packets()
        try:
            shake_detected = self._readings[BNO_REPORT_SHAKE_DETECTOR]
            # clear on read
            if shake_detected:
                self._readings[BNO_REPORT_SHAKE_DETECTOR] = False
            return shake_detected
        except KeyError:
            raise RuntimeError('no shake report found, is it enabled?') from None

    @property
    def stability_classification(self) -> Optional[str]:
        '''
        sensor's assessment of current stability:
        Unknown, On Table, Stationary, Stable, In motion
        '''
        self._process_available_packets()
        try:
            stability_classification = self._readings[BNO_REPORT_STABILITY_CLASSIFIER]
            return stability_classification
        except KeyError:
            raise RuntimeError('no stability classification report found, is it enabled?') from None

    @property
    def activity_classification(self) -> Optional[dict]:
        '''
        sensor's assessment of activity creating motions:
        Unknown, In-Vehicle, On-Bicycle, On-Foot, Still, Tilting, Walking, Running, OnStairs
        '''
        self._process_available_packets()
        try:
            activity_classification = self._readings[BNO_REPORT_ACTIVITY_CLASSIFIER]
            return activity_classification
        except KeyError:
            raise RuntimeError('no activity classification report found, is it enabled?') from None

    @property
    def raw_acceleration(self) -> Optional[tuple[int, int, int]]:
        '''raw, unscaled value from accelerometer registers'''
        self._process_available_packets()
        try:
            raw_acceleration = self._readings[BNO_REPORT_RAW_ACCELEROMETER]
            return raw_acceleration
        except KeyError:
            raise RuntimeError('no raw acceleration report found, is it enabled?') from None

    @property
    def raw_gyro(self) -> Optional[tuple[int, int, int]]:
        '''raw, unscaled value from gyro registers'''
        self._process_available_packets()
        try:
            raw_gyro = self._readings[BNO_REPORT_RAW_GYROSCOPE]
            return raw_gyro
        except KeyError:
            raise RuntimeError('no raw gyro report found, is it enabled? ') from None

    @property
    def raw_magnetic(self) -> Optional[tuple[int, int, int]]:
        '''raw, unscaled value from magnetometer registers'''
        self._process_available_packets()
        try:
            raw_magnetic = self._readings[BNO_REPORT_RAW_MAGNETOMETER]
            return raw_magnetic
        except KeyError:
            raise RuntimeError('no raw magnetic report found, is it enabled?') from None

    @property
    def calibration_status(self) -> int:
        '''status of self-calibration'''
        self._send_me_command(
            [
                0,  # calibrate accel
                0,  # calibrate gyro
                0,  # calibrate mag
                _ME_GET_CAL,
                0,  # calibrate planar acceleration
                0,  # 'on_table' calibration
                0,  # reserved
                0,  # reserved
                0,  # reserved
            ]
        )
        return self._magnetometer_accuracy

    @property
    def accelerometer(self):
        '''alias for acceleration to match ICM20948/USFS API'''
        return self.acceleration

    @property
    def gyroscope(self):
        '''alias for gyro to match ICM20948/USFS API'''
        return self.gyro

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    # Public methods

    def enable(self):
        '''enable the hardware for the sensor'''
        if self._enabled:
            print('WARNING: BNO085 already enabled')
            return

        # initialize I2C and sensor
        self._i2c = SMBus(self._i2c_bus_number)

        # initialize buffers
        self._dbuf = bytearray(2)
        self._data_buffer = bytearray(_DATA_BUFFER_SIZE)
        self._packet_slices: list[Any] = []
        self._sequence_number = [0] * (_BNO_CHANNEL_GYRO_ROTATION_VECTOR + 1)

        # reset and check ID
        for attempt in range(3):
            self.soft_reset()
            try:
                if self._check_id():
                    break
            except Exception as e:
                print('ERROR: BNO085 ID check failed (attempt {}): {}'.format(attempt + 1, e))
                time.sleep(0.5)
        else:
            raise RuntimeError('could not read BNO085 ID')

        # enable default sensor features
        self.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.enable_feature(BNO_REPORT_GYROSCOPE)
        self.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        self._enabled = True

    def disable(self):
        '''disable sensor and close I2C bus'''
        if not self._enabled:
            print('WARNING: BNO085 already disabled')
            return

        if self._i2c:
            self._i2c.close()
            self._i2c = None

        self._enabled = False

    def update(self):
        '''
        read hardware sensor reports and update cached data.
        call this before accessing sensor properties.
        '''
        if not self._enabled:
            print('WARNING: BNO085 not enabled, cannot update')
            return

        # access properties to trigger internal updates
        _ = self.quaternion
        _ = self.acceleration
        _ = self.gyro
        _ = self.magnetic

    def begin_calibration(self) -> None:
        '''begin sensor's self-calibration routine'''
        # start calibration for accel, gyro, and mag
        self._send_me_command(
            [
                1,  # calibrate accel
                1,  # calibrate gyro
                1,  # calibrate mag
                _ME_CAL_CONFIG,
                0,  # calibrate planar acceleration
                0,  # 'on_table' calibration
                0,  # reserved
                0,  # reserved
                0,  # reserved
            ]
        )

    def save_calibration_data(self) -> None:
        '''save self-calibration data to flash'''
        start_time = time.monotonic()
        local_buffer = bytearray(12)
        _insert_command_request_report(
            _SAVE_DCD,
            local_buffer,
            self._get_report_seq_id(_COMMAND_REQUEST),
        )
        self._send_packet(_BNO_CHANNEL_CONTROL, local_buffer)
        self._increment_report_seq(_COMMAND_REQUEST)
        while _elapsed(start_time) < _DEFAULT_TIMEOUT:
            self._process_available_packets()
            if self._dcd_saved_at > start_time:
                return
        raise RuntimeError('could not save calibration data')

    def enable_feature(self, feature_id: int, report_interval: int = _DEFAULT_REPORT_INTERVAL) -> None:
        '''enable a given feature of the BNO08x'''
        if feature_id == BNO_REPORT_ACTIVITY_CLASSIFIER:
            set_feature_report = self._get_feature_enable_report(
                feature_id, report_interval, _ENABLED_ACTIVITIES
            )
        else:
            set_feature_report = self._get_feature_enable_report(feature_id, report_interval)

        feature_dependency = _RAW_REPORTS.get(feature_id, None)
        if feature_dependency and feature_dependency not in self._readings:
            self.enable_feature(feature_dependency)

        self._send_packet(_BNO_CHANNEL_CONTROL, set_feature_report)

        start_time = time.monotonic()
        while _elapsed(start_time) < _FEATURE_ENABLE_TIMEOUT:
            self._process_available_packets(max_packets=10)
            if feature_id in self._readings:
                return
        raise RuntimeError('was not able to enable feature {}'.format(feature_id))

    def soft_reset(self) -> None:
        '''reset sensor to initial unconfigured state'''
        data = bytearray(1)
        data[0] = 1
        self._send_packet(_BNO_CHANNEL_EXE, data)
        time.sleep(0.5)
        self._send_packet(_BNO_CHANNEL_EXE, data)
        time.sleep(0.5)

        for _i in range(3):
            try:
                _packet = self._read_packet()
            except PacketError:
                time.sleep(0.5)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    # Internal methods

    def _send_me_command(self, subcommand_params: Optional[list[int]]) -> None:
        start_time = time.monotonic()
        local_buffer = self._command_buffer
        _insert_command_request_report(
            _ME_CALIBRATE,
            self._command_buffer,
            self._get_report_seq_id(_COMMAND_REQUEST),
            subcommand_params,
        )
        self._send_packet(_BNO_CHANNEL_CONTROL, local_buffer)
        self._increment_report_seq(_COMMAND_REQUEST)
        while _elapsed(start_time) < _DEFAULT_TIMEOUT:
            self._process_available_packets()
            if self._me_calibration_started_at > start_time:
                break

    def _process_available_packets(self, max_packets: Optional[int] = None) -> None:
        processed_count = 0
        while self._data_ready:
            if max_packets and processed_count > max_packets:
                return
            try:
                new_packet = self._read_packet()
            except PacketError:
                continue
            self._handle_packet(new_packet)
            processed_count += 1

    def _wait_for_packet_type(
        self, channel_number: int, report_id: Optional[int] = None, timeout: float = 5.0
    ) -> Packet:
        start_time = time.monotonic()
        while _elapsed(start_time) < timeout:
            new_packet = self._wait_for_packet()
            if new_packet.channel_number == channel_number:
                if report_id:
                    if new_packet.report_id == report_id:
                        return new_packet
                else:
                    return new_packet
            if new_packet.channel_number not in {_BNO_CHANNEL_EXE, _BNO_CHANNEL_SHTP_COMMAND}:
                self._handle_packet(new_packet)
        raise RuntimeError('timed out waiting for a packet on channel {}'.format(channel_number))

    def _wait_for_packet(self, timeout: float = _PACKET_READ_TIMEOUT) -> Packet:
        start_time = time.monotonic()
        while _elapsed(start_time) < timeout:
            if not self._data_ready:
                continue
            new_packet = self._read_packet()
            return new_packet
        raise RuntimeError('timed out waiting for a packet')

    def _update_sequence_number(self, new_packet: Packet) -> None:
        channel = new_packet.channel_number
        seq = new_packet.header.sequence_number
        self._sequence_number[channel] = seq

    def _handle_packet(self, packet: Packet) -> None:
        try:
            _separate_batch(packet, self._packet_slices)
            while len(self._packet_slices) > 0:
                self._process_report(*self._packet_slices.pop())
        except PacketError as pe:
            # recoverable packet error
            print('WARNING: packet error (recoverable): {}'.format(pe))
            self._packet_slices.clear()
        except Exception as error:
            print('ERROR: error handling packet: {}'.format(error))
            self._packet_slices.clear()

    def _handle_control_report(self, report_id: int, report_bytes: bytearray) -> None:
        if report_id == _SHTP_REPORT_PRODUCT_ID_RESPONSE:
            (
                sw_part_number,
                sw_major,
                sw_minor,
                sw_patch,
                sw_build_number,
            ) = parse_sensor_id(report_bytes)

        if report_id == _GET_FEATURE_RESPONSE:
            get_feature_report = _parse_get_feature_response_report(report_bytes)
            _report_id, feature_report_id, *_remainder = get_feature_report
            self._readings[feature_report_id] = _INITIAL_REPORTS.get(
                feature_report_id, (0.0, 0.0, 0.0)
            )
        if report_id == _COMMAND_RESPONSE:
            self._handle_command_response(report_bytes)

    def _handle_command_response(self, report_bytes: bytearray) -> None:
        (report_body, response_values) = _parse_command_response(report_bytes)
        (
            _report_id,
            _seq_number,
            command,
            _command_seq_number,
            _response_seq_number,
        ) = report_body
        command_status, *_rest = response_values
        if command == _ME_CALIBRATE and command_status == 0:
            self._me_calibration_started_at = time.monotonic()
        if command == _SAVE_DCD:
            if command_status == 0:
                self._dcd_saved_at = time.monotonic()
            else:
                raise RuntimeError('unable to save calibration data')

    def _process_report(self, report_id: int, report_bytes: bytearray) -> None:
        if report_id >= 0xF0:
            self._handle_control_report(report_id, report_bytes)
            return

        if report_id == BNO_REPORT_STEP_COUNTER:
            self._readings[report_id] = _parse_step_couter_report(report_bytes)
            return
        if report_id == BNO_REPORT_SHAKE_DETECTOR:
            shake_detected = _parse_shake_report(report_bytes)
            try:
                if not self._readings[BNO_REPORT_SHAKE_DETECTOR]:
                    self._readings[BNO_REPORT_SHAKE_DETECTOR] = shake_detected
            except KeyError:
                pass
            return
        if report_id == BNO_REPORT_STABILITY_CLASSIFIER:
            stability_classification = _parse_stability_classifier_report(report_bytes)
            self._readings[BNO_REPORT_STABILITY_CLASSIFIER] = stability_classification
            return
        if report_id == BNO_REPORT_ACTIVITY_CLASSIFIER:
            activity_classification = _parse_activity_classifier_report(report_bytes)
            self._readings[BNO_REPORT_ACTIVITY_CLASSIFIER] = activity_classification
            return
        sensor_data, accuracy = _parse_sensor_report_data(report_bytes)
        if report_id == BNO_REPORT_MAGNETOMETER:
            self._magnetometer_accuracy = accuracy
        self._readings[report_id] = sensor_data

    @staticmethod
    def _get_feature_enable_report(
        feature_id: int,
        report_interval: int,
        sensor_specific_config: int = 0,
    ) -> bytearray:
        set_feature_report = bytearray(17)
        set_feature_report[0] = _SET_FEATURE_COMMAND
        set_feature_report[1] = feature_id
        pack_into('<I', set_feature_report, 5, report_interval)
        pack_into('<I', set_feature_report, 13, sensor_specific_config)
        return set_feature_report

    def _check_id(self) -> bool:
        if self._id_read:
            return True
        data = bytearray(2)
        data[0] = _SHTP_REPORT_PRODUCT_ID_REQUEST
        data[1] = 0  # padding
        self._send_packet(_BNO_CHANNEL_CONTROL, data)
        while True:
            self._wait_for_packet_type(_BNO_CHANNEL_CONTROL, _SHTP_REPORT_PRODUCT_ID_RESPONSE)
            sensor_id = self._parse_sensor_id()
            if sensor_id:
                self._id_read = True
                return True

    def _parse_sensor_id(self) -> Optional[int]:
        if not self._data_buffer[4] == _SHTP_REPORT_PRODUCT_ID_RESPONSE:
            return None
        sw_major = self._get_data(2, '<B')
        sw_minor = self._get_data(3, '<B')
        sw_patch = self._get_data(12, '<H')
        sw_part_number = self._get_data(4, '<I')
        sw_build_number = self._get_data(8, '<I')
        return sw_part_number

    def _get_data(self, index: int, fmt_string: str) -> Any:
        # index arg is not including header, so add 4 into data buffer
        data_index = index + 4
        return unpack_from(fmt_string, self._data_buffer, offset=data_index)[0]

    @property
    def _data_ready(self):
        header = self._read_header()
        if header.channel_number > 5:
            return False
        if header.packet_byte_count == 0x7FFF:
            if header.sequence_number == 0xFF:
                return False
        return header.data_length > 0

    def _send_packet(self, channel: int, data: bytearray) -> int:
        data_length = len(data)
        write_length = data_length + 4

        pack_into('<H', self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel
        self._data_buffer[3] = self._sequence_number[channel]
        for idx, send_byte in enumerate(data):
            self._data_buffer[4 + idx] = send_byte

        try:
            write_msg = i2c_msg.write(self._i2c_address, list(self._data_buffer[:write_length]))
            self._i2c.i2c_rdwr(write_msg)
        except Exception as e:
            print('ERROR: I2C write error: {}'.format(e))
            raise

        self._sequence_number[channel] = (self._sequence_number[channel] + 1) % 256
        return self._sequence_number[channel]

    def _read_header(self):
        '''reads the first 4 bytes available as a header'''
        try:
            read_msg = i2c_msg.read(self._i2c_address, 4)
            self._i2c.i2c_rdwr(read_msg)
            for i, byte in enumerate(read_msg):
                self._data_buffer[i] = byte
        except Exception as e:
            print('ERROR: I2C read header error: {}'.format(e))
            raise
        packet_header = Packet.header_from_buffer(self._data_buffer)
        return packet_header

    def _read_packet(self) -> Packet:
        # read header first
        try:
            read_msg = i2c_msg.read(self._i2c_address, 4)
            self._i2c.i2c_rdwr(read_msg)
            for i, byte in enumerate(read_msg):
                self._data_buffer[i] = byte
        except Exception as e:
            print('ERROR: I2C read packet header error: {}'.format(e))
            raise

        header = Packet.header_from_buffer(self._data_buffer)
        packet_byte_count = header.packet_byte_count
        channel_number = header.channel_number
        sequence_number = header.sequence_number

        self._sequence_number[channel_number] = sequence_number
        if packet_byte_count == 0:
            raise PacketError('no packet available')
        packet_byte_count -= 4

        self._read(packet_byte_count)

        new_packet = Packet(self._data_buffer)
        self._update_sequence_number(new_packet)
        return new_packet

    def _read(self, requested_read_length):
        '''returns true if all requested data was read'''
        total_read_length = requested_read_length + 4
        if total_read_length > _DATA_BUFFER_SIZE:
            self._data_buffer = bytearray(total_read_length)
            print('WARNING: increased _data_buffer to bytearray({})'.format(total_read_length))
        try:
            read_msg = i2c_msg.read(self._i2c_address, total_read_length)
            self._i2c.i2c_rdwr(read_msg)
            for i, byte in enumerate(read_msg):
                self._data_buffer[i] = byte
        except Exception as e:
            print('ERROR: I2C read error: {}'.format(e))
            raise

    def _increment_report_seq(self, report_id: int) -> None:
        current = self._two_ended_sequence_numbers.get(report_id, 0)
        self._two_ended_sequence_numbers[report_id] = (current + 1) % 256

    def _get_report_seq_id(self, report_id: int) -> int:
        return self._two_ended_sequence_numbers.get(report_id, 0)

#EOF
