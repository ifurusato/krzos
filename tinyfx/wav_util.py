#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-11-25
# modified: 2025-11-25

import struct

def wav_duration(path):
    """
    Return duration in seconds for a PCM WAV file.
    """
    with open(path, 'rb') as f:
        header = f.read(44)  # standard WAV header size
    # WAV header fields (little-endian)
    num_channels     = struct.unpack('<H', header[22:24])[0]
    sample_rate      = struct.unpack('<I', header[24:28])[0]
    bits_per_sample  = struct.unpack('<H', header[34:36])[0]
    data_size        = struct.unpack('<I', header[40:44])[0]
    bytes_per_frame = (bits_per_sample // 8) * num_channels
    num_frames = data_size / bytes_per_frame
    return num_frames / sample_rate

#EOF
