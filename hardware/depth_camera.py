#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License.
#
# author:   Murray Altheim
# created:  2025-09-26
#
# DepthCamera class for OAK-D Lite providing depth frames via DepthAI 3.x API.
#
# Configure stereo depth preset mode:
#   DEFAULT:       Standard balanced preset.
#   FACE:          Tuned for face detection use cases.
#   FAST_ACCURACY: Faster processing with more accuracy.
#   FAST_DENSITY:  Faster processing with more dense (more points) output.
#   HIGH_DETAIL:   Highest detail depth map, slower but more precise.
#   ROBOTICS:      Tuned for robotics applications, usually with wider depth range and robustness.
#
# DepthAI Python API: https://docs.luxonis.com/software-v3/depthai/api/python/
#

import depthai as dai
import numpy as np
import json
import csv
from core.logger import Level, Logger
from core.component import Component

class DepthCamera(Component):
    '''
    Component wrapper for OAK-D Lite depth-only acquisition using DepthAI 3.x.

    :param config:      application configuration (unused here, but for pattern)
    :param level:       the logging Level
    '''
    def __init__(self, config=None, suppressed=False, enabled=True, level=Level.INFO):
        self._log = Logger('depth-camera', level)
        Component.__init__(self, self._log, suppressed=suppressed, enabled=enabled)

        # closer-in minimum depth, disparity range is doubled (from 95 to 190):
        extended_disparity = False
        # better accuracy for longer distance, fractional disparity 32-levels:
        subpixel = False
        # better handling for occlusions:
        lr_check = True

        # create pipeline
        self._pipeline = dai.Pipeline()
        # mono cameras and stereo depth node
        mono_left = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        mono_right = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        stereo = self._pipeline.create(dai.node.StereoDepth)
        # linking
        mono_left_out  = mono_left.requestFullResolutionOutput(type=dai.ImgFrame.Type.NV12)
        mono_right_out = mono_right.requestFullResolutionOutput(type=dai.ImgFrame.Type.NV12)
        # link mono outputs to stereo node
        mono_left_out.link(stereo.left)
        mono_right_out.link(stereo.right)

        # set stereo features
        # create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
#       stereo.setRectification(True)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
        # options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.setLeftRightCheck(lr_check)
        stereo.setExtendedDisparity(extended_disparity)
        stereo.setSubpixel(subpixel)

        # output queue
        self._depth_queue = stereo.depth.createOutputQueue()
        # start pipeline
        self._pipeline_handle = self._pipeline
        self._pipeline_handle.start()
        self._log.info('DepthCamera initialized and pipeline started.')

    def get_depth_frame_size(self):
        '''
        Returns the size (height, width) of the latest depth frame as a tuple.
        Returns None if no frame is available.
        '''
        np_depth = self.get_depth_frame()
        if np_depth is not None:
            return np_depth.shape
        else:
            self._log.warning('no depth frame available to get size.')
            return None

    def get_pixel_depth(self, x, y):
        '''
        Returns the depth (in mm) for pixel (x, y) from the latest depth frame.
        Returns None if no frame is available or the coordinates are out of bounds.
        '''
        np_depth = self.get_depth_frame()
        if np_depth is not None:
            if 0 <= y < np_depth.shape[0] and 0 <= x < np_depth.shape[1]:
                return int(np_depth[y, x])
            else:
                self._log.warning('pixel coordinates ({},{}) out of bounds for shape {}.'.format(x, y, np_depth.shape))
                return None
        else:
            self._log.warning('no depth frame available for pixel query.')
            return None

    def get_depth_frame(self):
        '''
        Gets the latest depth frame as a numpy array.
        Returns None if no frame is available.
        '''
        if not self.enabled:
            self._log.warning('DepthCamera is not enabled.')
            return None
        depth_frame = self._depth_queue.get()
        if depth_frame is not None:
            np_depth = depth_frame.getFrame() # uint16, depth in mm
            return np_depth
        else:
            self._log.debug('No depth frame available at this time.')
            return None

    def save_depth_frame_json(self, filename):
        '''
        Saves the latest depth frame as a JSON file.
        '''
        np_depth = self.get_depth_frame()
        if np_depth is not None:
            with open(filename, "w") as f:
                json.dump(np_depth.tolist(), f)
            self._log.info(f'Depth frame saved to {filename} (JSON).')
            return True
        else:
            self._log.warning('No depth frame to save.')
            return False

    def save_depth_frame_csv(self, filename):
        '''
        Saves the latest depth frame as a CSV file.
        '''
        np_depth = self.get_depth_frame()
        if np_depth is not None:
            np.savetxt(filename, np_depth, fmt='%d', delimiter=",")
            self._log.info(f'Depth frame saved to {filename} (CSV).')
            return True
        else:
            self._log.warning('No depth frame to save.')
            return False

    def enable(self):
        self._log.info('Enabling DepthCamera.')
        Component.enable(self)

    def disable(self):
        self._log.info('Disabling DepthCamera.')
        Component.disable(self)

#EOF
