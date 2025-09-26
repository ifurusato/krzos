#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License.
#
# author:   Murray Altheim
# created:  2025-09-26
#
# DepthCamera class for OAK-D Lite providing depth frames via DepthAI 3.x API with Script node host output.
#

import depthai as dai
import numpy as np
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
        self._pipeline = dai.Pipeline()

        # Mono cameras and stereo depth node
        mono_left = self._pipeline.create(dai.node.MonoCamera)
        mono_right = self._pipeline.create(dai.node.MonoCamera)
        stereo = self._pipeline.create(dai.node.StereoDepth)
        xout = self._pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("depth")

        # Configure mono cameras
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        # Configure stereo depth
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        stereo.depth.link(xout.input)

        # Device and queue
        self._device = dai.Device(self._pipeline)
        self._depth_queue = self._device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        self._log.info('DepthCamera initialized and ready.')

    def get_depth_frame(self):
        '''
        Gets the latest depth frame as a numpy array.
        Returns None if no frame is available.
        '''
        if not self.enabled:
            self._log.warning('DepthCamera is not enabled.')
            return None

        in_depth = self._depth_queue.tryGet()
        if in_depth is not None:
            depth_frame = in_depth.getFrame()
            return depth_frame
        else:
            self._log.debug('No depth frame available at this time.')
            return None

    def enable(self):
        self._log.info('Enabling DepthCamera.')
        Component.enable(self)

    def disable(self):
        self._log.info('Disabling DepthCamera.')
        Component.disable(self)

#EOF
