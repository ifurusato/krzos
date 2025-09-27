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

import numpy as np
import json
import csv
import cv2
import datetime as dt
import depthai as dai
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.component import Component
from core.orientation import Orientation

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DepthCamera(Component):
    '''
    Component wrapper for OAK-D Lite depth-only acquisition using DepthAI 3.x.

    The camera must be enabled to start the pipeline.

    :param config:      application configuration (currently unused)
    :param level:       the logging Level
    '''
    def __init__(self, config=None, suppressed=False, enabled=False, level=Level.INFO):
        self._log = Logger('depth-camera', level)
        Component.__init__(self, self._log, suppressed=suppressed, enabled=enabled)

        # configuration
        EXTENDED_DISPARITY = False   # closer-in minimum depth, disparity range is doubled (from 95 to 190)
        SUBPIXEL = False             # better accuracy for longer distance, fractional disparity 32-levels
        LR_CHECK = True              # better handling for occlusions

        # create pipeline
        self._pipeline = dai.Pipeline()
        # color camera
        color_cam  = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        # mono cameras and stereo depth node
        mono_left  = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
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
        stereo.setRectification(True)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        # options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.setLeftRightCheck(LR_CHECK)
        stereo.setExtendedDisparity(EXTENDED_DISPARITY)
        stereo.setSubpixel(SUBPIXEL)
        # output queues
        self._mono_left_queue  = mono_left_out.createOutputQueue()
        self._mono_right_queue = mono_right_out.createOutputQueue()
        self._depth_queue      = stereo.depth.createOutputQueue()
        self._log.info('DepthCamera initialized and pipeline started.')

    def get_depth_frame_size(self):
        '''
        Returns the size (height, width) of the latest depth frame as a tuple.
        Returns None if no frame is available.
        '''
        if not self.enabled:
            self._log.warning('depth camera not enabled.')
            return None
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
        if not self.enabled:
            self._log.warning('depth camera not enabled.')
            return None
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

    def get_frame(self, orientation=None):
        '''
        Returns either the PORT or STBD greyscale camera frame as an array 
        of uint8 values, where each value is a greyscale intensity of 0-255.
        '''
        if orientation is Orientation.PORT:
            port_frame = self._mono_left_queue.get().getFrame() if self._mono_left_queue is not None else None
            return port_frame
        elif orientation is Orientation.STBD:
            stbd_frame = self._mono_right_queue.get().getFrame() if self._mono_right_queue is not None else None
            return stbd_frame
        else:
            self._log.warning('unrecognised frame orientation.')

    def get_depth_frame(self):
        '''
        Gets the latest depth frame as a numpy array of uint16 values, 
        where each is depth value in millimeters.
        Returns None if no frame is available. Note that values may be zero 
        if the camera is unable to determine a depth value.
        '''
        if not self.enabled:
            self._log.warning('depth camera not enabled.')
            return None
        start = dt.datetime.now()
        depth_frame = self._depth_queue.get()
        end = dt.datetime.now()
        elapsed = (end - start).total_seconds() * 1000
        self._log.info(Fore.MAGENTA + "get depth frame: {:.2f}ms elapsed.".format(elapsed))
        if depth_frame is not None:
            np_depth = depth_frame.getFrame() # uint16, depth in mm
            return np_depth
        else:
            self._log.debug('no depth frame available at this time.')
            return None

    def save_depth_frame_json(self, filename):
        '''
        Saves the latest depth frame as a JSON file.
        '''
        if not self.enabled:
            self._log.warning('depth camera not enabled.')
            return False
        np_depth = self.get_depth_frame()
        if np_depth is not None:
            with open(filename, "w") as f:
                json.dump(np_depth.tolist(), f)
            self._log.info('depth frame saved to {} (JSON).'.format(filename))
            return True
        else:
            self._log.warning('no depth frame to save.')
            return False

    def save_depth_frame_csv(self, filename):
        '''
        Saves the latest depth frame as a CSV file.
        '''
        if not self.enabled:
            self._log.warning('depth camera not enabled.')
            return False
        np_depth = self.get_depth_frame()
        if np_depth is not None:
            np.savetxt(filename, np_depth, fmt='%d', delimiter=",")
            self._log.info('depth frame saved to {} (CSV).'.format(filename))
            return True
        else:
            self._log.warning('no depth frame to save.')
            return False

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def save_camera_frames_png(self, base_filename="output"):
        '''
        Saves PNG images for left, right, and depth frames from the pipeline.
        Filenames will be {base_filename}_left.png, {base_filename}_right.png, {base_filename}_depth.png
        Returns True if all frames were saved, False otherwise.
        '''
        port_frame  = self.get_frame(Orientation.PORT)
        stbd_frame = self.get_frame(Orientation.STBD)
        depth_frame = self.get_depth_frame()

        height, width = 480, 640 

        def nv12_to_gray(nv12_frame, width, height):
            # Y plane is first 'height' rows and first 'width' columns
            return nv12_frame[:height, :width]

        success = True
        if port_frame is not None:
            self._log.info("port NV12 shape: {}".format(port_frame.shape))
            left_gray = nv12_to_gray(port_frame, width, height)
            cv2.imwrite(f"{base_filename}_port.png", left_gray)
        else:
            self._log.warning("No left frame available to save.")
            success = False
        if stbd_frame is not None:
            self._log.info("stbd NV12 shape: {}".format(stbd_frame.shape))
            right_gray = nv12_to_gray(stbd_frame, width, height)
            cv2.imwrite(f"{base_filename}_stbd.png", right_gray)
        else:
            self._log.warning("No right frame available to save.")
            success = False
        if depth_frame is not None:
            depth_normalized = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
            depth_uint8 = depth_normalized.astype('uint8')
            cv2.imwrite(f"{base_filename}_depth.png", depth_uint8)
        else:
            self._log.warning("No depth frame available to save.")
            success = False
        return success

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def enable(self):
        self._log.info('enabling depth camera…')
        Component.enable(self)
        # start pipeline
        self._pipeline.start()

    def disable(self):
        self._log.info('disabling depth camera…')
        Component.disable(self)
        # stop pipeline
        self._pipeline.stop()

#EOF
