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
from datetime import datetime as dt
import depthai as dai
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.component import Component
from core.orientation import Orientation
from hardware.depth_utils import DepthUtils

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DepthCamera(Component):
    '''
    Component wrapper for OAK-D Lite depth-only acquisition using DepthAI 3.x.

    :param config:      application configuration (currently unused)
    :param level:       the logging Level
    '''
    def __init__(self, config=None, suppressed=False, enabled=True, level=Level.INFO):
        self._log = Logger('depth-camera', level)
        Component.__init__(self, self._log, suppressed=suppressed, enabled=enabled)
        _start_time = dt.now()
        # configuration
        _cfg = config.get('kros').get('hardware').get('depth_camera')
        _preset_mode_value  = _cfg.get('preset_mode')
        _preset_mode = DepthCamera.parse_preset_mode(_preset_mode_value, default=dai.node.StereoDepth.PresetMode.ROBOTICS)
        _extended_disparity = _cfg.get('extended_disparity')  # closer-in minimum depth, disparity range is doubled (from 95 to 190)
        _subpixel    =  _cfg.get('subpixel')                  # better accuracy for longer distance, fractional disparity 32-levels
        _lr_check    =  _cfg.get('left_right_check')          # better handling for occlusions
        self.scale_pixel_coordinates = True
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
        stereo.setDefaultProfilePreset(_preset_mode)
        # options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.setLeftRightCheck(_lr_check)
        stereo.setExtendedDisparity(_extended_disparity)
        stereo.setSubpixel(_subpixel)
        # output queues
        self._mono_left_queue  = mono_left_out.createOutputQueue()
        self._mono_right_queue = mono_right_out.createOutputQueue()
        self._depth_queue      = stereo.depth.createOutputQueue()
        # start pipeline
        self._pipeline.start()
        _elapsed_ms = round((dt.now() - _start_time).total_seconds() * 1000.0)
        self._log.info('depth camera initialized and pipeline started: {}ms elapsed.'.format(_elapsed_ms))

    @staticmethod
    def parse_preset_mode(mode_str, default=None):
        '''
        Convert a string to dai.node.StereoDepth.PresetMode enum.
        Accepts values like 'DEFAULT', 'FACE', etc.
        Returns `default` if input is invalid or not recognized.
        If `default` is None, returns dai.node.StereoDepth.PresetMode.DEFAULT.
        '''
        try:
            mode_str = mode_str.strip().upper()
            return getattr(dai.node.StereoDepth.PresetMode, mode_str)
        except Exception:
            if default is not None:
                return default
            import depthai as dai  # in case not already imported
            return dai.node.StereoDepth.PresetMode.DEFAULT

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

    def get_pixel_depths(self, points):
        '''
        Returns a numpy array of depth values (in mm) for a list/array of (x, y) pixel coordinates.
        Each input should be a tuple/list (x, y) or a numpy array of shape (N, 2).
        Returns np.nan for points that are out of bounds or if no frame is available.
        Parameters:
            points (list or np.ndarray): List or array of (x, y) pixel coordinates.
        Returns:
            np.ndarray: Array of depth values (in mm) with shape (N,). np.nan for invalid points.
        '''
        import numpy as np
        if not self.enabled:
            self._log.warning('depth camera not enabled.')
            return np.full(len(points), np.nan)
        np_depth = self.get_depth_frame()
        if np_depth is None:
            self._log.warning('no depth frame available for pixel query.')
            return np.full(len(points), np.nan)
        points = np.asarray(points)
        # split into x and y arrays
        x_arr = points[:, 0].astype(int)
        y_arr = points[:, 1].astype(int)
        # mask for in-bounds points
        h, w = np_depth.shape
        in_bounds = (x_arr >= 0) & (x_arr < w) & (y_arr >= 0) & (y_arr < h)
        depths = np.full(len(points), np.nan)
        # only assign depth for in-bounds points
        depths[in_bounds] = np_depth[y_arr[in_bounds], x_arr[in_bounds]]
        return depths

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
            return self._mono_left_queue.get().getFrame() if self._mono_left_queue is not None else None
        elif orientation is Orientation.STBD:
            return self._mono_right_queue.get().getFrame() if self._mono_right_queue is not None else None
        else:
            raise ValueError('unrecognised frame orientation.')

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
        _start_time = dt.now()
        depth_frame = self._depth_queue.get()
        _end_time   = dt.now()
        _elapsed_ms = (_end_time - _start_time).total_seconds() * 1000
        self._log.info(Fore.MAGENTA + Style.DIM + "get depth frame: {:.2f}ms elapsed.".format(_elapsed_ms))
        if depth_frame is not None:
            return depth_frame.getFrame() # uint16, depth in mm
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

    def get_reference_pixel_grid(self):
        '''
        Returns a numpy array of (x, y) pixel coordinates for the normalized reference grid
        (in 640x480 calibration space), e.g., 15 rows × 5 columns = 75 points.
        Returns:
            np.ndarray: Array of shape (75, 2) [(x, y), ...]
        '''
        import numpy as np
        rows = [256, 299, 302, 305, 309, 314, 319, 326, 334, 343, 356, 372, 394, 427, 479]
        cols = [20, 170, 320, 470, 620]
        grid = [(x, y) for y in rows for x in cols]
        return np.array(grid)

    def get_reference_depth_grid(self):
        '''
        Uses the reference pixel grid and queries the current depth frame for those points,
        returning a (15, 5) numpy array of depths (in mm), np.nan for invalid/missing points.
        Returns:
            np.ndarray: Depth grid, shape (15, 5)
        '''
        _start_time = dt.now()
        grid = self.get_reference_pixel_grid()
        np_depth = self.get_depth_frame()
        if np_depth is None:
            self._log.warning('no depth frame available for grid query.')
            return np.full((15, 5), np.nan)
        height, width = np_depth.shape
        # Optionally scale if enabled
        if self.scale_pixel_coordinates:
            x_scaled = np.round(grid[:, 0] * (width / 640)).astype(int)
            y_scaled = np.round(grid[:, 1] * (height / 480)).astype(int)
            points = np.stack([x_scaled, y_scaled], axis=1)
        else:
            points = grid.astype(int)
        depths = self.get_pixel_depths(points)
        _elapsed_ms = round((dt.now() - _start_time).total_seconds() * 1000.0)
        self._log.info(Fore.WHITE + 'depth camera returned reference depth: {}ms elapsed.'.format(_elapsed_ms))
        return depths.reshape((15, 5))

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

    def ground_to_pixel(self, X, Z, ref_width=640, ref_height=480):
        '''
        Given ground X (cm), Z (cm), return (x, y) image pixel coordinates
        (in depth frame space, as integers).
        If scale_pixel_coordinates is True, scales calibration-space pixel to actual frame size.
        '''
        # get calibration-space pixel coordinates
        x_calib, y_calib = DepthUtils.ground_to_pixel(X, Z)
        np_depth = self.get_depth_frame()
        if np_depth is None:
            self._log.warning('no depth frame available.')
            return None
        height, width = np_depth.shape
        if self.scale_pixel_coordinates:
            x = int(round(x_calib * (width / ref_width)))
            y = int(round(y_calib * (height / ref_height)))
        else:
            x = int(round(x_calib))
            y = int(round(y_calib))
        if 0 <= y < height and 0 <= x < width:
            return x, y
        else:
            self._log.warning(f'pixel ({x},{y}) out of bounds for shape {np_depth.shape}.')
            return None

    def get_ground_point(self, x, y, ref_width=640, ref_height=480):
        '''
        Returns (X, Z, depth_mm) for the given image pixel (x, y).
        If scale_pixel_coordinates is True, (x, y) are assumed to be in reference coordinates
        and will be scaled to the actual depth frame size.
        '''
        if not self.enabled:
            self._log.warning('depth camera not enabled.')
            return None
        np_depth = self.get_depth_frame()
        if np_depth is None:
            self._log.warning('no depth frame available.')
            return None
        height, width = np_depth.shape  # (rows, cols)
        # optionally scale input (x, y) from reference calibration space to actual frame
        if self.scale_pixel_coordinates:
            x_scaled = int(round(x * (width / ref_width)))
            y_scaled = int(round(y * (height / ref_height)))
        else:
            x_scaled = x
            y_scaled = y
        if 0 <= y_scaled < height and 0 <= x_scaled < width:
            depth_mm = int(np_depth[y_scaled, x_scaled])
            # Use calibration-space coordinates for DepthUtils
            if self.scale_pixel_coordinates:
                X, Z = DepthUtils.pixel_to_ground(x, y)
            else:
                X, Z = DepthUtils.pixel_to_ground(x_scaled, y_scaled)
            return (X, Z, depth_mm)
        else:
            self._log.warning(f'pixel coordinates ({x_scaled},{y_scaled}) out of bounds for shape {np_depth.shape}.')
            return None

    def get_depth_at_ground(self, X, Z):
        '''
        Returns the measured depth value (in millimeters) from the depth frame
        at the pixel corresponding to the specified ground coordinates (X, Z).

        Parameters:
            X (float): Lateral ground coordinate in centimeters (cm), relative to camera calibration.
            Z (float): Forward ground coordinate in centimeters (cm), relative to camera calibration.

        Returns:
            int or None: Measured depth at the mapped pixel (in mm), or None if mapping is invalid or no data available.
        '''
        pixel = self.ground_to_pixel(X, Z)
        if pixel is not None:
            x, y = pixel
            return self.get_pixel_depth(x, y)
        else:
            return None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def enable(self):
        self._log.info('enabling depth camera…')
        Component.enable(self)

    def disable(self):
        self._log.info('disabling depth camera…')
        Component.disable(self)
        # stop pipeline
        self._pipeline.stop()

#EOF
