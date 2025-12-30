#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2025-09-27
# modified: 2025-09-27
#

import numpy as np
from scipy.interpolate import interp1d

class DepthUtils:
    x_326 = np.array([67, 111, 154, 194, 233, 276, 320, 367, 414, 457, 505, 552, 594])
    X_326 = np.array([-60, -50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60])
    x_430 = np.array([76, 193, 320, 423, 537])
    X_430 = np.array([-20, -10, 0, 10, 20])
    x_interp = {
        326: interp1d(x_326, X_326, kind='linear', fill_value='extrapolate'),
        430: interp1d(x_430, X_430, kind='linear', fill_value='extrapolate'),
        # Add more rows here, e.g.:
        # 334: interp1d(x_334, X_334, kind='linear', fill_value='extrapolate'),
        # etc.
    }
    x_inverse_interp = {
        326: interp1d(X_326, x_326, kind='linear', fill_value='extrapolate'),
        430: interp1d(X_430, x_430, kind='linear', fill_value='extrapolate'),
        # Add more rows here
    }
    y_rows = np.array(list(x_interp.keys()))
    pixel_y = np.array([299, 302, 306, 310, 314, 319, 326, 334, 343, 354, 370, 394, 430, 480])
    ground_z_cm = np.array([160, 150, 140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30])
    z_interp = interp1d(pixel_y, ground_z_cm, kind='cubic', fill_value='extrapolate')
    y_inverse_interp = interp1d(ground_z_cm, pixel_y, kind='cubic', fill_value='extrapolate')

    @classmethod
    def pixel_row_to_ground_distance(cls, y):
        return float(cls.z_interp(y))

    @classmethod
    def pixel_to_lateral_ground(cls, x, y):
        idx = np.searchsorted(cls.y_rows, y)
        if idx == 0:
            return float(cls.x_interp[cls.y_rows[0]](x))
        elif idx == len(cls.y_rows):
            return float(cls.x_interp[cls.y_rows[-1]](x))
        y0, y1 = cls.y_rows[idx-1], cls.y_rows[idx]
        X0 = cls.x_interp[y0](x)
        X1 = cls.x_interp[y1](x)
        X = X0 + (X1 - X0) * (y - y0) / (y1 - y0)
        return float(X)

    @classmethod
    def pixel_to_ground(cls, x, y):
        X = cls.pixel_to_lateral_ground(x, y)
        Z = cls.pixel_row_to_ground_distance(y)
        return X, Z

    @classmethod
    def ground_to_pixel(cls, X, Z):
        # Inverse: ground Z -> y (row), then X -> x at that row
        y = float(cls.y_inverse_interp(Z))
        # Interpolate laterally as with pixel_to_lateral_ground, but using inverse
        idx = np.searchsorted(cls.y_rows, y)
        if idx == 0:
            x = float(cls.x_inverse_interp[cls.y_rows[0]](X))
        elif idx == len(cls.y_rows):
            x = float(cls.x_inverse_interp[cls.y_rows[-1]](X))
        else:
            y0, y1 = cls.y_rows[idx-1], cls.y_rows[idx]
            x0 = cls.x_inverse_interp[y0](X)
            x1 = cls.x_inverse_interp[y1](X)
            x = float(x0 + (x1 - x0) * (y - y0) / (y1 - y0))
        return x, y

#EOF
