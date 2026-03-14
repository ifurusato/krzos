#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-03-12
# modified: 2026-03-12

import os
import yaml
from navigate.floorplan import Waypoint, Point

class RouteLoader:
    '''
    Loads a saved route YAML file and returns a list of Waypoint objects.
    '''

    @staticmethod
    def load(path):
        '''
        Load and return a list of Waypoints from the given YAML file path.
        '''
        if not os.path.isabs(path):
            _path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', path)
            _path = os.path.normpath(_path)
        else:
            _path = path
        with open(_path, 'r') as f:
            _data = yaml.safe_load(f)
        _waypoints = []
        for _entry in _data.get('route', []):
            _pos = _entry['position']
            _wp = Waypoint(
                label          = _entry['label'],
                position       = Point(_pos['x'], _pos['y']),
                kind           = _entry['kind'],
                arrival_radius = _entry.get('arrival_radius', 200.0),
                heading        = _entry.get('heading', None),
            )
            _waypoints.append(_wp)
        return _waypoints

#EOF
