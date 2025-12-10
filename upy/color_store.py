#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-12-04
# modified: 2025-12-04

import json
import os

class ColorStore:
    '''
    A store for color tuples by name.
    '''
    def __init__(self, filename="colors.json"):
        self.filename = filename
        self.colors = {}  # name -> (r,g,b)
        self._load()

    def clear(self):
        self.colors = {}

    def put(self, name, rgb):
        '''
        Save a color in memory and persist to file.
        '''
        self.colors[name] = tuple(rgb)  # store as tuple
        self._save_file()

    def get(self, name):
        '''
        Return a stored color, or None if not found.
        '''
        return self.colors.get(name)

    def delete(self, name):
        '''
        Remove a color by name and persist the change.
        '''
        if name in self.colors:
            del self.colors[name]
            self._save_file()
            return True
        return False  # color didn't exist

    def _save_file(self):
        try:
            # convert tuples to lists for JSON
            serializable = {k: list(v) for k, v in self.colors.items()}
            with open(self.filename, "w") as f:
                json.dump(serializable, f)
        except Exception as e:
            print("Error saving colors:", e)

    def _load(self):
        '''
        Load colors from file if it exists.
        '''
        if self.filename in os.listdir():
            try:
                with open(self.filename, "r") as f:
                    data = json.load(f)
                    # convert lists back to tuples
                    self.colors = {k: tuple(v) for k, v in data.items()}
            except Exception as e:
                print("Error loading colors:", e)

#EOF
