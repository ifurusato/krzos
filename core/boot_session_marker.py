#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-31
# modified: 2025-11-08

import os
import sys
import time
import tempfile
from pathlib import Path
from core.logger import Logger, Level

class BootSessionMarker:
    '''
    Utility class for tracking whether an operation has been performed during
    the current boot session. Useful for operations that only need to happen
    once per power cycle (e.g., firmware loading).
    
    Uses /proc/sys/kernel/random/boot_id to detect system reboots. This UUID
    changes on every boot, so markers automatically become invalid after reboot.
    '''
    def __init__(self, marker_name='kros_session', level=Level.INFO):
        '''
        Parameters:
            marker_name: Unique identifier for this marker (e.g., 'vl53l5cx_firmware')
        '''
        self._log = Logger('boot-marker', level)
        self._marker_name = marker_name
        marker_dir = Path(tempfile.gettempdir())
        self._marker_path = marker_dir / '.{}_boot_marker'.format(marker_name)
        self._log.info('marker path: {}'.format(self._marker_path))
    
    def _get_boot_id(self):
        '''
        Get unique boot session ID from kernel.
        This UUID changes on every system reboot.
        '''
        try:
            with open('/proc/sys/kernel/random/boot_id', 'r') as f:
                return f.read().strip()
        except IOError as e:
            self._log.warning('could not read boot_id: {}'.format(e))
            return None
    
    def is_marked(self):
        '''
        Check if the operation has been marked as completed during this boot session.
        
        Returns:
            True if operation was performed this boot, False otherwise
        '''
        if not self._marker_path.exists():
            self._log.info('no marker found for "{}"'.format(self._marker_name))
            return False
        try:
            # read boot_id from marker file
            with open(self._marker_path, 'r') as f:
                marker_boot_id = f.read().strip()
            # get current boot_id
            current_boot_id = self._get_boot_id()
            if current_boot_id is None:
                self._log.info('current boot_id is None.')
                return False
            # check if marker is from this boot session (exact string match)
            if marker_boot_id == current_boot_id:
                self._log.info('marker valid for "{}" (this boot session)'.format(self._marker_name))
                return True
            else:
                self._log.info('marker invalid for "{}" (different boot session)'.format(self._marker_name))
                return False
        except IOError as e:
            self._log.warning('error reading marker for "{}": {}'.format(self._marker_name, e))
            return False
    
    def mark(self):
        '''
        Mark the operation as completed for this boot session.
        '''
        try:
            boot_id = self._get_boot_id()
            if boot_id is not None:
                with open(self._marker_path, 'w') as f:
                    f.write(boot_id)
                self._log.info('marker created for "{}"'.format(self._marker_name))
            else:
                self._log.warning('could not determine boot_id, marker not created for "{}"'.format(self._marker_name))
        except IOError as e:
            self._log.warning('could not create marker for "{}": {}'.format(self._marker_name, e))
    
    def clear(self):
        '''
        Remove the marker file (forces operation to run on next check).
        Useful for testing or manual resets.
        '''
        try:
            if self._marker_path.exists():
                self._marker_path.unlink()
                self._log.info('marker cleared for "{}"'.format(self._marker_name))
            else:
                self._log.info('no marker to clear for "{}"'.format(self._marker_name))
        except IOError as e:
            self._log.warning('could not clear marker for "{}": {}'.format(self._marker_name, e))

# EOF
