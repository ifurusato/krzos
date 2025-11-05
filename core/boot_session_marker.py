#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-10-31
# modified: 2025-10-31

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
    
    Uses /proc/uptime to detect system reboots, so markers become invalid
    after a reboot regardless of elapsed time.
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
    
    def _get_system_boot_time(self):
        '''
        Get system boot time as a float timestamp.
        This changes only when the system reboots.
        '''
        try:
            with open('/proc/uptime', 'r') as f:
                uptime_seconds = float(f.readline().split()[0])
            boot_time = time.time() - uptime_seconds
            return boot_time
        except (IOError, ValueError) as e:
            self._log.warning('could not read system uptime: {}'.format(e))
            return None
    
    def is_marked(self):
        '''
        Check if the operation has been marked as completed during this boot session.
        
        Returns:
            True if operation was performed this boot, False otherwise
        '''
        self._log.info('marker path: {}'.format(self._marker_path))
        if not self._marker_path.exists():
            self._log.info('no marker found for "{}"'.format(self._marker_name))
            return False
        
        try:
            # read boot time from marker file
            with open(self._marker_path, 'r') as f:
                marker_boot_time = float(f.read().strip())
            # get current boot time
            current_boot_time = self._get_system_boot_time()
            if current_boot_time is None:
                self._log.info('current boot time is None.')
                return False
            # check if marker is from this boot session
            # allow small tolerance for clock precision
            if abs(marker_boot_time - current_boot_time) < 1.0:
                self._log.info('marker valid for "{}" (this boot session)'.format(self._marker_name))
                return True
            else:
                self._log.info('marker invalid for "{}" (previous boot session)'.format(self._marker_name))
                return False
            
        except (ValueError, IOError) as e:
            self._log.warning('Error reading marker for "{}": {}'.format(self._marker_name, e))
            return False
    
    def mark(self):
        '''
        Mark the operation as completed for this boot session.
        '''
        try:
            boot_time = self._get_system_boot_time()
            if boot_time is not None:
                with open(self._marker_path, 'w') as f:
                    f.write(str(boot_time))
                self._log.info('marker created for "{}"'.format(self._marker_name))
            else:
                self._log.warning('could not determine boot time, marker not created for "{}"'.format(self._marker_name))
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

"""
def main():
    '''
    Test the BootSessionMarker functionality.
    '''
    print('Testing BootSessionMarker...\n')
    
    # Create a test marker
    marker = BootSessionMarker('test_operation', level=Level.INFO)
    
    # Show temp directory
    print('Temp directory: {}'.format(tempfile.gettempdir()))
    print('Marker file: {}\n'.format(marker._marker_path))
    
    # Test 1: Check if marked (should be False initially)
    print('Test 1: Initial check (should be False)')
    is_marked = marker.is_marked()
    print('  is_marked() = {}\n'.format(is_marked))
    
    # Test 2: Mark the operation
    print('Test 2: Mark the operation')
    marker.mark()
    print('  Marked\n')
    
    # Test 3: Check again (should be True now)
    print('Test 3: Check after marking (should be True)')
    is_marked = marker.is_marked()
    print('  is_marked() = {}\n'.format(is_marked))
    
    # Test 4: Show boot time info
    print('Test 4: Boot time information')
    boot_time = marker._get_system_boot_time()
    if boot_time:
        uptime_seconds = time.time() - boot_time
        print('  System boot time: {}'.format(time.ctime(boot_time)))
        print('  System uptime: {:.2f} seconds ({:.2f} hours)\n'.format(
            uptime_seconds, uptime_seconds / 3600.0))
    
    # Test 5: Multiple instances share state
    print('Test 5: Create new instance with same name')
    marker2 = BootSessionMarker('test_operation', level=Level.INFO)
    is_marked2 = marker2.is_marked()
    print('  is_marked() on new instance = {}\n'.format(is_marked2))
    
    # Test 6: Clear the marker
    print('Test 6: Clear the marker')
    marker.clear()
    is_marked = marker.is_marked()
    print('  is_marked() after clear = {}\n'.format(is_marked))
    
    print('All tests completed!')

if __name__ == '__main__':
    main()
"""

# EOF
