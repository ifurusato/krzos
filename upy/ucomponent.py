#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-06-29
# modified: 2025-06-27 (ucomponent)

from logger import Logger
from colorama import Fore, Style

class Component(object):
    '''
    A basic component providing support for enable or disable, suppress or
    release, and close flags.

    For documentation, check the CPython version.
    '''
    def __init__(self, logger, suppressed=True, enabled=False):
        if not isinstance(logger, Logger):
            raise ValueError('wrong type for logger argument: {}'.format(type(logger)))
        self._log        = logger
        if not isinstance(suppressed, bool):
            raise ValueError('wrong type for suppressed argument: {}'.format(type(suppressed)))
        self._suppressed = suppressed
        if not isinstance(enabled, bool):
            raise ValueError('wrong type for enabled argument: {}'.format(type(enabled)))
        self._enabled    = enabled
        self._closed     = False

    @property
    def classname(self):
        return type(self).__name__

    @property
    def enabled(self):
        return self._enabled

    @property
    def disabled(self):
        return not self._enabled

    @property
    def suppressed(self):
        return self._suppressed

    @property
    def is_active(self):
        return self.enabled and not self.suppressed

    @property
    def closed(self):
        return self._closed

    def enable(self):
        if not self.closed:
            self._enabled = True
            self._log.debug('enabled.')
        else:
            self._log.warning('cannot enable: already closed.')

    def suppress(self):
        self._suppressed = True
        self._log.debug('suppressed.')

    def release(self):
        self._suppressed = False
        self._log.debug('released.')

    def disable(self):
        if self.enabled:
            self._enabled = False
            self._log.debug('disabled.')
        else:
            self._log.debug('already disabled.')
        return True

    def close(self):
        if not self.closed:
            _nil = self.disable()
            self._closed = True
            self._log.debug('closed.')
        else:
            self._log.debug('already closed.')
        return True

#EOF
