#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2021-10-11
# modified: 2025-11-27

import itertools
import asyncio
from colorama import init, Fore, Style
init()

from core.dequeue import DeQueue
from core.logger import Logger, Level
from core.publisher import Publisher

class QueuePublisher(Publisher):
    NAME = 'pub:queue'
    _PUBLISHER_LOOP = '__queue_publisher_loop'
    '''
    A Publisher that publishes messages from a queue, available as a global
    publishing service. This is to permit lower-priority, non-asynchronous
    processes to publish messages to the message bus, with no particular
    guarantee of immediate delivery.

    :param config:          the application configuration
    :param message_bus:     the asynchronous message bus
    :param message_factory: the factory for messages
    :param level:           the optional log level
    '''
    def __init__(self, config, message_bus, message_factory, level=Level.INFO):
        Publisher.__init__(self, QueuePublisher.NAME, config, message_bus, message_factory, suppressed=False, enabled=False, level=level)
        _cfg = self._config['kros'].get('publisher').get('queue')
        _loop_freq_hz  = _cfg.get('loop_freq_hz')
        self._log.info('queue publisher loop frequency: {:d}Hz'.format(_loop_freq_hz))
        self._publish_delay_sec = 1.0 / _loop_freq_hz
        self._queue    = DeQueue()
        self._counter  = itertools.count()
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def name(self):
        return QueuePublisher.NAME

    def put(self, message):
        if self.enabled:
            if not self.is_active:
                self._log.warning('message {} ignored: queue publisher inactive.'.format(message.name))
            else:
                NO_WAIT = True # TENTATIVE for synchronous calls
                if NO_WAIT:
                    self._queue.put_nowait(message) 
                else:
                    self._queue.put(message)
                self._log.info('put message \'{}\' ({}) into queue ({:d} {})'.format(
                        message.event.name, message.name, self._queue.size, 'item' if self._queue.size == 1 else 'items'))
        else:
            self._log.warning('queue publisher disabled.')

    def enable(self):
        if not self.enabled:
            self._log.info('enable queue publisher.')
#           super().enable()
            Publisher.enable(self)
            if self._message_bus.get_task_by_name(QueuePublisher._PUBLISHER_LOOP):
                raise Exception('already enabled.')
            elif self._message_bus.loop is None:
                self._log.warning('no message bus loop available.')
#               raise Exception('no message bus loop available.')
            else:
                self._log.info('creating task for publisher loop…')
                self._message_bus.loop.create_task(self._publisher_loop(lambda: self.enabled), name=QueuePublisher._PUBLISHER_LOOP)
                self._log.info('enabled.')
        else:
            self._log.warning('failed to enable publisher loop.')

    async def _publisher_loop(self, f_is_enabled):
        self._log.info('starting queue publisher loop: ' + Fore.YELLOW + ( '(suppressed, type \'m\' to release)' if self.suppressed else '(released)') )
        try:
            while f_is_enabled():
                _count = next(self._counter)
#               self._log.debug('[{:03d}] begin publisher loop…'.format(_count))
                if not self.suppressed:
                    while not self._queue.empty():
                        await asyncio.sleep(0) # TENTATIVE
                        _message = self._queue.poll()
                        await Publisher.publish(self, _message)
                        self._log.info('[{:03d}] published message '.format(_count)
                                + Fore.WHITE + '{} '.format(_message.name)
                                + Fore.CYAN + 'for event \'{}\' with value: '.format(_message.event.label)
                                + Fore.YELLOW + '{}'.format(_message.payload.value))
                await asyncio.sleep(self._publish_delay_sec)
        except asyncio.CancelledError:
            self._log.info("publisher loop cancelled.")
            raise
        finally:
            self._log.info('publisher loop complete.')

#EOF
