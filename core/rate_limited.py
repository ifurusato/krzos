#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-11-17
# modified: 2025-11-17

import time
import threading
from functools import wraps

def rate_limited(min_interval_ms):
    '''
    Decorator to allow function execution only if a minimum interval has passed.

    Usage:

        @rate_limited(500) # 500 ms between calls
        def method():
            print("method executed")
    '''
    min_interval_sec = min_interval_ms / 1000.0
    lock = threading.Lock()  # for thread-safety

    def decorator(func):
        last_called = [0.0]  # mutable to allow updates

        def wrapper(*args, **kwargs):
            with lock:
                now = time.time()
                elapsed = now - last_called[0]
                if elapsed < min_interval_sec:
                    # Too soon; skip call
                    return
                last_called[0] = now
            return func(*args, **kwargs)
        return wrapper
    return decorator

def rate_limited_method(func):
    '''
    A rate-limited decorator for instance methods that prevents the decorated
    method from executing more than once within a user-configurable delay
    interval stored on the instance.

    Usage:

    This requires a class variable self._rate_delay_ms:

        class MyClass:
            def __init__(self, delay):
                self._rate_delay_ms = delay
                self._last_called = 0

            @rate_limited_method
            def my_method(self):
                print("Executed!")

        obj = MyClass(500)
        obj.my_method()   # executed!
        obj.my_method()   # kkipped if called within 500 ms
    '''
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        now = time.time()
        delay_ms = getattr(self, '_rate_delay_ms', 0)
        last_called = getattr(self, '_last_called', 0)
        elapsed = (now - last_called) * 1000
        if elapsed < delay_ms:
            return
        setattr(self, '_last_called', now)
        return func(self, *args, **kwargs)
    return wrapper

#EOF
