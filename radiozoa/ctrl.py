#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License.
#
# author:   Ichiro Furusato
# created:  2025-11-16
# modified: 2026-02-05

import sys, gc
from controller import Controller

# force module reload
for mod in ['main', 'i2c_slave', 'controller']:
    if mod in sys.modules:
        del sys.modules[mod]
gc.collect()

def run_cli():

    ctrl = Controller()
    prompt = "▶ "

    print("Controller CLI")
    print("Type 'quit' or 'exit' to return to REPL.")
    while True:
        try:
            cmd = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            print("\nexiting CLI")
            break
        if not cmd:
            continue
        elif cmd in ("quit", "exit"):
            print("bye.")
            break
        try:
            print("cmd: '{}'".format(cmd))
            ctrl.process(cmd)
        except Exception as e:
            print("{} raised by control: {}".format(type(e), e))
            sys.print_exception(e)

run_cli()

#EOF
