#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-08-01
# modified: 2026-08-02
#
# KR0S Robot Operating System Daemon (krosd). This also uses the krosd.service.
#
# see: https://dpbl.wordpress.com/2017/02/12/a-tutorial-on-python-daemon/
#
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

import os
import sys
import traceback
from pathlib import Path
import time
import subprocess
import signal

try:
    import daemon
    from daemon.pidfile import PIDLockFile
except Exception:
    sys.exit("This script requires the python-daemon module.\nInstall with: pip3 install --user python-daemon")

from core.util import Util
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.config_loader import ConfigLoader
from hardware.button import Button

# constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

USE_DAEMON = False  # set False to run in foreground
MODULE_NAME = 'kros'
WORK_DIR = '/home/pi/workspaces/workspace-krzos/krzos/'
APPLICATION_FILENAME = '{}.py'.format(MODULE_NAME)
APPLICATION_PATH = os.path.join(WORK_DIR, APPLICATION_FILENAME)
print(Fore.BLUE + Style.BRIGHT + "application path: '{}'".format(APPLICATION_PATH) + Style.RESET_ALL)
PID_FILE = WORK_DIR + '.krosd.pid'

class KrosDaemon:
    '''
    Monitors a push button to enable and disable KROS.
    ''' 
    def __init__(self, level):
        self._level = level
        self._log = Logger("krosd", self._level)
        self._log.info('initialising krosd…')
        
        self._kros_enabled = False
        self._process = None
        self._config = ConfigLoader().configure()
        self._button = None
        
        self._setup_button()
        
        _rosd_mask = os.umask(0)
        os.umask(_rosd_mask)
        self._log.info('mask: {}{}'.format(Fore.GREEN, _rosd_mask))
        self._log.info('uid:  {}{}'.format(Fore.GREEN, os.getuid()))
        self._log.info('gid:  {}{}'.format(Fore.GREEN, os.getgid()))
        self._log.info('cwd:  {}{}'.format(Fore.GREEN, os.getcwd()))
        self._log.info('pid:  {}{}'.format(Fore.GREEN, PID_FILE))
        self._send_tinyfx_command("ch3 on")
        self._log.info('krosd ready.')
    
    def _setup_button(self):
        if self._button is None:
            self._log.info('initialising daemon button hardware…')
            self._button = Button(config=self._config, callback=self._on_button_pressed, level=self._level)
    
    def _get_timestamp(self):
        return dt.utcfromtimestamp(dt.utcnow().timestamp()).isoformat()
    
    def _on_button_pressed(self):
        if self._process is None or self._process.poll() is not None:
            self._log.info('button pushed; starting KROS process…')
            self._enable_kros()
        else:
            self._log.info('button pushed; stopping KROS process…')
            self._process.send_signal(signal.SIGINT)
    
    def enable(self):
        self._enable_kros()

    def _enable_kros(self):
        if Util.already_running(APPLICATION_FILENAME):
            self._log.warning('{} already running…'.format(MODULE_NAME))
        else:
#           self._send_tinyfx_command("ch3 off") # let's let kros do this instead
            time.sleep(0.3)
            self._log.info('starting {} at {}…'.format(APPLICATION_FILENAME, self._get_timestamp()))
            _cmd = [sys.executable, APPLICATION_PATH, '-s', '-D']
            self._process = subprocess.Popen(_cmd, cwd=WORK_DIR, close_fds=True)
            self._kros_enabled = True
            self._log.info('{} enabled; pid: {}'.format(MODULE_NAME, self._process.pid))
            
    def _send_tinyfx_command(self, command):
        self._log.info('tinyfx command: ' + Fore.GREEN + "'{}'".format(command))
        from hardware.tinyfx_controller import TinyFxController

        tfxc = None
        try:
            tfxc = TinyFxController()
            tfxc.enable()
            tfxc.send_request(command)
        except Exception as e:
            print(string.format('failed to send TinyFX command: {}', e))
        finally:
            if tfxc:
                tfxc.close()

    def check_process_status(self):
        '''
        Monitors the child process status.
        '''
        if self._process is not None:
            return_code = self._process.poll()
            if return_code is not None:
                self._log.info('{} process exited with code {}.'.format(MODULE_NAME, return_code))
                self._process = None
                self._kros_enabled = False
                
    def close(self):
        self._log.info(Fore.WHITE + 'closing krosd…')
        if self._button is not None:
            self._log.info('releasing daemon button hardware…')
            self._button.close()
            self._button = None
        if self._process is not None and self._process.poll() is None:
            self._log.info('shutting down child process…')
            self._process.send_signal(signal.SIGINT)
            try:
                self._process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
            self._process = None
        self._log.info(Fore.WHITE + 'krosd closed.')
        
def cleanup_stale_processes():
    '''
    Checks for and terminates stale running python processes for this application.
    '''
    current_pid = os.getpid()
    try:
        output = subprocess.check_output(['pgrep', '-f', 'krosd.py'], text=True)
        pids = [int(p) for p in output.strip().split('\n') if p.isdigit()]
        for pid in pids:
            if pid != current_pid:
                print('terminating stale process: {}'.format(pid))
                try:
                    os.kill(pid, signal.SIGTERM)
                    time.sleep(0.2)
                    os.kill(pid, signal.SIGKILL)
                except OSError:
                    pass
    except subprocess.CalledProcessError:
        pass

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_daemon = None

def main():
    global _daemon
    try:
        cleanup_stale_processes()
        
        _daemon = KrosDaemon(Level.INFO)
        while True:
            _daemon.check_process_status()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print(Fore.YELLOW + '\nKeyboard interrupt received.' + Style.RESET_ALL)
    except Exception:
        print(Fore.WHITE + 'error starting kros daemon: {}'.format(traceback.format_exc()) + Style.RESET_ALL)
    finally:
        if _daemon:
            try:
                _daemon.close()
            except Exception:
                print(Fore.RED + 'error closing kros daemon: {}'.format(traceback.format_exc()) + Style.RESET_ALL)
            finally:
                _daemon = None
        print(Fore.WHITE + 'krosd complete.' + Style.RESET_ALL)


def shutdown(signum, frame):
    print('krosd.shutdown (signal {})'.format(signum))
    if _daemon:
        try:
            _daemon.close()
        except Exception:
            pass
    sys.exit(0)
    
if Path(PID_FILE).is_file() and not Util.already_running(APPLICATION_FILENAME):
    try:
        os.remove(PID_FILE)
        print(Fore.WHITE + 'deleted previous pid file.' + Style.RESET_ALL)
    except OSError:
        pass

if __name__ == '__main__':
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    
    if USE_DAEMON:
        with daemon.DaemonContext(
            stdout=sys.stdout,
            stderr=sys.stderr,
            working_directory=WORK_DIR,
            umask=0o002,
            pidfile=PIDLockFile(PID_FILE),
            signal_map={
                signal.SIGTERM: shutdown,
                signal.SIGTSTP: shutdown
            }   
        ) as context:
            main()
    else:   
        print(Fore.YELLOW + "Running in foreground mode (no daemon)" + Style.RESET_ALL)
        main()
        
#EOF
