#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-08-30
# modified: 2025-08-30

import os, sys, subprocess
import ast
import time
import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from hardware.rgb_led import RGBLED
from hardware.color import Color

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Diagnostics:
    '''
    A diagnostics class that scans for test files and then executes any methods
    decorated with "@pytest.mark.unit"
    '''
    def __init__(self, level=Level.INFO):
        self._log = Logger('diagnostics', level=Level.INFO)
        self._led_control = RGBLED(0.667)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _find_test_files(self):
        '''
        Scans the current working directory for .py files containing pytest functions with the '@pytest.mark.unit' decorator.

        Returns:
            A list of filenames that contain at least one unit-test function.
        '''
        test_files = []
        for filename in os.listdir("."):
            self._led_control.set_color(Color.BLUE)
            if filename.endswith(".py"):
                try:
                    if self._get_marked_tests(filename, marker="unit"):
                        test_files.append(filename)
                except (SyntaxError, IOError):
                    continue
            time.sleep(10 / 1000)
            self._led_control.set_color(Color.BLACK)
        return test_files

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_marked_tests(self, filename, marker="unit"):
        '''
        Parses a Python file to find all test functions with a specific marker.

        Args:
            filename: The path to the Python file.
            marker: The name of the pytest marker to look for.

        Returns:
            A list of function names that are decorated with the specified marker.
        '''
        marked_tests = []
        with open(filename, "r") as _file:
            self._led_control.set_color(Color.DARK_BLUE)
            tree = ast.parse(_file.read())
            for node in ast.walk(tree):
                if isinstance(node, ast.FunctionDef) and node.name.startswith("test_"):
                    for decorator in node.decorator_list:
                        if (
                            isinstance(decorator, ast.Attribute)
                            and isinstance(decorator.value, ast.Attribute)
                            and decorator.value.attr == "mark"
                            and decorator.attr == marker
                        ):
                            marked_tests.append(node.name)
            time.sleep(10 / 1000)
            self._led_control.set_color(Color.BLACK)
        return marked_tests

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _execute_tests(self, files_to_test):
        '''
        Executes pytest on the provided list of files and specific functions.

        Returns True if errors were found.

        Args:
            files_to_test: A list of filenames to run pytest on.
        '''
        if not files_to_test:
            self._log.warning("no unit test files found to run.")
            return False
        self._log.info("running unit tests on the following functions:")
        for _file in files_to_test:
            unit_tests = self._get_marked_tests(_file)
            for test_name in unit_tests:
                self._log.info("    {}:{}".format(_file, test_name))
        try:
            command = [sys.executable, "-m", "pytest", "-s", "-q"]
#           command = [sys.executable, "-m", "pytest", "-s", "-q", "--log-cli-level=INFO"]
#           command = [sys.executable, "-m", "pytest", "-s", "-rN"]
            for _file in files_to_test:
                unit_tests = self._get_marked_tests(_file)
                for test_name in unit_tests:
                    command.append("{}.py::{}".format(os.path.splitext(_file)[0], test_name))
            if len(command) == 3:
                self._log.warning("no unit tests found to run.")
                return False
            # run pytest, suppress output on success only
#           result = subprocess.run(command, capture_output=True, text=True, check=False)
            self._led_control.set_color(Color.DARK_MAGENTA)
            result = subprocess.run(
                command,
#               stdout=subprocess.PIPE,
#               stderr=subprocess.PIPE,
                text=True,
                check=False
            )
            self._led_control.set_color(Color.BLACK)
            # log non-pytest messages
            self._log.info("Finished running pytest on {} files.".format(len(files_to_test)))
            if result.returncode == 0:
                self._log.info(Fore.GREEN + "all unit tests passed successfully.")
                self._led_control.set_color(Color.GREEN)
                time.sleep(3)
                return False
            else:
                # if tests failed, print the full output for debugging
                raise subprocess.CalledProcessError(result.returncode, command)
                self._log.info(Fore.YELLOW + result.stdout)
                self._log.info(Fore.YELLOW + result.stderr)
                raise subprocess.CalledProcessError(result.returncode, command)
        except subprocess.CalledProcessError as e:
            self._log.info("stdout from running tests: {}".format(e.stdout))
            self._log.error("error from running tests: {}".format(e.stderr))
            self._led_control.set_color(Color.RED)
            time.sleep(3)
            return True
        except FileNotFoundError:
            self._log.error("pytest is not installed or not found. Please install it using 'pip3 install pytest'.")
            self._led_control.set_color(Color.RED)
            time.sleep(3)
            return True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def run(self, run_tests=False):
        '''
        Orchestrate the scanning and optional execution of tests.
        '''
        _start_time = dt.datetime.now()
        try:
            there_were_errors = False
            test_files = self._find_test_files()
            if not test_files:
                self._log.warning("no unit test files found.")
            elif test_files:
                self._log.info("found the following unit test files:")
                for _file in test_files:
                    self._log.info("    {}".format(_file))
                if run_tests:
                    there_were_errors = self._execute_tests(test_files)
                else:
                    self._log.info(Fore.YELLOW + "diagnostics called without '--run-tests' argument.")
            else:
                self._log.warning("no unit test files were found in the current directory.")
        except subprocess.CalledProcessError:
            sys.exit(1)
        finally:
            if not there_were_errors:
                self._led_control.cleanup()
            _elapsed_ms = int((dt.datetime.now() - _start_time).total_seconds() * 1000)
            self._log.info('diagnostics complete: {}ms elapsed.'.format(_elapsed_ms))

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():
    _diagnostics = Diagnostics()
    _diagnostics.run("--run-tests" in sys.argv)

if __name__ == "__main__":
    main()

#EOF
