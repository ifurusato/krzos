#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2019-2021 by Murray Altheim. All rights reserved. This file is part
# of the K-Series Robot Operating System (KROS) project, released under the MIT
# License. Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-01-14
# modified: 2025-11-13

import os, logging, math, traceback, threading
from pathlib import Path
from logging import FileHandler
from datetime import datetime as dt
from enum import Enum
from colorama import init, Fore, Style
init()

from core.log_stats import LogStats
from core.util import Util
from core.ansi_filtering_file_handler import AnsiFilteringFileHandler

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Level(Enum):
    DEBUG    = ( logging.DEBUG,    'DEBUG'    ) # 10
    INFO     = ( logging.INFO,     'INFO'     ) # 20
    DATA     = ( logging.INFO + 5, 'DATA'     ) # 25
    WARN     = ( logging.WARN,     'WARN'     ) # 30
    ERROR    = ( logging.ERROR,    'ERROR'    ) # 40
    CRITICAL = ( logging.CRITICAL, 'CRITICAL' ) # 50

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __new__(cls, *args, **kwds):
        obj = object.__new__(cls)
        obj._value_ = args[0]
        return obj

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    # ignore the first param since it's already set by __new__
    def __init__(self, num, label):
        self._label = label
        logging.addLevelName(num, label)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def from_string(label):
        if label.upper()   == 'DEBUG':
            return Level.DEBUG
        elif label.upper() == 'INFO':
            return Level.INFO
        elif label.upper() == 'DATA':
            return Level.DATA
        elif label.upper() == 'WARN':
            return Level.WARN
        elif label.upper() == 'ERROR':
            return Level.ERROR
        elif label.upper() == 'CRITICAL':
            return Level.CRITICAL
        else:
            raise NotImplementedError

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DataOnlyFilter(logging.Filter):
    '''
    A logging filter that allows only records at the DATA level.
    '''
    def filter(self, record):
        return record.levelno == Level.DATA.value

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class ExcludeDataFilter(logging.Filter):
    '''
    A logging filter that blocks any records at the DATA level.
    '''
    def filter(self, record):
        return record.levelno != Level.DATA.value

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Logger(object):
    _log_stats = LogStats() # singleton
    _fh        = None # optional file handler

    # static members for shared data logger
    _data_fh = None
    _data_logger_owner_id = None
    _data_logger_lock = threading.Lock()

    __suppress       = False
    __color_debug    = Fore.BLUE   + Style.DIM
    __color_info     = Fore.CYAN   + Style.NORMAL
    __color_notice   = Fore.WHITE  + Style.BRIGHT
    __color_warning  = Fore.YELLOW + Style.NORMAL
    __color_error    = Fore.RED    + Style.NORMAL
    __color_critical = Fore.WHITE  + Style.NORMAL
    __color_reset    = Style.RESET_ALL

    INCLUDE_METADATA  = True
    USE_ISO_TIMESTAMP = False

    def __init__(self, name, log_to_console=True, log_to_file=False, data_logger=False, level=Level.INFO):
        '''
        Writes to a named log with the provided level, defaulting to a
        console (stream) handler unless 'log_to_file' is True, in which
        case only write to file, not to the console.

        :param name:                  the name identified with the log output
        :param log_to_console:        if True will log to console
        :param log_to_file:           if True will log to file (only for this instance)
        :param data_logger:           if True, this logger will participate in shared data logging
        :param level:                 the log level
        '''
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _strip_ansi_codes       = True # used only with file output, to strip ANSI characters from log data
        self._include_timestamp = True
        self._date_format       = '%Y-%m-%dT%H:%M:%S'
#       self._date_format       = '%Y-%m-%dT%H:%M:%S.%f'
#       self._date_format       = '%H:%M:%S'
        # i18n?
        self.__DEBUG_TOKEN = 'DEBUG'
        self.__INFO_TOKEN  = 'INFO '
        self.__WARN_TOKEN  = 'WARN '
        self.__ERROR_TOKEN = 'ERROR'
        self.__FATAL_TOKEN = 'FATAL'
        self._mf           = '{}{} : {}{}'

        # create logger ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self.__mutex = threading.Lock()
        self.__log   = logging.getLogger(name)
        self.__log.propagate = False
        self._name   = name
        self._sh     = None # optional stream handler
        self._fh     = None # optional instance-specific file handler

        if not self.__log.handlers:
            if log_to_console: # log to console ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                self._sh = logging.StreamHandler()
                # filter out data messages from the console
                self._sh.addFilter(ExcludeDataFilter())
                fmt = ''
                if self._include_timestamp:
                    fmt += Logger.timestamp_format()  # timestamp portion
                # add the rest of the formatter, padded name
                padded_name = '{:<16}'.format(self._name)
                fmt += '{reset} {name} : %(message)s'.format(reset=Fore.RESET, name=padded_name)
                self._sh.setFormatter(logging.Formatter(fmt, datefmt=self._date_format))
                self.__log.addHandler(self._sh)

            if log_to_file and not data_logger: # instance-specific file logging
                # if ./log/ directory doesn't exist, create it
                if not os.path.exists('./log'):
                    try:
                        os.makedirs('./log')
                    except OSError as e:
                        raise Exception('could not create ./log directory: {}'.format(e))
                _ts = dt.utcfromtimestamp(dt.utcnow().timestamp()).isoformat().replace(':','_').replace('-','_').replace('.','_')
                _filename = './log/kros-{}.log'.format(_ts)
                self.info("logging to file: {}".format(_filename))
                if _strip_ansi_codes: # using ANSI filtering file handler
                    self._fh = AnsiFilteringFileHandler(filename=_filename, mode='w', encoding='utf-8')
                else: # using file handler
                    self._fh = FileHandler(filename=_filename, mode='w', encoding='utf-8')
                # filter out data messages from the standard log file
                self._fh.addFilter(ExcludeDataFilter())
                # build file format to match console format, but without ANSI codes
                fmt = ''
                if self._include_timestamp:
                    # uncolored timestamp for file
                    fmt += '%(asctime)s.%(msecs)03dZ\t:'
                padded_name = '{:<16}'.format(self._name)
                fmt += ' {name} : %(message)s'.format(name=padded_name)
                self._fh.setFormatter(logging.Formatter(fmt, datefmt=self._date_format))
                self.__log.addHandler(self._fh)

        if data_logger: # shared data logging ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            with Logger._data_logger_lock:
                if Logger._data_fh is None:
                    self.info('🍆 A. creating shared data logger...')
                    _project_root = Path(__file__).resolve().parents[1]
                    _log_dir = _project_root / 'log'
                    # make the log directory if it doesn't exist
                    if not _log_dir.exists():
                        try:
                            _log_dir.mkdir(parents=True, exist_ok=True)
                        except OSError as e:
                            raise Exception("could not create ./log directory {}: {}".format(str(_log_dir), e))
                    _ts = dt.utcfromtimestamp(dt.utcnow().timestamp()).isoformat().replace(':','_').replace('-','_').replace('.','_')
                    _filename = _log_dir / 'kros-data-{}.csv'.format(_ts)
                    self.info("shared data log file: {}".format(_filename))
                    Logger._data_fh = FileHandler(filename=_filename, mode='w', encoding='utf-8')
                    # add filter for data messages only
                    Logger._data_fh.addFilter(DataOnlyFilter())
                    # set format based on metadata flag
                    if Logger.INCLUDE_METADATA:
                        if Logger.USE_ISO_TIMESTAMP:
                            self.info('🍆 B. using ISO timestamp formatter.')
                            Logger._data_fh.setFormatter(logging.Formatter('%(asctime)s.%(msecs)03dZ,%(name)s,%(message)s', datefmt=self._date_format))
                        else: # use epoch float timestamp
                            self.info('🍆 C. using epoch float timestamp formatter.')
                            Logger._data_fh.setFormatter(logging.Formatter('%(created)f,%(name)s,%(message)s'))
                    else:
                        self.info('🍆 D. using raw data formatter.')
                        Logger._data_fh.setFormatter(logging.Formatter('%(message)s'))
                    Logger._data_logger_owner_id = id(self)
                    self.info('🍆 E. this logger instance is now the owner of the shared data logger.')
            # add the shared handler to this logger instance
            self.__log.addHandler(Logger._data_fh)

        self.level = level

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def close_data_handler():
        '''
        Closes the shared data file handler, if it is open.
        '''
        if Logger._data_fh:
            _filename = os.path.basename(Logger._data_fh.baseFilename)
#           print('close_data_handler: {}'.format(Logger._data_fh.baseFilename))
#           print('close_data_handler: {}'.format(_filename))
            print(Logger.timestamp_now() + Fore.RESET + ' {:<16} : '.format('logger')
                    + Fore.CYAN + Style.NORMAL + 'INFO  : ' + Fore.MAGENTA + 'closing log file: {}'.format(_filename) + Style.RESET_ALL)
            try:
                Logger._data_fh.close()
            except Exception as e:
                print(Fore.RED + "{} raised closing data file handler: {}".format(type(e), e) + Style.RESET_ALL)
            finally:
                Logger._data_fh = None
                Logger._data_logger_owner_id = None
                print(Logger.timestamp_now() + Fore.RESET + ' {:<16} : '.format('logger')
                        + Fore.CYAN + Style.NORMAL + 'INFO  : ' + Fore.MAGENTA + 'data handler file closed.\n' + Style.RESET_ALL)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def getFileHandler(self):
        '''
        Returns the File Handler if it has been defined.
        '''
        return Logger._fh

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def addHandler(self, handler):
        '''
        Add an additional handler to the logger.
        '''
        self.__log.addHandler(handler)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def timestamp_format():
        '''
        Return the timestamp portion of the log format string.
        '''
        return '{blue}{dim}%(asctime)s.%(msecs)03dZ\t:'.format(blue=Fore.BLUE, dim=Style.DIM)

    @staticmethod
    def timestamp_now():
        '''
        Return the current UTC timestamp, using the same colors as timestamp_format.
        '''
        fmt_string = Logger.timestamp_format()
        color_prefix = fmt_string.split('%')[0]
        now = dt.utcnow()
        ts = now.strftime('%Y-%m-%d %H:%M:%S')
        ms = int(now.microsecond / 1000)
        return '{prefix}{ts}.{ms:03d}Z\t:'.format(prefix=color_prefix, ts=ts, ms=ms)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        '''
        Return the name of this Logger.
        '''
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Closes down logging. If this instance is the owner of the shared
        data logger, it will be closed. Then, informs the logging system
        to perform an orderly shutdown by flushing and closing all handlers.
        This should be called at application exit.
        '''
        if Logger._data_fh is not None:
            if id(self) == Logger._data_logger_owner_id:
                self.info('this instance is the owner, closing shared data logger.')
                try:
                    if Logger._data_fh:
                        Logger._data_fh.close()
                except Exception as e:
                    self.error('exception closing data file handler: {}'.format(e))
                finally:
                    Logger._data_fh = None
                    Logger._data_logger_owner_id = None
            else:
                print('WAS NOT ID: DATA LOG CLOSE self id: {}; owner id: {}'.format(id(self), Logger._data_logger_owner_id ))

        logging.shutdown()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def suppress(self):
        '''
        Suppresses all log messages except critical errors and log-to-file
        messages. This is global across all Loggers.
        '''
        type(self).__suppress = True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def release(self):
        '''
        Releases (un-suppresses) all log messages except critical errors
        and log-to-file messages. This is global across all Loggers.
        '''
        type(self).__suppress = False

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def level(self):
        '''
        Return the level of this logger.
        '''
        return self._level

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @level.setter
    def level(self, level):
        '''
        Set the level of this logger to the argument.
        '''
        self._level = level
        self.__log.setLevel(self._level.value)
        if self._fh:
            self._fh.setLevel(level.value)
        if self._sh:
            self._sh.setLevel(level.value)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def is_at_least(self, level):
        '''
        Returns True if the current log level is less than or equals the
        argument. E.g.,

            if self._log.is_at_least(Level.WARN):
                # returns True for WARN or ERROR or CRITICAL
        '''
        return self._level.value >= level.value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def stats(self):
        '''
        Return the global log statistics.
        '''
        return Logger._log_stats

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def suppressed(self):
        '''
        Return True if this logger has been suppressed.
        '''
        return type(self).__suppress

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def data(self, *args):
        '''
        Logs a message with the DATA level. The arguments are converted to
        a single comma-separated string. This call is routed by filters
        exclusively to the shared data log file.
        '''
        if not args:
            return
        Logger._log_stats.data_count()
        # convert all arguments to string and join as a CSV line
        csv_message = ",".join(map(str, args))
        self.__log.log(Level.DATA.value, csv_message)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def debug(self, message):
        '''
        Prints a debug message.
        '''
        if not self.suppressed:
            Logger._log_stats.debug_count()
            with self.__mutex:
                self.__log.debug(self._mf.format(Logger.__color_debug, self.__DEBUG_TOKEN, message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def info(self, message):
        '''
        Prints an informational message.
        '''
        if not self.suppressed:
            Logger._log_stats.info_count()
            with self.__mutex:
                self.__log.info(self._mf.format(Logger.__color_info, self.__INFO_TOKEN, message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def notice(self, message):
        '''
        Functionally identical to info() except it prints the message brighter.
        '''
        if not self.suppressed:
            Logger._log_stats.info_count()
            with self.__mutex:
                self.__log.info(self._mf.format(Logger.__color_notice, self.__INFO_TOKEN, message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def warning(self, message):
        '''
        Prints a warning message.
        '''
        if not self.suppressed:
            Logger._log_stats.warn_count()
            with self.__mutex:
                self.__log.warning(self._mf.format(Logger.__color_warning, self.__WARN_TOKEN, message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def error(self, message):
        '''
        Prints an error message.
        '''
        if not self.suppressed:
            Logger._log_stats.error_count()
            with self.__mutex:
                self.__log.error(self._mf.format(Logger.__color_error, self.__ERROR_TOKEN, Style.NORMAL + message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def critical(self, message):
        '''
        Prints a critical or otherwise application-fatal message.
        '''
        with self.__mutex:
            Logger._log_stats.critical_count()
            self.__log.critical(self._mf.format(Logger.__color_critical, self.__FATAL_TOKEN, Style.BRIGHT + message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def file(self, message):
        '''
        This is just info() but without any formatting.
        '''
        with self.__mutex:
            Logger._log_stats.info_count()
            self.__log.info(message)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def heading(self, title, message=None, info=None):
        '''
        Print a formatted, titled message to info(), inspired by maven console messaging.

        :param title:    the required title of the heading.
        :param message:  the optional message to display; if None only the title will be displayed.
        :param info:     an optional second message to display right-justified; ignored if None.
        '''
        if not self.suppressed:
            Logger._log_stats.info_count()
            _H = '┈'
            MAX_WIDTH = 100
            MARGIN = 27
            if title is None or len(title) == 0:
                raise ValueError('no title parameter provided (required)')
            _available_width = MAX_WIDTH - MARGIN
            self.info(self._get_title_bar(title, _available_width))
            if message:
                if info is None:
                    info = ''
                _min_msg_width = len(message) + 1 + len(info)
                if _min_msg_width >= _available_width:
                    # if total length is greater than available width, just print
                    self.info(Fore.WHITE + Style.BRIGHT + '{} {}'.format(message, info))
                else:
                    _message_2_right = info.rjust(_available_width - len(message) - 2)
                    self.info(Fore.WHITE + Style.BRIGHT + '{} {}'.format(message, _message_2_right))
                # print footer
                self.info(Fore.WHITE + Style.BRIGHT + Util.repeat(_H, _available_width-1))
            # print spacer
            self.info('')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_title_bar(self, message, MAX_WIDTH):
#       _H = '┈'
#       _H = '━'
        _H = '═'
#       _L = '┥ '
        _L = '╡ '
#       _R = ' ┝'
        _R = ' ╞'
        _carrier_width = len(message) + 4
        _hyphen_width = math.floor( ( MAX_WIDTH - _carrier_width ) / 2 )
        if _hyphen_width <= 0:
            return message
        elif len(message) % 2 == 0: # message is even length
            return Fore.WHITE + Style.BRIGHT + Util.repeat(_H, _hyphen_width) + _L + Fore.CYAN + Style.NORMAL\
                    + message + Fore.WHITE + Style.BRIGHT + _R + Util.repeat(_H, _hyphen_width)
        else:
            return Fore.WHITE + Style.BRIGHT + Util.repeat(_H, _hyphen_width) + _L + Fore.CYAN + Style.NORMAL\
                    + message + Fore.WHITE + Style.BRIGHT + _R + Util.repeat(_H, _hyphen_width-1)

#EOF
