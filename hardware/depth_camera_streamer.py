#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Portions copyright 2020-2025 by Murray Altheim. All rights reserved. This file
# is part of the Robot Operating System project, released under the MIT License.
# Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-09-28
# modified: 2025-09-28

import cv2
import time
from flask import Flask, Response
import logging # for flask log integration
from colorama import init, Fore, Style
init()

from core.util import Util
from core.logger import Logger, Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class FlaskToCustomLogger(logging.Handler):
    def __init__(self):
        super().__init__()
        self._log = Logger('flask', level=Level.INFO)

    def emit(self, record):
        msg = self.format(record)
#       print(Fore.BLACK + "message: '{}'".format(msg) + Style.RESET_ALL)
        if "This is a development server." in msg or "Press CTRL+C to quit" in msg:
            pass # suppress the annoying messages
        elif record.levelno >= logging.ERROR:
            self._log.error(msg)
        elif record.levelno >= logging.WARNING:
            self._log.warning(msg)
        elif record.levelno >= logging.INFO:
            self._log.info(msg)
        else:
            self._log.debug(msg)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class DepthCameraStreamer:

    def __init__(self, depth_camera):
        self._log = Logger('cam-streamer', level=Level.INFO)
        self.app = Flask(__name__)
        self._depth_camera = depth_camera
        self.streaming_enabled = False
        # Flask/Werkzeug log capture ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        flask_log = logging.getLogger('flask.app')
        werkzeug_log = logging.getLogger('werkzeug')
        handler = FlaskToCustomLogger()
        handler.setFormatter(logging.Formatter('%(levelname)s: %(message)s'))
        flask_log.addHandler(handler)
        werkzeug_log.addHandler(handler)
        flask_log.propagate = False
        werkzeug_log.propagate = False
        # end log capture ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._log.info('ready.')

        @self.app.route('/video_feed')
        def video_feed():
            if not self.streaming_enabled:
                self._log.info('Video stream requested but streaming is disabled.')
                return "Stream disabled", 503
            return Response(self.gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def gen_frames(self):
        self._log.info('starting video stream generator…')
        while self._depth_camera.enabled:
            if not self.streaming_enabled:
                time.sleep(0.1)
                continue
            frame = self._depth_camera.get_latest_color_frame()
            if frame is None:
                self._log.warning('No color frame available, skipping frame.')
                continue
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                self._log.error('Failed to encode frame.')
                continue
            frame_bytes = buffer.tobytes()
            try:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            except GeneratorExit:
                self._log.info('Client disconnected from video feed.')
                break

    def enable_stream(self):
        self.streaming_enabled = True
        self._log.info("Video streaming enabled.")

    def disable_stream(self):
        self.streaming_enabled = False
        self._log.info("Video streaming disabled.")

    def run(self, host='0.0.0.0', port=5000):
        ip_address = Util.get_ip_address()
        url = "http://{}:{}/video_feed".format(ip_address, port)
        self._log.info(Fore.GREEN + "video stream available at: {}".format(url))
        if Util.is_port_in_use(port, host):
            self._log.warning("port {} is already in use, not starting another server.".format(port))
            return
        self.app.run(host=host, port=port, threaded=False, debug=False, use_reloader=False)
#       self.app.run(host=host, port=port, threaded=True)

#EOF
