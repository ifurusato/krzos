# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-07-11
# modified: 2025-07-15

import time
import asyncio
from machine import SPI
from pyb import Pin
from sysfont import sysfont
from st7735py import TFT, TFTColor

from colors import *
from ucomponent import Component
from logger import Logger, Level

class Display(Component):
    SCREEN_HEIGHT = 80
    SCREEN_WIDTH = 160
    MAX_LINE_LENGTH = 26 # console width
    X_OFFSET = 10
    ROTATION = 1 # 0-3, where 1 or 3 are landscape, rotates 90° cw each step
    LAYOUTS = { # font layout map: font size -> (row height, y_start, max lines)
        1.0: (13, 35, 5), # used for console
        1.2: (16, 38, 4),
        1.4: (17, 33, 4),
        1.5: (17, 34, 4),
        1.6: (18, 33, 4),
        1.8: (18, 38, 4),
        2.0: (20, 36, 3),
        2.5: (30, 40, 1),
        3.0: (32, 52, 1),
    }
    '''
    Supports an 80x160 TFT display.

    The pins are hard-coded for this specific hardware.
    '''
    def __init__(self, rotation=1):
        self._log = Logger('display', Level.INFO)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._spi = SPI(4, baudrate=31250000)
        self._backlight = Pin('E10', Pin.OUT)
        self._backlight(1) # initially off
        self._tft = TFT(self._spi, 'E13', None, 'E11') # SPI, DC, RST (None if tied to SYS_RESET), CS
        self._tft.rgb(False) # use RGB order
        self._tft.initr()
        self._last_text = ''
        self._tft.invertcolor(True)
        self._tft.rotation(rotation)
        self._console_buffer = []
        self._console_max_lines = 3 # 1-5 lines
        self._console_default = Display.to_tft_color((0, 255, 255)) # cyan
        self._amber = Display.to_tft_color((255, 220, 0))
        self._apple = Display.to_tft_color((216, 255, 0))
        self._ready = Display.to_tft_color((160, 188, 0))
        self.clear()
        self.enable()

    def enable(self):
        '''
        Enable the display by turning on screen updates and the backlight.
        '''
        if not self.enabled:
            Component.enable(self)
            self._tft.on(True)
            self._backlight(0)

    def disable(self):
        '''
        Disable the display by turning off screen updates and the backlight.
        '''
        if self.enabled:
            Component.disable(self)
            self._tft.on(False)
            self._backlight(1)

    def close(self):
        if not self.closed:
            self.clear()
            self.disable()
            try:
                self._spi.deinit()
            except AttributeError:
                pass
            finally:
                Component.close(self)

    def clear(self):
        '''
        Clear the entire display. This also clears the console buffer of any content.
        '''
        self._console_buffer.clear()
        self._tft.fill(TFT.BLACK)

    def show_line(self, text, line=1, size=1.5, x_offset=None, color=TFT.WHITE, animate=False, clear=False):
        '''
        Display text (or list of strings) starting at a given line.

        Args:
            text (str | list[str]):  text or list of lines to display.
            line (int):              1-based line number to start from.
            size (float):            font size (must be in LAYOUTS).
            color (int):             text color (RGB565).
            animate (bool):          if True, always print every request.
            clear (bool):            clear screen before rendering.
        '''
        if not animate and text != self._console_buffer and text == self._last_text:
            print("don't bother.")
            return # don't bother
        self._last_text = text
        if size not in self.LAYOUTS:
            raise ValueError(f"Font size {size} not configured in LAYOUTS")
        if x_offset is None:
            x_offset=Display.X_OFFSET
        row_height, y_start, max_lines = self.LAYOUTS[size]
        # normalize to list of lines
        lines = [text] if isinstance(text, str) else text[:]
        if not isinstance(lines, list) or any(not isinstance(l, str) for l in lines):
            raise TypeError("Text must be a string or list of strings")
        if (line - 1 + len(lines)) > max_lines:
            raise ValueError("{} lines starting from line {} exceeds maximum of {} for font size {}".format(
                    len(lines), line, max_lines, size))
        if clear:
            self.clear()
        for i, content in enumerate(lines):
            y = y_start + (line - 1 + i) * row_height
            pos = (x_offset, y)
            self._tft.text(pos, content, color, sysfont, size)

    def hello(self, persist=False):
        asyncio.create_task(self._hello_async(persist))

    async def _hello_async(self, persist=False):
        '''
        An animated "Hello." as a greeting.
        '''
        self._log.info('hello.')
        self.clear()
        apple_rgb = (216, 255, 0)
        black_rgb = (0, 0, 0)
        steps = 32
        fade_up = Display.fade_colors(black_rgb, apple_rgb, steps)
        fade_down = fade_up[-2::-1] # reverse excluding last color (the peak)
        fade_cycle = fade_up + fade_down
        for color_rgb in fade_cycle:
            tft_color = Display.to_tft_color(color_rgb)
            self.show_line(" Hello.", line=1, size=3.0, color=tft_color, animate=True)
            await asyncio.sleep(0.025)
        if persist:
            apple_color = Display.to_tft_color(apple_rgb)
            self.show_line(" Hello.", line=1, size=3.0, color=apple_color, animate=False)

    def ready(self):
        self._log.info('ready.')
        # def show_line(text, line=1, size=1.5, x_offset=None, color=TFT.WHITE, animate=False, clear=False):
        self.show_line(" Ready.", line=1, size=3.0, color=self._ready)

    def ip_address(self, ip_address):
        '''
        Display the IP address.
        '''
        self._log.info('ip address: {}'.format(ip_address))
        self.show_line(["", "IP address:", ip_address], line=1, size=1.5, color=self._amber, clear=True)

    async def console(self, text=None, color=None, clear=False):
        '''
        Display scrolling console-style text with a buffer size determined by self._console_max_lines.

        Args:
            text (str | None): New line of text to add. If None, a blank line is added.
            clear (bool): If True, clears the console buffer and screen.
        '''
        console_row_height = self.LAYOUTS[1.0][0] # 13 pixels
        x_offset = 2
        line_to_add = "" if text is None else text
        if len(line_to_add) > Display.MAX_LINE_LENGTH:
            line_to_add = line_to_add[:Display.MAX_LINE_LENGTH - 1] + '>'
        if clear:
            self.clear()
            if text is not None:
                self._console_buffer.append(line_to_add)
            if not self._console_buffer:
                return
        elif text is not None:
            self._console_buffer.append(line_to_add)
        if color is None:
            color = self._console_default
        if len(self._console_buffer) > self._console_max_lines:
            self._console_buffer.pop(0)
        # calculate the top Y position of the console display area.
        console_window_top_y = Display.SCREEN_HEIGHT - (self._console_max_lines * console_row_height)
        # calculate the actual height of the console window based on the number of lines
        console_window_height = self._console_max_lines * console_row_height
        # clear entire fixed console window area once
        self._tft.fillrect((x_offset, console_window_top_y), (Display.SCREEN_WIDTH - x_offset, console_window_height), TFT.BLACK)
        # yield control after clearing the rectangle
        await asyncio.sleep(0)
        for i, buffered_line in enumerate(self._console_buffer):
            current_line_y_pos = console_window_top_y + (i * console_row_height)
            self._tft.text((x_offset, current_line_y_pos), buffered_line, color, sysfont, 1.0)
            await asyncio.sleep(0)

    @staticmethod
    def fade_colors(start_rgb, end_rgb, steps):
        '''
        Generate a list of RGB tuples fading from start_rgb to end_rgb in given steps.
        '''
        fade_list = []
        for step in range(steps):
            interp = tuple(
                int(start + (float(step) / (steps - 1)) * (end - start))
                for start, end in zip(start_rgb, end_rgb)
            )
            fade_list.append(interp)
        return fade_list

    @staticmethod
    def to_tft_color(color):
        '''
        Convert a Color instance (or RGB tuple) to a TFTColor (RGB565 integer).

        Args:
            color (Color | tuple): An instance of Color or (r, g, b) tuple.

        Returns:
            int: TFTColor-compatible 16-bit color.
        '''
        if isinstance(color, Color):
            return TFTColor(*color.rgb)
        elif isinstance(color, tuple) and len(color) == 3:
            return TFTColor(*color)
        else:
            raise TypeError("Expected Color or (r, g, b) tuple, got {}".format(type(color)))

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def main():

    COLOR_TEST      = True
    COLORS_TEST     = True
    CONSOLE_TEST    = True
    HELLO_TEST      = True
    IP_ADDRESS_TEST = False
    FORMAT_TEST     = False
    MULTILINE_TEST  = False

    ORANGE  = Display.to_tft_color((255, 220, 0))
    MAGENTA = Display.to_tft_color(COLOR_MAGENTA)
    CYAN    = Display.to_tft_color(COLOR_CYAN)
    VIOLET  = Display.to_tft_color((180, 64, 240)) # was (138,  43, 226)
    APPLE   = Display.to_tft_color((216, 255, 0))

    display = None

    try:

        display = Display()
        time.sleep(1)

        if COLOR_TEST:
            display.show_line("row 1 white", line=1, size=1.2, color=TFT.WHITE, clear=True)
            display.show_line("row 2 red",   line=2, size=1.2, color=TFT.RED)
            display.show_line("row 3 green", line=3, size=1.2, color=TFT.GREEN)
            display.show_line("row 4 blue",  line=4, size=1.2, color=TFT.BLUE)
            time.sleep(3)

        if COLORS_TEST:

            # predefined colors:
            #   BLACK, RED, MAROON, GREEN, FOREST, BLUE,
            #   NAVY, CYAN, YELLOW, PURPLE, WHITE, GRAY
            # imported colors:
            #   COLOR_BLACK, COLOR_RED, COLOR_GREEN, COLOR_BLUE,
            #   COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
            #   COLOR_DARK_RED, COLOR_DARK_GREEN, COLOR_DARK_BLUE,
            #   COLOR_DARK_CYAN, COLOR_DARK_MAGENTA, COLOR_DARK_YELLOW,
            #   COLOR_ORANGE, COLOR_INDIGO, COLOR_VIOLET

            display.show_line("row 1 orange",  line=1, size=1.2, color=ORANGE, clear=True)
            display.show_line("row 2 magenta", line=2, size=1.2, color=MAGENTA)
            display.show_line("row 3 cyan",    line=3, size=1.2, color=CYAN)
            display.show_line("row 4 violet",  line=4, size=1.2, color=VIOLET)
            time.sleep(3)

        if CONSOLE_TEST:
            display.console(clear=True)
            lines = [
                "Line one",
                "Line two",
                "Line three",
                "Line four",
                "Line five",
                "Line six",
                "Line seven",
                "Line eight",
                "Line nine",
                "Line ten",
                "Line eleven",
                "Line twelve",
                "",
                "",
                "",
                "",
                "",
            ]
            for line in lines:
                display.console(line)
                time.sleep(0.15)
            time.sleep(3)

        if HELLO_TEST:
            display.hello(persist=True)
            time.sleep(3)

        if IP_ADDRESS_TEST:
            display.show_line(["", "IP address:", "192.168.1.73"], line=1, size=1.6, color=APPLE, clear=True)
            time.sleep(e)

        if FORMAT_TEST:
            h = 1.0
            display.show_line("row 1 ht: {}".format(h), line=1, size=h, clear=True)
            display.show_line("row 2 is boring",  line=2, size=h)
            display.show_line("row 3 is lost",    line=3, size=h)
            display.show_line("row 4 is roaring", line=4, size=h)
            display.show_line("row 5 is gone", line=5, size=h)
            time.sleep(3)

            heights = [ 1.0, 1.2, 1.4, 1.5, 1.6, 1.8, 2.0 ]
            for h in heights:
                display.show_line("row 1 ht: {}".format(h), line=1, size=h, clear=True)
                display.show_line("row 2",  line=2, size=h)
                display.show_line("row 3",    line=3, size=h)
                if h < 1.8:
                    display.show_line("row 4", line=4, size=h)
                time.sleep(3)

        if MULTILINE_TEST:
            lines = [ 'line 1', 'line 2', 'line 3' ]
            display.show_line(lines, line=1, size=1.2, color=TFT.WHITE)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print('{} raised: {}'.format(type(e), e))
    finally:
        if display:
            display.close()
        print('complete.')

# for REPL usage or testing
def exec():
    main()

if __name__ == "__main__":
    main()

#EOF
