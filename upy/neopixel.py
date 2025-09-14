# NeoPixel driver for MicroPython
# MIT license; Copyright (c) 2016 Damien P. George, 2021 Jim Mussared

from machine import bitstream


class NeoPixel:
    ORDER_MAP = {
        "RGB": (0, 1, 2),
        "GRB": (1, 0, 2),
        "BRG": (2, 0, 1),
        "BGR": (2, 1, 0),
        "RBG": (0, 2, 1),
        "GBR": (1, 2, 0),
    }

    ORDER = (0, 1, 2)

    def __init__(self, pin, n, bpp=3, timing=1, color_order="RGB", brightness=1.0):
        self.pin = pin
        self.n = n
        self.bpp = bpp
        self.buf = bytearray(n * bpp)
        self.pin.init(pin.OUT)
        self.brightness = brightness
        # Timing arg can either be 1 for 800kHz or 0 for 400kHz,
        # or a user-specified timing ns tuple (high_0, low_0, high_1, low_1).
        self.timing = (
            ((400, 850, 800, 450) if timing else (800, 1700, 1600, 900))
            if isinstance(timing, int)
            else timing
        )
        if bpp == 3:
            if color_order not in self.ORDER_MAP:
                raise ValueError("Unsupported color_order: {}".format(color_order))
            self.ORDER = self.ORDER_MAP[color_order]

    def __len__(self):
        return self.n

    def __setitem__(self, i, v):
        offset = i * self.bpp
        for i in range(self.bpp):
            self.buf[offset + self.ORDER[i]] = int(v[i] * self.brightness)

    def __getitem__(self, i):
        offset = i * self.bpp
        return tuple(self.buf[offset + self.ORDER[i]] for i in range(self.bpp))

    def fill(self, v):
        b = self.buf
        l = len(self.buf)
        bpp = self.bpp
        for i in range(bpp):
            c = int(v[i] * self.brightness)
            j = self.ORDER[i]
            while j < l:
                b[j] = c
                j += bpp

    def write(self):
        # BITSTREAM_TYPE_HIGH_LOW = 0
        bitstream(self.pin, 0, self.timing, self.buf)

