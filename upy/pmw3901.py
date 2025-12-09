
# Note: PAA5100 class at bottom

import time
import ustruct as struct
from machine import Pin, SPI
import machine

WAIT = -1 # sentinel for waits in vendor sequences

class PMW3901:
    REG_PRODUCT_ID  = 0x00
    REG_MOTION      = 0x02
    REG_DELTA_X     = 0x03
    REG_DELTA_Y     = 0x04
    REG_SQUAL       = 0x05
    REG_PIXEL_SUM   = 0x06
    REG_SHUTTER     = 0x07
    REG_ORIENTATION = 0x5B
    '''
    MicroPython PMW3901 driver with PAA5100 support and Pimoroni "secret_sauce" init.

    Pass secret_sauce=True to call the vendor calibration sequence during __init__.

    Wiring table for defaults: SPI(1), CS='PA4':

        PAA5100 / PMW3901 pin -> STM32F405 pin
            VCC  -> 3V3 (3.3 V supply pin on WeAct)
            CS   -> PA4
            SCK  -> PA5
            MOSI -> PA7
            MISO -> PA6
            INT  -> PA0 (or PB0 / PB1 / any EXTI-capable GPIO you prefer)
            GND  -> GND
    '''
    def __init__(self, spi=None, cs=None, sck=None, mosi=None, miso=None,
                 baudrate=2000000, led_pin=None, led_pwm_freq=1000, secret_sauce=False):
        # SPI setup
        if spi is None:
            try:
                if sck or mosi or miso:
                    self.spi = SPI(1, baudrate=baudrate, polarity=1, phase=1,
                                   sck=sck, mosi=mosi, miso=miso)
                else:
                    self.spi = SPI(1, baudrate=baudrate, polarity=1, phase=1)
            except Exception:
                self.spi = SPI(1, baudrate=baudrate, polarity=1, phase=1)
        else:
            self.spi = spi
        # CS setup
        if cs is None:
            try:
                self.cs = Pin('PA4', Pin.OUT)
            except Exception:
                self.cs = Pin(4, Pin.OUT)
        else:
            self.cs = cs if isinstance(cs, Pin) else Pin(cs, Pin.OUT)
        self.cs.value(1)  # deselect
        self._rotation = 0
        self._orientation = 0
        # LED support (optional)
        self.led_pin = None
        self._led_pwm = None
        self._led_pwm_freq = led_pwm_freq
        self._saved_led_reg = None
        if led_pin is not None:
            self._setup_led(led_pin, led_pwm_freq)
        # optional vendor init
        if secret_sauce and hasattr(self, "_secret_sauce"):
            # Allow subclass to do its init/calibration sequence
            self._secret_sauce()

    # high-level API ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def init(self, registers=None, delay_after_ms=50):
        if not registers:
            return
        flat = []
        for reg, val in registers:
            flat.append(reg)
            flat.append(val)
        self._bulk_write(flat)
        time.sleep_ms(delay_after_ms)

    @property
    def id(self):
        return self.read_register(self.REG_PRODUCT_ID)

    @property
    def revision(self):
        return self.id

    def get_motion(self, use_burst=True):
        if use_burst:
            d = self.motion_burst()
            return {"motion": d["motion"], "dx": d["dx"], "dy": d["dy"], "squal": d["squal"]}
        else:
            motion = self.read_register(self.REG_MOTION)
            dx_raw = self.read_register(self.REG_DELTA_X)
            dy_raw = self.read_register(self.REG_DELTA_Y)
            dx = dx_raw - 256 if dx_raw & 0x80 else dx_raw
            dy = dy_raw - 256 if dy_raw & 0x80 else dy_raw
            squal = None
            try:
                squal = self.read_register(self.REG_SQUAL)
            except Exception:
                pass
            return {"motion": motion, "dx": dx, "dy": dy, "squal": squal}

    def get_motion_slow(self):
        time.sleep_ms(1)
        m = self.get_motion()
        time.sleep_ms(1)
        return m

    def set_rotation(self, deg):
        if deg not in (0, 90, 180, 270):
            raise ValueError("rotation must be one of 0, 90, 180, 270")
        self._rotation = deg

    def set_rotation(self, deg):
        '''
        Set orientation of PMW3901 in increments of 90 degrees. 
        
        Args:
            deg:  rotation in multiple of 90 degrees
        '''
        if deg == 0:
            self.set_orientation(invert_x=True, invert_y=True, swap_xy=True)
        elif deg == 90:
            self.set_orientation(invert_x=False, invert_y=True, swap_xy=False)
        elif deg == 180:
            self.set_orientation(invert_x=False, invert_y=False, swap_xy=True)
        elif deg == 270:
            self.set_orientation(invert_x=True, invert_y=False, swap_xy=False)
        else:
            raise ValueError("Degrees must be one of 0, 90, 180 or 270")
        self._rotation = deg

    def set_orientation(self, invert_x=True, invert_y=True, swap_xy=True):
        '''
        Set orientation of PMW3901 manually.
        
        Swapping is performed before flipping.
        
        Args:
            invert_x:  invert the X axis
            invert_y:  invert the Y axis
            swap_xy:   swap the X/Y axes
        '''
        value = 0
        if swap_xy:
            value |= 0b10000000
        if invert_y:
            value |= 0b01000000
        if invert_x: 
            value |= 0b00100000
        self. write_register(self.REG_ORIENTATION, value)
        self._orientation = value

    def reset(self, reset_pin=None, active_low=True):
        if reset_pin is None:
            return
        pin = reset_pin if isinstance(reset_pin, Pin) else Pin(reset_pin, Pin.OUT)
        if active_low:
            pin.value(0)
            time.sleep_ms(10)
            pin.value(1)
            time.sleep_ms(50)
        else:
            pin.value(1)
            time.sleep_ms(10)
            pin.value(0)
            time.sleep_ms(50)

    def dump_regs(self, start=0x00, length=32):
        out = {}
        for a in range(start, start + length):
            try:
                out[a] = self.read_register(a)
            except Exception:
                out[a] = None
        return out

    # LED helpers (optional) ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _setup_led(self, led_pin, freq):
        self.led_pin = led_pin if isinstance(led_pin, Pin) else Pin(led_pin, Pin.OUT)
        pwm_cls = getattr(machine, "PWM", None)
        if pwm_cls is not None:
            try:
                self._led_pwm = pwm_cls(self.led_pin)
                try:
                    self._led_pwm.freq(freq)
                except Exception:
                    pass
            except Exception:
                self._led_pwm = None
        else:
            self._led_pwm = None
        if self._led_pwm:
            self._set_pwm_duty(0)
        else:
            try:
                self.led_pin.value(0)
            except Exception:
                pass

    def _set_pwm_duty(self, val8):
        if not self._led_pwm:
            return
        try:
            self._led_pwm.duty_u16(int(val8 * 257))
            return
        except Exception:
            pass
        try:
            self._led_pwm.duty(int(val8 * 4))
            return
        except Exception:
            pass
        try:
            self._led_pwm.duty_cycle(val8 / 255.0)
            return
        except Exception:
            pass

    def set_led(self, brightness):
        b = int(max(0, min(255, brightness)))
        if self._led_pwm:
            self._set_pwm_duty(b)
        elif self.led_pin is not None:
            self.led_pin.value(1 if b > 0 else 0)

    def led_on(self):
        self.set_led(255)

    def led_off(self):
        self.set_led(0)

    def blink(self, times=3, on_ms=100, off_ms=100, brightness=255):
        for _ in range(times):
            self.set_led(brightness)
            time.sleep_ms(on_ms)
            self.set_led(0)
            time.sleep_ms(off_ms)

    # callback support ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def add_callback(self, callback, int_pin=None, trigger=Pin.IRQ_RISING):
        '''
        Attach callback directly to INT pin. callback is called in IRQ context.
        - callback(pin) will be invoked on the interrupt.
        - int_pin can be a Pin instance or a pin id; if omitted, uses self.int_pin.
        '''
        pin = int_pin if int_pin is not None else self.int_pin
        if not isinstance(pin, Pin):
            pin = Pin(pin, Pin.IN)
        self._int_pin = pin
        self._user_int_cb = callback
        pin.irq(trigger=trigger, handler=callback)

    def remove_callback(self):
        '''
        Detach any previously installed INT callback.
        '''
        self._int_pin.irq(handler=None)
        self._int_pin = None
        self._user_int_cb = None

    # low-level SPI helpers ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _cs_low(self):
        self.cs.value(0)

    def _cs_high(self):
        self.cs.value(1)

    def _delay_us(self, us):
        try:
            time.sleep_us(us)
        except AttributeError:
            time.sleep(us / 1000.0)

    def read_register(self, reg):
        addr = reg & 0x7F
        self._cs_low()
        self.spi.write(bytes([addr]))
        self._delay_us(35)
        buf = bytearray(1)
        self.spi.readinto(buf)
        self._delay_us(1)
        self._cs_high()
        return buf[0]

    def write_register(self, reg, value):
        addr = reg | 0x80
        self._cs_low()
        self.spi.write(bytes([addr, value & 0xFF]))
        self._delay_us(20)
        self._cs_high()

    def _read(self, register, length=1):
        out = []
        for i in range(length):
            out.append(self.read_register(register + i))
        if length == 1:
            return out[0]
        return out

    def _bulk_write(self, data):
        # data is a flat list: [reg, val, reg, val, ...], WAIT sentinel for sleeps
        for x in range(0, len(data), 2):
            register, value = data[x:x+2]
            if register == WAIT:
                time.sleep(value / 1000.0)
            else:
                self.write_register(register, value)

    def motion_burst(self):
        '''
        Basic short burst (7 bytes) used by generic PMW3901 read.
        Returns dict with fields parsed from the 7-byte burst.
        '''
        self._cs_low()
        self.spi.write(bytes([self.REG_MOTION & 0x7F]))
        self._delay_us(35)
        buf = bytearray(7)
        self.spi.readinto(buf)
        self._cs_high()
        motion = buf[0]
        dx_raw = buf[1]
        dy_raw = buf[2]
        squal = buf[3]
        pixel_sum = buf[4]
        shutter = (buf[6] << 8) | buf[5]
        dx = dx_raw - 256 if dx_raw & 0x80 else dx_raw
        dy = dy_raw - 256 if dy_raw & 0x80 else dy_raw
        return {"motion": motion, "dx": dx, "dy": dy, "squal": squal, "pixel_sum": pixel_sum, "shutter": shutter}

class PAA5100(PMW3901):
    REG_MOTION_BURST = 0x16

    def __init__(self, *args, secret_sauce=False, **kwargs):
        super().__init__(*args, **kwargs)
        self._use_16bit = False
        if secret_sauce:
            self._secret_sauce()

    def enable_16bit_mode(self, enable=True):
        self._use_16bit = bool(enable)

    def _read16_signed(self, low_reg):
        low = self.read_register(low_reg)
        high = self.read_register(low_reg + 1)
        val = (high << 8) | low
        if val & 0x8000:
            val = val - 0x10000
        return val

    def motion_burst(self):
        '''
        PAA5100 extended burst read (12 bytes) matching Pimoroni format.
        Parses fields and returns a dict with parsed fields.
        '''
        self._cs_low()
        self.spi.write(bytes([self.REG_MOTION_BURST & 0x7F]))
        self._delay_us(35)
        buf = bytearray(12)
        self.spi.readinto(buf)
        self._cs_high()

        # parse according to mapping:
        # buf[0] = dr, buf[1]=obs, buf[2..3]=x (low,high), buf[4..5]=y, buf[6]=quality,
        # buf[7]=raw_sum, buf[8]=raw_max, buf[9]=raw_min, buf[10]=shutter_upper, buf[11]=shutter_lower
        dr = buf[0]
        obs = buf[1]
        # little-endian signed 16-bit
        x = struct.unpack("<h", bytes([buf[2], buf[3]]))[0]
        y = struct.unpack("<h", bytes([buf[4], buf[5]]))[0]
        quality = buf[6]
        raw_sum = buf[7]
        raw_max = buf[8]
        raw_min = buf[9]
        shutter_upper = buf[10]
        shutter_lower = buf[11]
        return {"dr": dr, "obs": obs, "x": x, "y": y, "quality": quality,
                "raw_sum": raw_sum, "raw_max": raw_max, "raw_min": raw_min,
                "shutter_upper": shutter_upper, "shutter_lower": shutter_lower}

    def get_motion(self, timeout=5):
        '''
        Poll using the extended Pimoroni burst and validation.
        Returns (x, y) on success or raises RuntimeError on timeout.
        '''
        start = time.ticks_ms()
        timeout_ms = int(timeout * 1000)
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            d = self.motion_burst()
            dr = d["dr"]
            quality = d["quality"]
            shutter_upper = d["shutter_upper"]
            # Pimoroni validation:
            if (dr & 0b10000000) and not (quality < 0x19 and shutter_upper == 0x1F):
                return d["x"], d["y"]
            time.sleep_ms(10)
        raise RuntimeError("Timed out waiting for motion data.")

    def enable_sensor_led(self, value=None):
        '''
        Minimal enable: restores saved LED register or writes provided value.

        - If `value` is None this uses self._saved_led_reg (must exist if not passing value).
        - Writes reg 0x6F on page 0x14 and restores page 0x00.
        '''
        v = value if value is not None else self._saved_led_reg
        self.write_register(0x7F, 0x14)
        self.write_register(0x6F, v)
        self.write_register(0x7F, 0x00)

    def disable_sensor_led(self):
        '''
        Stop the sensor from driving the on-board LED.

        This writes the same register / page sequence used to enable LED pulsing,
        but clears the LED control register so the sensor stops pulsing the LED.
        Safe to call even if the LED wasn't enabled.

        Saves the current LED register value to self._saved_led_reg if readable,
        then clears the LED control register (0x6F) on page 0x14.

        Safe to call in a finally/cleanup block (best-effort).
        '''
        try:
            self.write_register(0x7F, 0x14)
        except Exception:
            return
        try:
            try:
                self._saved_led_reg = self.read_register(0x6F)
            except Exception:
                self._saved_led_reg = None
            # clear LED control register so the sensor stops driving the LED
            try:
                self.write_register(0x6F, 0x00)
            except Exception:
                pass
        finally:
            try:
                self.write_register(0x7F, 0x00)
            except Exception:
                pass

    def x_disable_sensor_led(self):
        '''
        Stop the sensor from driving the on-board LED.

        This writes the same register / page sequence used to enable LED pulsing,
        but clears the LED control register so the sensor stops pulsing the LED.
        Safe to call even if the LED wasn't enabled.
        '''
        try:
            self._bulk_write([0x7F, 0x14, 0x6F, 0x00, 0x7F, 0x00])
        except Exception:
            try:
                self.write_register(0x7F, 0x14)
                self.write_register(0x6F, 0x00)
                self.write_register(0x7F, 0x00)
            except Exception:
                pass

    def _secret_sauce(self):
        '''
        Ported Pimoroni PAA5100 vendor init sequence.
        Safe to call from __init__ by passing secret_sauce=True.
        '''
        seq = [
            0x7F, 0x00,
            0x55, 0x01,
            0x50, 0x07,
            0x7F, 0x0E,
            0x43, 0x10
        ]
        self._bulk_write(seq)

        if self._read(0x67) & 0b10000000:
            self.write_register(0x48, 0x04)
        else:
            self.write_register(0x48, 0x02)

        self._bulk_write([
            0x7F, 0x00,
            0x51, 0x7B,
            0x50, 0x00,
            0x55, 0x00,
            0x7F, 0x0E
        ])

        if self._read(0x73) == 0x00:
            c1 = self._read(0x70)
            c2 = self._read(0x71)
            if c1 <= 28:
                c1 += 14
            if c1 > 28:
                c1 += 11
            c1 = max(0, min(0x3F, c1))
            c2 = (c2 * 45) // 100
            self._bulk_write([
                0x7F, 0x00,
                0x61, 0xAD,
                0x51, 0x70,
                0x7F, 0x0E
            ])
            self.write_register(0x70, c1)
            self.write_register(0x71, c2)

        self._bulk_write([
            0x7F, 0x00,
            0x61, 0xAD,
            0x7F, 0x03,
            0x40, 0x00,
            0x7F, 0x05,
            0x41, 0xB3,
            0x43, 0xF1,
            0x45, 0x14,
            0x5F, 0x34,
            0x7B, 0x08,
            0x5E, 0x34,
            0x5B, 0x11,
            0x6D, 0x11,
            0x45, 0x17,
            0x70, 0xE5,
            0x71, 0xE5,
            0x7F, 0x06,
            0x44, 0x1B,
            0x40, 0xBF,
            0x4E, 0x3F,
            0x7F, 0x08,
            0x66, 0x44,
            0x65, 0x20,
            0x6A, 0x3A,
            0x61, 0x05,
            0x62, 0x05,
            0x7F, 0x09,
            0x4F, 0xAF,
            0x5F, 0x40,
            0x48, 0x80,
            0x49, 0x80,
            0x57, 0x77,
            0x60, 0x78,
            0x61, 0x78,
            0x62, 0x08,
            0x63, 0x50,
            0x7F, 0x0A,
            0x45, 0x60,
            0x7F, 0x00,
            0x4D, 0x11,
            0x55, 0x80,
            0x74, 0x21,
            0x75, 0x1F,
            0x4A, 0x78,
            0x4B, 0x78,
            0x44, 0x08,
            0x45, 0x50,
            0x64, 0xFF,
            0x65, 0x1F,
            0x7F, 0x14,
            0x65, 0x67,
            0x66, 0x08,
            0x63, 0x70,
            0x6F, 0x1C,
            0x7F, 0x15,
            0x48, 0x48,
            0x7F, 0x07,
            0x41, 0x0D,
            0x43, 0x14,
            0x4B, 0x0E,
            0x45, 0x0F,
            0x44, 0x42,
            0x4C, 0x80,
            0x7F, 0x10,
            0x5B, 0x02,
            0x7F, 0x07,
            0x40, 0x41,
            WAIT, 0x0A,
            0x7F, 0x00,
            0x32, 0x00,
            0x7F, 0x07,
            0x40, 0x40,
            0x7F, 0x06,
            0x68, 0xF0,
            0x69, 0x00,
            0x7F, 0x0D,
            0x48, 0xC0,
            0x6F, 0xD5,
            0x7F, 0x00,
            0x5B, 0xA0,
            0x4E, 0xA8,
            0x5A, 0x90,
            0x40, 0x80,
            0x73, 0x1F,
            WAIT, 0x0A,
            0x73, 0x00
        ])

#EOF
