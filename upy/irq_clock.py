
from pyb import Pin, Timer

e4 = Pin('E4', Pin.OUT_PP)

tim = Timer(15, freq=20)
ch = tim.channel(1, Timer.PWM, pin=e4, pulse_width_percent=50)

