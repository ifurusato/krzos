
import sys

class Chadburn:
    _instances = []

    def __init__(self, index, name, pwm, direction, description):
        self._index = index
        self._name = name
        self._pwm = pwm
        self._direction = direction
        self._description = description
        Chadburn._instances.append(self)

    @property
    def index(self):
        return self._index

    @property
    def name(self):
        return self._name

    @property
    def pwm(self):
        return self._pwm

    @property
    def direction(self):
        return self._direction

    @property
    def description(self):
        return self._description

    def __str__(self):
        return "{}: pwm={}%, dir={}, desc='{}'".format(
            self._name, self._pwm, self._direction, self._description)

    def __repr__(self):
        return "Chadburn({}, pwm={}, dir={}, desc='{}')".format(
            self._name, self._pwm, self._direction, self._description)

    @classmethod
    def all(cls):
        return cls._instances

    @classmethod
    def from_index(cls, index):
        for inst in cls._instances:
            if inst.index == index:
                return inst
        raise ValueError("No Chadburn entry with index '{}'".format(index))

Chadburn.EMERGENCY_ASTERN     = Chadburn(-8, "EMERGENCY_ASTERN",     0,   -1, "emergency astern")
Chadburn.FULL_ASTERN          = Chadburn(-7, "FULL_ASTERN",          15,  -1, "full astern")
Chadburn.THREE_QUARTER_ASTERN = Chadburn(-6, "THREE_QUARTER_ASTERN", 30,  -1, "three quarters astern")
Chadburn.TWO_THIRDS_ASTERN    = Chadburn(-5, "TWO_THIRDS_ASTERN",    38,  -1, "two thirds astern")
Chadburn.HALF_ASTERN          = Chadburn(-4, "HALF_ASTERN",          54,  -1, "half astern")
Chadburn.ONE_THIRD_ASTERN     = Chadburn(-3, "ONE_THIRD_ASTERN",     61,  -1, "one third astern")
Chadburn.SLOW_ASTERN          = Chadburn(-2, "SLOW_ASTERN",          79,  -1, "slow astern")
Chadburn.DEAD_SLOW_ASTERN     = Chadburn(-1, "DEAD_SLOW_ASTERN",     93,  -1, "dead slow astern")

Chadburn.STOP                 = Chadburn( 0, "STOP",                100,   0, "stop")

Chadburn.DEAD_SLOW_AHEAD      = Chadburn( 1, "DEAD_SLOW_AHEAD",      93,   1, "dead slow ahead")
Chadburn.SLOW_AHEAD           = Chadburn( 2, "SLOW_AHEAD",           79,   1, "slow ahead")
Chadburn.ONE_THIRD_AHEAD      = Chadburn( 3, "ONE_THIRD_AHEAD",      61,   1, "one third ahead")
Chadburn.HALF_AHEAD           = Chadburn( 4, "HALF_AHEAD",           54,   1, "half ahead")
Chadburn.TWO_THIRDS_AHEAD     = Chadburn( 5, "TWO_THIRDS_AHEAD",     38,   1, "two thirds ahead")
Chadburn.THREE_QUARTER_AHEAD  = Chadburn( 6, "THREE_QUARTER_AHEAD",  30,   1, "three quarters ahead")
Chadburn.FULL_AHEAD           = Chadburn( 7, "FULL_AHEAD",           15,   1, "full ahead")
Chadburn.FLANK_AHEAD          = Chadburn( 8, "FLANK_AHEAD",           0,   1, "flank ahead")

# export Chadburn instances to module-level names
_mod_globals = sys.modules[__name__].__dict__
for inst in Chadburn._instances:
    _mod_globals[inst.name] = inst

#EOF
