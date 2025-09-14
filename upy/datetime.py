
import time

class timedelta:
    def __init__(self, days=0, seconds=0, minutes=0, hours=0):
        self.total_seconds = (
            days * 86400 + hours * 3600 + minutes * 60 + seconds
        )

    def __add__(self, other):
        if isinstance(other, datetime):
            return other + self
        raise TypeError("Unsupported operand type(s) for +")

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        return timedelta(seconds=self.total_seconds - other.total_seconds)

    def __repr__(self):
        return f"timedelta(seconds={self.total_seconds})"


class datetime:
    def __init__(self, year, month, day, hour=0, minute=0, second=0):
        self.struct_time = (year, month, day, hour, minute, second, 0, 0)
    
    @classmethod
    def now(cls):
        t = time.localtime()
        return cls(*t[:6])
    
    def timestamp(self):
        return time.mktime(self.struct_time + (0, 0))

    def __add__(self, delta):
        if not isinstance(delta, timedelta):
            raise TypeError("Can only add timedelta to datetime")
        new_ts = self.timestamp() + delta.total_seconds
        t = time.localtime(new_ts)
        return datetime(*t[:6])

    def __sub__(self, other):
        if isinstance(other, timedelta):
            new_ts = self.timestamp() - other.total_seconds
            t = time.localtime(new_ts)
            return datetime(*t[:6])
        elif isinstance(other, datetime):
            return timedelta(seconds=self.timestamp() - other.timestamp())
        raise TypeError("Unsupported operand type(s) for -")

    def __str__(self):
        return "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(*self.struct_time[:6])

    def __repr__(self):
        return f"datetime({', '.join(map(str, self.struct_time[:6]))})"

    def date(self):
        return self.struct_time[:3]

    def time(self):
        return self.struct_time[3:6]

