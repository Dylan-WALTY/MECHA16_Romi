from time import ticks_us, ticks_diff
from pyb import Timer, Pin


class Encoder:
    """Quadrature encoder interface using STM32 hardware timer ENC_AB mode.

    Tracks accumulated position and instantaneous velocity with automatic
    16-bit counter overflow handling. Velocity is returned in mm/s.
    """

    def __init__(self, tim: int, chA_pin: Pin, chB_pin: Pin):
        """Configure the timer in ENC_AB mode and initialise tracking state."""
        self.tim = Timer(tim, period=0xFFFF, prescaler=0)
        self.tim.channel(1, pin=chA_pin, mode=Timer.ENC_AB)
        self.tim.channel(2, pin=chB_pin, mode=Timer.ENC_AB)

        self.wheel_radius       = 35        # mm
        self.counts_per_rev     = 1440      # 4× quadrature of 360 PPR
        self.PI                 = 3.14159
        self.wheel_circumference = 2 * self.PI * self.wheel_radius  # mm

        self.position   = 0
        self.prev_count = self.tim.counter()
        self.delta      = 0
        self.dt         = 0
        self.prev_time  = ticks_us()

    def update(self):
        """Read the timer counter, accumulate position, and compute dt.

        Handles 16-bit rollover in both directions.
        """
        self.now   = ticks_us()
        self.count = self.tim.counter()

        self.delta = self.count - self.prev_count
        if self.delta < -32768:
            self.delta += 65536
        elif self.delta > 32768:
            self.delta -= 65536

        self.position  += self.delta
        self.dt         = ticks_diff(self.now, self.prev_time) / 1_000_000.0
        self.prev_count = self.count
        self.prev_time  = self.now

    def get_position(self):
        """Return accumulated encoder position in counts (negated for convention)."""
        return -self.position

    def get_velocity(self):
        """Return instantaneous wheel velocity in mm/s; returns 0 if dt is too small."""
        return ((-self.delta / self.dt) * (self.wheel_circumference / self.counts_per_rev)
                if self.dt > 0.0000001 else 0)

    def zero(self):
        """Reset accumulated position to zero and re-anchor the count baseline."""
        self.position   = 0
        self.prev_count = self.tim.counter()
