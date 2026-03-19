from pyb import Pin, Timer


class Motor:
    """DRV8838 H-bridge motor driver.

    Controls motor direction and speed via a PWM channel and two
    digital direction/enable pins. Effort is expressed as a signed
    percentage in the range [-100, 100].
    """

    def __init__(self, PWM_pin: Pin, DIR_pin: Pin, nSLP_pin: Pin,
                 tim: Timer, chan: int):
        """Configure PWM channel, direction pin, and nSLEEP enable pin."""
        self.PWM_chan = tim.channel(chan, pin=PWM_pin,
                                   mode=Timer.PWM, pulse_width_percent=0)
        self.DIR_pin  = Pin(DIR_pin,  mode=Pin.OUT_PP)
        self.nSLP_pin = Pin(nSLP_pin, mode=Pin.OUT_PP)

    def set_effort(self, effort):
        """Set motor effort in [-100, 100]; negative values reverse direction."""
        if effort >= 0:
            self.DIR_pin.low()
            self.PWM_chan.pulse_width_percent(effort)
        else:
            self.DIR_pin.high()
            self.PWM_chan.pulse_width_percent(-effort)

    def enable(self):
        """Take the driver out of sleep mode (brake state, zero effort)."""
        self.set_effort(0)
        self.nSLP_pin.high()

    def disable(self):
        """Put the driver into sleep mode, cutting motor power."""
        self.nSLP_pin.low()
