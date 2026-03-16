from time import ticks_us, ticks_diff   # Use to get dt value in update()
from pyb import Timer, Pin


class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class'''

    def __init__(self, tim: int, chA_pin: Pin, chB_pin: Pin):
        '''Initializes an Encoder object'''
        self.tim = Timer(tim, period = 0xFFFF, prescaler = 0)
        self.tim.channel(1, pin=chA_pin, mode=Timer.ENC_AB)
        self.tim.channel(2, pin=chB_pin, mode=Timer.ENC_AB)
        self.wheel_radius = 35 # in mm
        self.counts_per_rev = 1440 # Counts per revolution of the encoder (4 times the 360 PPR due to quadrature encoding)
        self.PI = 3.14159
        self.wheel_circumference = 2 * self.PI * self.wheel_radius # in mm
        self.position   = 0     # Total accumulated position of the encoder
        self.prev_count = self.tim.counter()     # Counter value from the most recent update
        self.delta      = 0     # Change in count between last two updates
        self.dt         = 0     # Amount of time between last two updates
        self.prev_time  = ticks_us()     # Time of the most recent update in microseconds
    
    def update(self):
        '''Runs one update step on the encoder's timer counter to keep
           track of the change in count and check for counter reload'''
        
        self.now = ticks_us() # Current time in microseconds
        self.count = self.tim.counter() # Current count from the timer counter u
        
        self.delta = self.count - self.prev_count
        if self.delta < -32768:  # Handle counter rollover (upward)
            self.delta += 65536
        elif self.delta > 32768:  # Handle counter rollover (downward)
            self.delta -= 65536

        self.position += self.delta # Update position with the change in count
        self.dt = ticks_diff(self.now, self.prev_time) / 1000000.0 # Time difference in seconds
        self.prev_count = self.count # Update previous count for next iteration
        self.prev_time = self.now # Update previous time for next iteration
        pass
            
    def get_position(self):
        '''Returns the most recently updated value of position as determined
           within the update() method'''
        return -self.position
            
    def get_velocity(self):
        '''Returns a measure of velocity using the the most recently updated
           value of delta as determined within the update() method'''
        return (-self.delta/self.dt) * (self.wheel_circumference/self.counts_per_rev) if self.dt > 0.0000001 else 0
    
    def zero(self):
        '''Sets the present encoder position to zero and causes future updates
           to measure with respect to the new zero position'''
        self.position = 0 # Reset position to zero
        self.prev_count = self.tim.counter() # Reset previous count to zero to measure from
        pass