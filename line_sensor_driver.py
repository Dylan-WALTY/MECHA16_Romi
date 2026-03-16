from pyb import Pin, ADC

class line_sensor:
    '''A IR sensor driver interface encapsulated in a Python class. Connects to 
    8 Pins through analog in to read ADC values. Controls Odd.'''
   
    def __init__(self, adc_pins: list, ctrl_pin: Pin):
        '''Initializes a ir sensor object'''
        self.adcs = [ADC(pin) for pin in adc_pins]
        self.ctrl_pin = Pin(ctrl_pin, mode=Pin.OUT_PP)
        self.ctrl_pin.high()
        self.white = [0] * len(self.adcs)
        self.black = [4095] * len(self.adcs)
        
    def read_sensors(self, samples = 10):
        '''Reads the IR sensors and returns the ADC values'''
        vals = [0] * len(self.adcs)
        for _ in range(samples):
            for i, adc in enumerate(self.adcs):
                vals[i] += adc.read()
        return [v // samples for v in vals]
    
    def calibrate_white(self):
        ''' Read and store white surface calibration datum'''
        self.white = self.read_sensors()

    def calibrate_black(self):
        ''' Read and store black surface calibration datum'''
        self.black = self.read_sensors()

    def read_normalized(self):
        ''' Returns normalized ADC reading between 0(white) and 1(black)'''
        raw = self.read_sensors()
        normalized = []
        for i in range(len(self.adcs)):
            span = self.black[i] - self.white[i]
            if span == 0:
                normalized.append(0.0)
            else:
                val = (raw[i] - self.white[i]) / span
                normalized.append(max(0.0, min(1.0, val)))
        return normalized
    
    def get_centroid(self):
        ''' Returns the centroid of the sensor array, 0 is center, negative is left, positive is right'''
        vals = self.read_normalized()
        weighted_sum = sum(i * v for i, v in enumerate (vals))
        total = sum(vals)
        if total == 0:
            return None
        raw_centroid = weighted_sum / total
        return raw_centroid - (len(self.adcs) - 1) / 2