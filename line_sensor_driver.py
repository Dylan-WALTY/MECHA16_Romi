from pyb import Pin, ADC


class line_sensor:
    """8-channel reflectance sensor array driver.

    Reads raw ADC values from up to 8 IR sensors, supports per-sensor
    white/black calibration, and computes a signed centroid error for
    use as the PID process variable in line-following control.
    """

    def __init__(self, adc_pins: list, ctrl_pin: Pin):
        """Set up ADC objects and activate the sensor emitters via ctrl_pin."""
        self.adcs     = [ADC(pin) for pin in adc_pins]
        self.ctrl_pin = Pin(ctrl_pin, mode=Pin.OUT_PP)
        self.ctrl_pin.high()
        self.white = [0]    * len(self.adcs)
        self.black = [4095] * len(self.adcs)

    def read_sensors(self, samples=10):
        """Return averaged raw ADC readings across all sensors."""
        vals = [0] * len(self.adcs)
        for _ in range(samples):
            for i, adc in enumerate(self.adcs):
                vals[i] += adc.read()
        return [v // samples for v in vals]

    def calibrate_white(self):
        """Capture and store the white-surface calibration baseline."""
        self.white = self.read_sensors()

    def calibrate_black(self):
        """Capture and store the black-line calibration baseline."""
        self.black = self.read_sensors()

    def read_normalized(self):
        """Return per-sensor values normalised to [0.0 = white, 1.0 = black]."""
        raw        = self.read_sensors()
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
        """Return the weighted centroid of the sensor array.

        The centroid is centred so that 0.0 means the line is directly
        underneath the array midpoint; negative values indicate the line
        is to the left and positive to the right.

        Returns None if no sensor detects the line (total weight == 0).
        """
        vals         = self.read_normalized()
        weighted_sum = sum(i * v for i, v in enumerate(vals))
        total        = sum(vals)
        if total == 0:
            return None
        return weighted_sum / total - (len(self.adcs) - 1) / 2
