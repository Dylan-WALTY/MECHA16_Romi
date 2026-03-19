from pyb import Pin, I2C
from struct import calcsize, unpack_from, pack_into
from utime import sleep_ms
import os


class BNO055:
    """I2C driver for the Bosch BNO055 9-DOF IMU.

    Operates the sensor in IMU mode (0x08) — gyroscope + accelerometer
    fusion only, no magnetometer. All register reads and writes retry up
    to 5 times with a 50 ms recovery delay to tolerate I2C glitches
    caused by motor vibration. Calibration coefficients can be saved to
    and restored from a plain-text file to avoid a full recalibration on
    every boot.
    """

    DEV_ADDR = 0x28

    class reg:
        EUL_HEADING  = (0x1A, b"<h")
        EUL_DATA_ALL = (0x1A, b"<hhh")
        GYR_DATA_ALL = (0x14, b"<hhh")
        GYR_DATA_Z   = (0x18, b"<h")
        OPR_MODE     = (0x3D, b"<B")
        CALIB_STAT   = (0x35, b"<B")
        CAL_DATA     = (0x55, b"<hhhhhhhhhhh")

    def __init__(self, i2c: I2C, RST: Pin):
        """Reset the BNO055 and wait for it to boot (≥ 650 ms)."""
        self._i2c     = i2c
        self._rst_pin = Pin(RST, mode=Pin.OUT_PP)
        self._buf     = bytearray(22)

        self._rst_pin.low()
        sleep_ms(10)
        self._rst_pin.high()
        sleep_ms(1000)

    def read_reg(self, reg):
        """Read a register; retries up to 5 times on OSError with 50 ms delay."""
        length = calcsize(reg[1])
        buf    = memoryview(self._buf)[:length]
        last_err = None
        for _ in range(5):
            try:
                self._i2c.mem_read(buf, BNO055.DEV_ADDR, reg[0])
                return unpack_from(reg[1], buf)
            except OSError as e:
                last_err = e
                sleep_ms(50)
        raise last_err

    def write_reg(self, reg, *value):
        """Write a register; retries up to 5 times on OSError with 50 ms delay."""
        length = calcsize(reg[1])
        buf    = memoryview(self._buf)[:length]
        pack_into(reg[1], buf, 0, *value)
        last_err = None
        for _ in range(5):
            try:
                self._i2c.mem_write(buf, BNO055.DEV_ADDR, reg[0])
                return
            except OSError as e:
                last_err = e
                sleep_ms(50)
        raise last_err

    def mode_fusion(self, mode):
        """Switch operating mode, always passing through CONFIG (0x00) first.

        Common modes: 0x08 = IMU (accel+gyro), 0x0C = NDOF (all sensors).
        """
        self.write_reg(BNO055.reg.OPR_MODE, 0x00)
        sleep_ms(25)
        self.write_reg(BNO055.reg.OPR_MODE, mode)
        sleep_ms(20)

    def calibration_status(self):
        """Return a dict with calibration levels (0–3) for sys, gyro, accel, mag."""
        status = self.read_reg(BNO055.reg.CALIB_STAT)[0]
        return {
            "sys":   (status >> 6) & 0x03,
            "gyro":  (status >> 4) & 0x03,
            "accel": (status >> 2) & 0x03,
            "mag":   (status >> 0) & 0x03,
        }

    def is_calibrated(self, check_mag=False):
        """Return True when the required sensors reach calibration level 3.

        In IMU mode (0x08) only gyro calibration is checked; accel never
        reaches 3 in this mode and mag is unused.
        """
        s = self.calibration_status()
        if check_mag:
            return s["gyro"] == 3
        return s["gyro"] == 3

    def manual_calibrate(self, check_mag=False):
        """Block until required sensors are fully calibrated, printing progress."""
        print("\n--- BNO055 Calibration ---")
        if check_mag:
            print("GYRO : Hold the robot perfectly still for a few seconds")
            print("ACCEL: Slowly tilt through several stable orientations")
            print("MAG  : Move the robot in a figure-8 pattern")
        else:
            print("GYRO : Set the robot on a flat surface and hold it PERFECTLY STILL")
            print("       Wait for GYRO to reach 3/3. This usually takes 5-10 seconds.")
            print("       (ACCEL will show 0 in IMU mode - this is normal, ignore it)")
        print()

        while True:
            try:
                if self.is_calibrated(check_mag):
                    break
                s = self.calibration_status()
                if check_mag:
                    print("GYRO:{gyro}/3  ACCEL:{accel}/3  MAG:{mag}/3".format(**s), end="\r")
                else:
                    print("GYRO:{gyro}/3  (waiting for 3/3 -- hold still)     ".format(**s), end="\r")
            except OSError:
                print("(I2C glitch, retrying...)                             ", end="\r")
            sleep_ms(500)

        print("\nCalibration complete!")

    def read_calibration_coeff(self):
        """Read and return the 11 raw calibration coefficients from the sensor."""
        return self.read_reg(BNO055.reg.CAL_DATA)

    def write_calibration_coeff(self, coeffs):
        """Write 11 calibration coefficients to the sensor (must be in CONFIG mode)."""
        self.write_reg(BNO055.reg.CAL_DATA, *coeffs)

    def save_calibration(self, filename="calibration.txt"):
        """Switch to CONFIG, read coefficients, save to file, then restore IMU mode."""
        self.write_reg(BNO055.reg.OPR_MODE, 0x00)
        sleep_ms(25)
        coeffs = self.read_calibration_coeff()
        with open(filename, "w") as f:
            f.write(",".join(str(c) for c in coeffs))
        print("Calibration saved to {}".format(filename))
        self.write_reg(BNO055.reg.OPR_MODE, 0x08)
        sleep_ms(20)
        return coeffs

    def load_and_apply_calibration(self, filename="calibration.txt"):
        """Load coefficients from file and write to sensor in CONFIG mode.

        Returns True on success, False if the file is missing or malformed.
        """
        try:
            files = os.listdir()
        except OSError:
            files = []
        if filename not in files:
            return False
        try:
            with open(filename, "r") as f:
                data = f.read().strip()
            coeffs = tuple(int(x) for x in data.split(","))
            if len(coeffs) != 11:
                print("calibration.txt: wrong value count ({} vs 11)".format(len(coeffs)))
                return False
        except (OSError, ValueError) as e:
            print("Failed to read {}: {}".format(filename, e))
            return False
        self.write_reg(BNO055.reg.OPR_MODE, 0x00)
        sleep_ms(50)
        self.write_calibration_coeff(coeffs)
        sleep_ms(25)
        print("Calibration loaded and applied.")
        return True

    def get_euler(self):
        """Return (heading, roll, pitch) in degrees from the Euler register."""
        head, roll, pitch = self.read_reg(BNO055.reg.EUL_DATA_ALL)
        return (head / 16.0, roll / 16.0, pitch / 16.0)

    def get_yaw(self):
        """Return the absolute heading (yaw) in degrees."""
        return self.read_reg(BNO055.reg.EUL_HEADING)[0] / 16.0

    def get_angle_vel(self):
        """Return (x, y, z) gyroscope rates in degrees/s."""
        x, y, z = self.read_reg(BNO055.reg.GYR_DATA_ALL)
        return (x / 16.0, y / 16.0, z / 16.0)

    def get_yaw_rate(self):
        """Return the Z-axis (yaw) angular rate in degrees/s."""
        return self.read_reg(BNO055.reg.GYR_DATA_Z)[0] / 16.0
