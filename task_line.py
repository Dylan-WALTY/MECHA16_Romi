from task_share          import Share, Queue
from closed_loop_driver  import closed_loop_control
from utime               import ticks_us, ticks_diff
import micropython
from line_sensor_driver  import line_sensor

S0_INIT = micropython.const(0)
S1_READ = micropython.const(1)
S2_CAL  = micropython.const(2)


class task_line:
    """Line sensor reading and PID steering correction task.

    Reads the 8-channel IR array every period, computes a centroid error,
    and runs a PID controller to produce a steering correction shared with
    the motor tasks. Supports two-step white/black calibration triggered
    by a shared flag from task_user.

    States
    ------
    S0_INIT : Zero the controller and shared outputs, then move to S1_READ.
    S1_READ : Main loop — compute centroid and correction while lineGo is True.
    S2_CAL  : Perform one calibration step (white first, then black) and return.
    """

    def __init__(self,
                 line: line_sensor, loop: closed_loop_control,
                 calibrateIRFlag, steeringCorrection, baseSpeed,
                 lineGo, cen_data, cen_time,
                 centroid_share: Share,
                 line_detected_share: Share):
        """Bind sensor, controller, and all inter-task shares."""
        self._state: int = S0_INIT

        self._line = line
        self._loop = loop

        self._steeringCorrection: Share = steeringCorrection
        self._baseSpeed:          Share = baseSpeed
        self._lineGo:             Share = lineGo
        self._lastTime                  = ticks_us()
        self._calStep                   = 0
        self._calibrateIRFlag:    Share = calibrateIRFlag
        self._cen_data:           Queue = cen_data
        self._cen_time:           Queue = cen_time
        self._startTime                 = ticks_us()
        self._wasRunning                = False

        self._centroid_share:      Share = centroid_share
        self._line_detected_share: Share = line_detected_share

        print("Line Sensor Task object instantiated")

    def run(self):
        while True:

            # S0: initialise controller and shared outputs
            if self._state == S0_INIT:
                self._loop.set_set_point(0)
                self._centroid_share.put(0.0)
                self._line_detected_share.put(False)
                self._state = S1_READ

            # S1: read sensors, run PID, publish correction
            elif self._state == S1_READ:

                if self._lineGo.get() and not self._wasRunning:
                    self._startTime = ticks_us()
                    self._lastTime  = ticks_us()
                    self._cen_data.clear()
                    self._cen_time.clear()
                    self._loop.reset()
                    self._wasRunning = True

                if not self._lineGo.get():
                    self._wasRunning = False
                    self._steeringCorrection.put(0.0)
                    cen = self._line.get_centroid()
                    self._centroid_share.put(cen if cen is not None else 0.0)
                    self._line_detected_share.put(cen is not None)
                    yield self._state
                    continue

                cen = self._line.get_centroid()
                self._centroid_share.put(cen if cen is not None else 0.0)
                self._line_detected_share.put(cen is not None)

                if cen is None:
                    cen = 0

                t  = ticks_us()
                dt = ticks_diff(t, self._lastTime) / 1_000_000.0
                self._lastTime = t

                correction = self._loop.c_loop(cen, dt)
                self._steeringCorrection.put(correction)

                if not self._cen_data.full():
                    self._cen_data.put(cen)
                    self._cen_time.put(int(ticks_diff(t, self._startTime) / 1000))

                if self._calibrateIRFlag.get():
                    self._calibrateIRFlag.put(False)
                    self._state = S2_CAL

            # S2: single calibration step (alternates white → black)
            elif self._state == S2_CAL:
                if self._calStep == 0:
                    self._line.calibrate_white()
                    self._calStep = 1
                else:
                    self._line.calibrate_black()
                    self._calStep = 0
                self._state = S1_READ

            yield self._state
