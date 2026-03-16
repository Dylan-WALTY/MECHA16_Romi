from task_share          import Share, Queue
from closed_loop_driver  import closed_loop_control
from utime               import ticks_us, ticks_diff
import micropython
from line_sensor_driver import line_sensor

S0_INIT = micropython.const(0)
S1_READ = micropython.const(1)
S2_CAL  = micropython.const(2)

class task_line:
    '''
    Line sensor task.
    Added: centroid_share — a Share that master task reads to detect line-lost
    condition (value is 0.0 when no line found, nonzero when on line).
    line_detected_share — True/False Share master uses to trigger garage entry.
    '''

    def __init__(self,
                 line: line_sensor, loop: closed_loop_control,
                 calibrateIRFlag, steeringCorrection, baseSpeed,
                 lineGo, cen_data, cen_time,
                 centroid_share: Share,        # ← NEW: master reads this to detect line lost
                 line_detected_share: Share):  # ← NEW: True if line currently visible
        
        self._state: int = S0_INIT

        self._line = line
        self._loop = loop

        self._steeringCorrection: Share = steeringCorrection
        self._baseSpeed: Share = baseSpeed
        self._lineGo: Share = lineGo
        self._lastTime = ticks_us()
        self._calStep = 0
        self._calibrateIRFlag: Share = calibrateIRFlag
        self._cen_data: Queue = cen_data
        self._cen_time: Queue = cen_time
        self._startTime = ticks_us()
        self._wasRunning = False

        self._centroid_share: Share       = centroid_share       # ← NEW
        self._line_detected_share: Share  = line_detected_share  # ← NEW

        print("Line Sensor Task object instantiated")

    def run(self):
        while True:

            if self._state == S0_INIT:
                self._loop.set_set_point(0)
                self._centroid_share.put(0.0)
                self._line_detected_share.put(False)
                self._state = S1_READ

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
                    # Still update centroid share even when stopped so master
                    # can detect line presence before enabling lineGo
                    cen = self._line.get_centroid()
                    self._centroid_share.put(cen if cen is not None else 0.0)
                    self._line_detected_share.put(cen is not None)
                    yield self._state
                    continue

                cen = self._line.get_centroid()

                # Publish centroid and detection flag every tick for master
                self._centroid_share.put(cen if cen is not None else 0.0)
                self._line_detected_share.put(cen is not None)

                if cen is None:
                    cen = 0

                t = ticks_us()
                dt = ticks_diff(t, self._lastTime) / 1_000_000.0
                self._lastTime = t

                correction = self._loop.c_loop(cen, dt)
                self._steeringCorrection.put(correction)

                if not self._cen_data.full():
                    self._cen_data.put(cen)
                    self._cen_time.put(int(ticks_diff(t, self._startTime)/1000))

                if self._calibrateIRFlag.get():
                    self._calibrateIRFlag.put(False)
                    self._state = S2_CAL

            elif self._state == S2_CAL:
                if self._calStep == 0:
                    self._line.calibrate_white()
                    self._calStep = 1
                else:
                    self._line.calibrate_black()
                    self._calStep = 0
                self._state = S1_READ

            yield self._state