from motor_driver        import Motor as motor_driver
from encoder_driver      import Encoder as encoder
from task_share          import Share, Queue
from closed_loop_driver  import closed_loop_control
from utime               import ticks_us, ticks_diff
import micropython

S0_INIT = micropython.const(0)
S1_WAIT = micropython.const(1)
S2_RUN  = micropython.const(2)

_V_BAT = 7.4

def _effort_to_volts(effort: float) -> float:
    return (effort / 100.0) * _V_BAT


class task_motor:
    '''
    Motor closed-loop control task.

    BUG 2 FIX — arc_share now publishes per-tick arc DELTA (mm moved this
    step), not the cumulative total since zero(). The observer's B_D matrix
    was designed around per-step arc increments consistent with Ts = 0.02 s.
    Publishing a cumulative total caused a massive jump on the first observer
    tick after the motors started (because the encoder had been accumulating
    counts since boot) and continuously inflated the s_L/s_R input values,
    destabilising the wheel-speed and heading rows of the observer.

    The fix:
      - _prev_arc_mm is reset to the current cumulative arc whenever the
        motor task enters S2_RUN (just after enc.zero() and loop.reset()).
      - _publish_observer computes delta = current_arc - _prev_arc_mm and
        publishes that, then saves current_arc as the new _prev_arc_mm.
      - In S1_WAIT, delta is always published as 0.0 so the observer sees
        no spurious motion while the motors are stopped.

    Bug 4 fix: added trajectoryGo share so the data-buffer auto-stop logic
    is suppressed during trajectory following.
    '''

    def __init__(self,
                 mot: motor_driver, enc: encoder, loop: closed_loop_control,
                 goFlag: Share, dataValues: Queue, timeValues: Queue,
                 setPoint: Share, kp: Share, ki: Share, kd: Share, kff: Share,
                 gainsUpdated: Share, steeringCorrection, baseSpeed, lineGo,
                 correctionSign: int, arc_share: Share, volts_share: Share,
                 trajectoryGo: Share = None):

        self._state: int        = S0_INIT

        self._mot: motor_driver = mot
        self._enc: encoder      = enc
        self._loop: closed_loop_control = loop

        self._goFlag: Share     = goFlag
        self._dataValues: Queue = dataValues
        self._timeValues: Queue = timeValues

        self._arc_share: Share   = arc_share
        self._volts_share: Share = volts_share

        # Bug 2 fix: track previous cumulative arc to compute per-tick deltas
        self._prev_arc_mm: float = 0.0

        self._startTime: int    = 0
        self._lastTime: int     = 0

        self._setPoint: Share     = setPoint
        self._kp: Share           = kp
        self._ki: Share           = ki
        self._kd: Share           = kd
        self._kff: Share          = kff
        self._gainsUpdated: Share = gainsUpdated

        self._steeringCorrection: Share = steeringCorrection
        self._correctionSign            = correctionSign
        self._lineGo: Share             = lineGo
        self._trajectoryGo: Share       = trajectoryGo

        print("Motor Task object instantiated")

    def _current_arc_mm(self) -> float:
        """Return the current cumulative arc in mm (same sign convention as before)."""
        mm_per_count = self._enc.wheel_circumference / self._enc.counts_per_rev
        return -self._enc.position * mm_per_count

    def _publish_observer(self, effort: float, delta_mm: float = 0.0):
        """
        Publish the per-tick arc delta and motor voltage to the observer.

        delta_mm must be pre-computed by the caller; it is 0.0 in S1_WAIT
        so the observer sees no spurious motion while motors are stopped.
        """
        self._arc_share.put(delta_mm)
        self._volts_share.put(_effort_to_volts(effort))

    def run(self):
        while True:

            if self._state == S0_INIT:
                self._loop.set_K_p(self._kp.get())
                self._loop.set_K_i(self._ki.get())
                self._loop.set_K_d(self._kd.get())
                self._loop.set_K_ff(self._kff.get())
                self._loop.set_set_point(self._setPoint.get())
                # Initialise prev_arc so the first delta is 0, not a large number
                self._prev_arc_mm = self._current_arc_mm()
                self._publish_observer(0.0, 0.0)
                self._state = S1_WAIT

            elif self._state == S1_WAIT:
                self._enc.update()
                # While waiting, publish zero delta so the observer is frozen
                self._publish_observer(0.0, 0.0)

                if self._goFlag.get():
                    self._dataValues.clear()
                    self._timeValues.clear()
                    self._enc.zero()
                    # After zero(), cumulative position resets to 0.
                    # Reset prev_arc_mm to 0 so the first delta is also 0.
                    self._prev_arc_mm = 0.0
                    self._startTime = ticks_us()
                    self._lastTime  = self._startTime
                    self._loop.reset()
                    self._state = S2_RUN

            elif self._state == S2_RUN:

                if not self._goFlag.get():
                    self._mot.set_effort(0)
                    self._loop.reset()
                    self._publish_observer(0.0, 0.0)
                    self._state = S1_WAIT
                    yield self._state
                    continue

                self._enc.update()
                vel = self._enc.get_velocity()

                t  = ticks_us()
                dt = ticks_diff(t, self._lastTime) / 1_000_000.0
                self._lastTime = t

                if self._gainsUpdated.get():
                    self._loop.set_K_p(self._kp.get())
                    self._loop.set_K_i(self._ki.get())
                    self._loop.set_K_d(self._kd.get())
                    self._loop.set_K_ff(self._kff.get())
                    self._gainsUpdated.put(False)

                base       = self._setPoint.get()
                correction = self._steeringCorrection.get()
                self._loop.set_set_point(base + self._correctionSign * correction)

                effort = self._loop.c_loop(vel, dt)
                self._mot.set_effort(effort)

                # Compute per-tick arc delta (Bug 2 fix)
                current_arc = self._current_arc_mm()
                delta_mm = current_arc - self._prev_arc_mm
                self._prev_arc_mm = current_arc

                self._publish_observer(effort, delta_mm)

                # Data-buffer auto-stop:
                # Only active during a plain step-response test
                # (lineGo=False AND trajectoryGo=False).
                traj_active = self._trajectoryGo.get() if self._trajectoryGo else False

                if not self._lineGo.get() and not traj_active:
                    if not self._dataValues.full():
                        self._dataValues.put(float(vel))
                        self._timeValues.put(int(ticks_diff(t, self._startTime) / 1000.0))

                    if self._dataValues.full():
                        self._mot.set_effort(0)
                        self._publish_observer(0.0, 0.0)
                        self._state = S1_WAIT
                        self._goFlag.put(False)

            yield self._state