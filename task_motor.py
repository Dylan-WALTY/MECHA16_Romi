from motor_driver        import Motor as motor_driver
from encoder_driver      import Encoder as encoder
from task_share          import Share, Queue
from closed_loop_driver  import closed_loop_control
from utime               import ticks_us, ticks_diff
import micropython

S0_INIT = micropython.const(0)
S1_WAIT = micropython.const(1)
S2_RUN  = micropython.const(2)

_V_BAT = 7.4   # nominal battery voltage used to convert effort to volts


def _effort_to_volts(effort: float) -> float:
    """Convert a [-100, 100] effort value to an equivalent voltage."""
    return (effort / 100.0) * _V_BAT


class task_motor:
    """Closed-loop velocity control task for a single motor.

    Implements a PID + feedforward controller that runs as a cooperative
    task. Velocity setpoint and PID gains are read from shared variables
    each tick so they can be updated at runtime without restarting.

    During line-following or trajectory-following the motor runs in
    *autonomous mode* and the step-test data buffer / auto-stop logic is
    disabled. Autonomous mode latches True the first tick that lineGo or
    trajectoryGo is seen True and remains True for the entire activation,
    so a transient False on either signal (e.g. during a pivot turn) does
    not accidentally re-enable the auto-stop.

    The first PID tick after activation is skipped so the controller
    always receives a clean, single-period dt rather than a potentially
    large stale interval from the previous run.

    The effective wheel setpoint (base speed ± steering correction) is
    clamped to ±_MAX_SP_MMPS before being passed to the PID to prevent
    runaway commands from stale trajectory values.
    """

    _MAX_SP_MMPS = 350.0

    def __init__(self,
                 mot: motor_driver, enc: encoder, loop: closed_loop_control,
                 goFlag: Share, dataValues: Queue, timeValues: Queue,
                 setPoint: Share, kp: Share, ki: Share, kd: Share, kff: Share,
                 gainsUpdated: Share, steeringCorrection, baseSpeed, lineGo,
                 correctionSign: int, arc_share: Share, volts_share: Share,
                 trajectoryGo: Share = None):
        """Bind hardware objects and inter-task shares; set initial state."""
        self._state: int = S0_INIT

        self._mot:  motor_driver        = mot
        self._enc:  encoder             = enc
        self._loop: closed_loop_control = loop

        self._goFlag:     Share = goFlag
        self._dataValues: Queue = dataValues
        self._timeValues: Queue = timeValues
        self._arc_share:  Share = arc_share
        self._volts_share: Share = volts_share
        self._prev_arc_mm: float = 0.0

        self._startTime: int = 0
        self._lastTime:  int = 0

        self._setPoint:     Share = setPoint
        self._kp:           Share = kp
        self._ki:           Share = ki
        self._kd:           Share = kd
        self._kff:          Share = kff
        self._gainsUpdated: Share = gainsUpdated

        self._steeringCorrection: Share = steeringCorrection
        self._correctionSign            = correctionSign
        self._lineGo:       Share = lineGo
        self._trajectoryGo: Share = trajectoryGo

        self._autonomous_mode: bool = False
        self._first_run_tick:  bool = True

        print("Motor Task object instantiated")

    def _current_arc_mm(self) -> float:
        """Return cumulative arc length travelled by this wheel in mm."""
        mm_per_count = self._enc.wheel_circumference / self._enc.counts_per_rev
        return -self._enc.position * mm_per_count

    def _publish_observer(self, effort: float, delta_mm: float = 0.0):
        """Write per-tick arc delta and equivalent voltage to the observer shares."""
        self._arc_share.put(delta_mm)
        self._volts_share.put(_effort_to_volts(effort))

    def run(self):
        while True:

            # S0: one-time initialisation — load gains and reset arc baseline
            if self._state == S0_INIT:
                self._loop.set_K_p(self._kp.get())
                self._loop.set_K_i(self._ki.get())
                self._loop.set_K_d(self._kd.get())
                self._loop.set_K_ff(self._kff.get())
                self._loop.set_set_point(self._setPoint.get())
                self._prev_arc_mm = self._current_arc_mm()
                self._publish_observer(0.0, 0.0)
                self._state = S1_WAIT

            # S1: idle — update encoder for observer; transition on goFlag
            elif self._state == S1_WAIT:
                self._enc.update()
                self._publish_observer(0.0, 0.0)

                if self._goFlag.get():
                    self._dataValues.clear()
                    self._timeValues.clear()
                    self._enc.zero()
                    self._prev_arc_mm     = 0.0
                    self._startTime       = ticks_us()
                    self._lastTime        = self._startTime
                    self._loop.reset()
                    self._autonomous_mode = False
                    self._first_run_tick  = True
                    self._state           = S2_RUN

            # S2: closed-loop running
            elif self._state == S2_RUN:

                if not self._goFlag.get():
                    self._mot.set_effort(0)
                    self._loop.reset()
                    self._publish_observer(0.0, 0.0)
                    self._autonomous_mode = False
                    self._first_run_tick  = True
                    self._state = S1_WAIT
                    yield self._state
                    continue

                self._enc.update()
                vel = self._enc.get_velocity()

                # Latch autonomous mode the first tick a run signal is seen
                traj_active = self._trajectoryGo.get() if self._trajectoryGo else False
                if self._lineGo.get() or traj_active:
                    self._autonomous_mode = True

                # Skip PID on the first tick to avoid a large initial dt
                if self._first_run_tick:
                    self._lastTime       = ticks_us()
                    self._first_run_tick = False
                    self._publish_observer(0.0, 0.0)
                    yield self._state
                    continue

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
                sp = base + self._correctionSign * correction
                sp = max(-self._MAX_SP_MMPS, min(self._MAX_SP_MMPS, sp))
                self._loop.set_set_point(sp)

                effort = self._loop.c_loop(vel, dt)
                self._mot.set_effort(effort)

                current_arc       = self._current_arc_mm()
                delta_mm          = current_arc - self._prev_arc_mm
                self._prev_arc_mm = current_arc
                self._publish_observer(effort, delta_mm)

                # Step-test data collection (disabled in autonomous mode)
                if not self._autonomous_mode:
                    if not self._dataValues.full():
                        self._dataValues.put(float(vel))
                        self._timeValues.put(int(ticks_diff(t, self._startTime) / 1000.0))

                    if self._dataValues.full():
                        self._mot.set_effort(0)
                        self._publish_observer(0.0, 0.0)
                        self._autonomous_mode = False
                        self._state = S1_WAIT
                        self._goFlag.put(False)

            yield self._state
