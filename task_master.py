import micropython
import math as _math
from utime      import ticks_us, ticks_diff
from task_share import Share, Queue
from task_trajectory import TRAJ_GARAGE

# ── State constants ───────────────────────────────────────────────────────────
S0_IDLE          = micropython.const(0)  # Waiting for masterGo
S1_LINE_FOLLOW   = micropython.const(1)  # Normal line following
S2_TRAJ_PREP     = micropython.const(2)  # Line lost early — arm garage trajectory
S3_TRAJ_GARAGE   = micropython.const(3)  # Executing garage trajectory
S4_BUMP_RECOVER  = micropython.const(4)  # Wall hit — closed-loop 90° left turn
S5_LINE_FIND     = micropython.const(5)  # Creep forward until line reacquired
S6_DONE          = micropython.const(6)  # Course complete

# ── Tuning constants ──────────────────────────────────────────────────────────
_TURN_TARGET_RAD  = _math.pi / 2.0   # 90 degrees left after wall bump
_TURN_SPEED       = 80.0             # mm/s at each wheel during IMU turn
_TURN_TOL_RAD     = 0.08             # ~4.5° — tighten if overshooting

_CREEP_SPEED      = 80.0             # mm/s forward crawl during line-find

_LINE_LOST_TICKS  = 5                # consecutive no-line ticks before acting

# ── Finish detection ──────────────────────────────────────────────────────────
# The course is ~3000mm total. The only two line-loss events are:
#   1. Garage entry  — happens ~400-500mm into the run
#   2. CP#5 arrival  — happens at the very end (~3000mm)
# Any line-loss beyond this threshold is treated as CP#5, not the garage.
_FINISH_THRESHOLD_MM = 2000.0


class task_master:
    """
    Master / supervisor task for the full course.

    Course sequence:
        CP#0 → CP#1 → CP#2 approach : line follow
        Line disappears (< 2000mm)   : TRAJ_GARAGE — right turn then straight to wall
        Wall bump                    : IMU closed-loop 90° left turn
        Line reacquire               : creep forward until line seen
        CP#2 → CP#3 → CP#4 → CP#5   : line follow — arcs cross at CP#3/4 junction,
                                       line sensor handles it naturally
        Line disappears (> 2000mm)   : CP#5 reached — stop, course complete

    BUG 4 FIX — estimatorGo lifecycle:
        _stop_all() now clears estimatorGo so the estimator resets cleanly
        on abort. _start_run() re-enables it. This prevents stale state
        from accumulating if the user aborts and restarts with 'y'.

    BUG 7 FIX — _dist_from_start():
        Uses true Euclidean distance sqrt(dx²+dy²) from the run-start
        position rather than |X| alone. The robot can start heading in any
        direction, so X alone does not reliably measure total travel distance.
    """

    def __init__(self,
                 lineGo,          # Share(bool)
                 trajectoryGo,    # Share(bool)
                 trajectoryDone,  # Share(bool)
                 crash_detect,    # Queue: bump events from task_bump
                 x_pos_hat,       # Share(float): estimator X mm
                 y_pos_hat,       # Share(float): estimator Y mm
                 masterGo,        # Share(bool): set True by task_user 'y'
                 trajTask,        # task_trajectory instance
                 leftMotorGo,     # Share(bool)
                 rightMotorGo,    # Share(bool)
                 leftSetPoint,    # Share(float): written directly during turn/creep
                 rightSetPoint,   # Share(float): written directly during turn/creep
                 psi_hat,         # Share(float): IMU heading rad
                 line_detected,   # Share(bool): True when task_line sees a line
                 estimatorGo):    # Share(bool): Bug 4 fix — master owns lifecycle

        self._lineGo       = lineGo
        self._trajGo       = trajectoryGo
        self._trajDone     = trajectoryDone
        self._crash        = crash_detect
        self._x_hat        = x_pos_hat
        self._y_hat        = y_pos_hat
        self._masterGo     = masterGo
        self._trajTask     = trajTask
        self._leftMotorGo  = leftMotorGo
        self._rightMotorGo = rightMotorGo
        self._leftSP       = leftSetPoint
        self._rightSP      = rightSetPoint
        self._psi_hat      = psi_hat
        self._line_det     = line_detected
        self._estimatorGo  = estimatorGo   # Bug 4 fix

        self._x_at_start      = 0.0
        self._y_at_start      = 0.0        # Bug 7 fix: need Y origin too
        self._psi_turn_target = 0.0
        self._line_lost_count = 0

        self._state = S0_IDLE
        print("Master Task object instantiated")

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _stop_all(self):
        self._lineGo.put(False)
        self._trajGo.put(False)
        self._leftMotorGo.put(False)
        self._rightMotorGo.put(False)
        # Bug 4 fix: reset estimator so it doesn't drift between runs
        self._estimatorGo.put(False)

    def _start_run(self):
        """Arm estimator, motors, and line following for a fresh run."""
        self._estimatorGo.put(True)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._trajGo.put(False)
        self._lineGo.put(True)

    def _start_line(self):
        self._trajGo.put(False)
        self._lineGo.put(True)

    def _clear_bump_queue(self):
        while not self._crash.empty():
            self._crash.get()

    def _dist_from_start(self):
        """
        Bug 7 fix: true Euclidean distance from run-start position.
        The robot can start pointing in any direction, so using X alone
        gives the wrong distance when the robot has turned.
        """
        dx = self._x_hat.get() - self._x_at_start
        dy = self._y_hat.get() - self._y_at_start
        return _math.sqrt(dx*dx + dy*dy)

    def _angle_diff(self, target, current):
        """Shortest signed angular distance from current to target (rad)."""
        diff = target - current
        while diff >  _math.pi: diff -= 2.0 * _math.pi
        while diff < -_math.pi: diff += 2.0 * _math.pi
        return diff

    # ── State machine ─────────────────────────────────────────────────────────
    def run(self):
        while True:

            # ── S0: idle — wait for 'y' ───────────────────────────────────────
            if self._state == S0_IDLE:
                self._stop_all()
                if self._masterGo.get():
                    self._x_at_start      = self._x_hat.get()
                    self._y_at_start      = self._y_hat.get()   # Bug 7 fix
                    self._line_lost_count = 0
                    self._clear_bump_queue()
                    self._start_run()                            # Bug 4 fix
                    self._state = S1_LINE_FOLLOW

            # ── S1: line following ────────────────────────────────────────────
            elif self._state == S1_LINE_FOLLOW:

                if not self._masterGo.get():
                    self._stop_all()
                    self._state = S0_IDLE
                    yield self._state
                    continue

                # Count consecutive ticks without line
                if self._line_det.get():
                    self._line_lost_count = 0
                else:
                    self._line_lost_count += 1

                if self._line_lost_count >= _LINE_LOST_TICKS:
                    self._line_lost_count = 0
                    # ── KEY DECISION: garage entry or CP#5? ───────────────────
                    if self._dist_from_start() > _FINISH_THRESHOLD_MM:
                        # Far enough into the course — this is CP#5, we're done
                        self._state = S6_DONE
                    else:
                        # Early in the course — this is the garage entry
                        self._state = S2_TRAJ_PREP

            # ── S2: prep garage trajectory ────────────────────────────────────
            elif self._state == S2_TRAJ_PREP:
                self._lineGo.put(False)
                self._trajTask.set_trajectory(TRAJ_GARAGE)
                self._trajDone.put(False)
                self._clear_bump_queue()
                self._trajGo.put(True)
                self._state = S3_TRAJ_GARAGE

            # ── S3: garage trajectory — wait for wall bump ────────────────────
            elif self._state == S3_TRAJ_GARAGE:

                if not self._masterGo.get():
                    self._stop_all()
                    self._state = S0_IDLE
                    yield self._state
                    continue

                if not self._crash.empty():
                    self._clear_bump_queue()
                    self._trajGo.put(False)
                    self._leftSP.put(0.0)
                    self._rightSP.put(0.0)
                    # Target = current heading + 90° left
                    self._psi_turn_target = self._psi_hat.get() + _TURN_TARGET_RAD
                    while self._psi_turn_target >  _math.pi:
                        self._psi_turn_target -= 2.0 * _math.pi
                    while self._psi_turn_target < -_math.pi:
                        self._psi_turn_target += 2.0 * _math.pi
                    self._state = S4_BUMP_RECOVER

                elif self._trajDone.get():
                    # Trajectory timed out without bump — still try the turn
                    self._state = S4_BUMP_RECOVER

            # ── S4: IMU closed-loop 90° left turn ────────────────────────────
            elif self._state == S4_BUMP_RECOVER:

                if not self._masterGo.get():
                    self._stop_all()
                    self._state = S0_IDLE
                    yield self._state
                    continue

                err = self._angle_diff(self._psi_turn_target, self._psi_hat.get())

                if abs(err) <= _TURN_TOL_RAD:
                    self._leftSP.put(0.0)
                    self._rightSP.put(0.0)
                    self._line_lost_count = 0
                    self._state = S5_LINE_FIND
                else:
                    speed = min(_TURN_SPEED, abs(err) * 200.0)
                    if err > 0:
                        # Turn left (CCW): left back, right forward
                        self._leftSP.put(-speed)
                        self._rightSP.put( speed)
                    else:
                        # Overshot — correct right (CW)
                        self._leftSP.put( speed)
                        self._rightSP.put(-speed)

            # ── S5: creep forward until line reacquired ───────────────────────
            elif self._state == S5_LINE_FIND:

                if not self._masterGo.get():
                    self._stop_all()
                    self._state = S0_IDLE
                    yield self._state
                    continue

                if self._line_det.get():
                    self._leftSP.put(0.0)
                    self._rightSP.put(0.0)
                    self._line_lost_count = 0
                    self._start_line()
                    self._state = S1_LINE_FOLLOW
                else:
                    self._leftSP.put(_CREEP_SPEED)
                    self._rightSP.put(_CREEP_SPEED)

            # ── S6: course complete ───────────────────────────────────────────
            elif self._state == S6_DONE:
                self._stop_all()
                self._masterGo.put(False)  # task_user S7_RUN sees this → "Course complete!"
                self._state = S0_IDLE

            yield self._state