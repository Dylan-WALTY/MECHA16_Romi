"""
task_master.py — encoder-distance-based course sequencer.

Sequences the Romi through 10 states covering the full competition course:

  S1  Line follow east   LINE_FOLLOW_MM      start → top curve
  S2  RIGHT pivot 90°    TURN_S2_MM          face west
  S3  Drive west         until bump / WALL_MAX_MM
  S4  LEFT  pivot 90°    TURN_S4_MM          face south
  S5  Line follow south  SOUTH_DRIVE_MM
  S6  RIGHT pivot 90°    TURN_S6_MM          face west
  S7  Line follow west   WEST_DRIVE_MM       fixed distance, no line-loss check
  S8  LEFT  pivot 180°   TURN_S8_MM          face east
  S9  Line follow east   SLALOM_DRIVE_MM     slalom section
  S10 Stop

All line-follow segments run for a fixed encoder distance. task_master never
calls enc.update() — that is task_motor's responsibility every 50 ms.

Pivot convention (robot frame):
  RIGHT pivot = left FWD, right BACK  (+spd, -spd)
  LEFT  pivot = left BACK, right FWD  (-spd, +spd)
"""

import micropython
from pyb import Pin
from task_share import Share, Queue

# ── Tuning constants ───────────────────────────────────────────────────────────

TURN_SPEED      =  90.0   # mm/s  all pivot turns
LINE_SPEED_S1   = 150.0   # mm/s  S1: line follow east (top curve)
LINE_SPEED_S5   = 150.0   # mm/s  S5: line follow south
LINE_SPEED_S7   = 150.0   # mm/s  S7: line follow west
LINE_SPEED_S9   = 150.0   # mm/s  S9: line follow east (slaloms)
DRIVE_SPEED_S3  = 150.0   # mm/s  S3: drive west to wall

TURN_S2_MM      = 112.0   # mm  S2: RIGHT 90°
TURN_S4_MM      = 127.0   # mm  S4: LEFT  90°
TURN_S6_MM      = 110.0   # mm  S6: RIGHT 90°
TURN_S8_MM      = 170.0   # mm  S8: LEFT  180°

WALL_MAX_MM     = 600.0   # mm  S3: fallback distance if bump never fires
LINE_FOLLOW_MM  = 1775.0  # mm  S1: east, top curve
SOUTH_DRIVE_MM  =  368.0  # mm  S5: south
WEST_DRIVE_MM   = 2000.0  # mm  S7: west — tune to reach CP4
SLALOM_DRIVE_MM =  750.0  # mm  S9: east, slaloms

# ── State constants ────────────────────────────────────────────────────────────

_MM_PER_COUNT = (2.0 * 3.14159 * 35.0) / 1440.0   # ≈ 0.1527 mm/count

S0_IDLE        = micropython.const(0)
S1_LINE_A      = micropython.const(1)
S2_TURN_RIGHT  = micropython.const(2)
S3_TO_WALL     = micropython.const(3)
S4_TURN_LEFT   = micropython.const(4)
S5_LINE_B      = micropython.const(5)
S6_TURN_RIGHT2 = micropython.const(6)
S7_LINE_C      = micropython.const(7)
S8_TURN_180    = micropython.const(8)
S9_LINE_D      = micropython.const(9)
S10_DONE       = micropython.const(10)


class task_master:
    """Top-level course supervisor finite-state machine.

    Coordinates all motor, line-sensor, and estimator shares to drive the
    robot through the competition course. A run is started by setting
    masterGo=True (via 'y' in the serial UI) and can be aborted at any
    time with masterGo=False (via 'x').

    All segment distances are measured by averaging the left and right
    encoder arc lengths from a snapshot taken at the start of each segment,
    making the distance estimate robust against steering-induced asymmetry.
    """

    def __init__(self,
                 lineGo, trajectoryGo, trajectoryDone, crash_detect,
                 x_pos_hat, y_pos_hat, masterGo, trajTask,
                 leftMotorGo, rightMotorGo, leftSetPoint, rightSetPoint,
                 psi_hat, line_detected, estimatorGo,
                 left_encoder, right_encoder, bump_pins=None):
        """Bind all inter-task shares and configure bump-pin inputs."""
        self._lineGo       = lineGo
        self._trajGo       = trajectoryGo
        self._crash        = crash_detect
        self._masterGo     = masterGo
        self._leftMotorGo  = leftMotorGo
        self._rightMotorGo = rightMotorGo
        self._leftSP       = leftSetPoint
        self._rightSP      = rightSetPoint
        self._estimatorGo  = estimatorGo
        self._lenc         = left_encoder
        self._renc         = right_encoder

        if bump_pins is not None:
            self._bump_pins = [Pin(p, mode=Pin.IN, pull=Pin.PULL_UP)
                               for p in bump_pins]
        else:
            self._bump_pins = []

        self._seg_start_L   = 0.0
        self._seg_start_R   = 0.0
        self._seg_target_mm = 0.0

        self._lf_target_mm = 0.0
        self._lf_start_L   = 0.0
        self._lf_start_R   = 0.0
        self._lf_armed     = False

        self._idle_stopped = False
        self._state        = S0_IDLE
        print("Master Task object instantiated")

    # ── Encoder helpers ────────────────────────────────────────────────────────

    def _pos_mm(self, enc):
        """Return current encoder position converted to mm."""
        return enc.get_position() * _MM_PER_COUNT

    def _snap_seg(self):
        """Snapshot both encoder positions as the start of a new segment."""
        self._seg_start_L = self._pos_mm(self._lenc)
        self._seg_start_R = self._pos_mm(self._renc)

    def _seg_dist(self):
        """Return average distance travelled by both wheels since last snap."""
        l = abs(self._pos_mm(self._lenc) - self._seg_start_L)
        r = abs(self._pos_mm(self._renc) - self._seg_start_R)
        return (l + r) / 2.0

    def _seg_done(self):
        """Return True when the segment distance target has been reached."""
        return self._seg_dist() >= self._seg_target_mm

    # ── Motor helpers ──────────────────────────────────────────────────────────

    def _motors_off(self):
        """Stop all motors and disable every autonomous-control share."""
        self._lineGo.put(False)
        self._trajGo.put(False)
        self._leftSP.put(0.0)
        self._rightSP.put(0.0)
        self._leftMotorGo.put(False)
        self._rightMotorGo.put(False)
        self._estimatorGo.put(False)

    def _motors_on(self):
        """Enable motors and the state estimator."""
        self._estimatorGo.put(True)
        self._trajGo.put(True)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)

    def _set_wheels(self, left, right):
        """Write left and right speed setpoints."""
        self._leftSP.put(float(left))
        self._rightSP.put(float(right))

    def _start_line_follow(self, target_mm, speed):
        """Arm line-follow mode and snapshot encoder baseline for distance tracking."""
        self._trajGo.put(False)
        self._lineGo.put(False)
        self._leftSP.put(speed)
        self._rightSP.put(speed)
        self._estimatorGo.put(True)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._lineGo.put(True)
        self._lf_target_mm = target_mm
        self._lf_start_L   = self._pos_mm(self._lenc)
        self._lf_start_R   = self._pos_mm(self._renc)
        self._lf_armed     = False

    def _lf_tick(self):
        """Return True when average wheel travel has reached lf_target_mm."""
        dist_L = abs(self._pos_mm(self._lenc) - self._lf_start_L)
        dist_R = abs(self._pos_mm(self._renc) - self._lf_start_R)
        return (dist_L + dist_R) / 2.0 >= self._lf_target_mm

    def _start_move(self, left_sp, right_sp, target_mm):
        """Start a fixed-distance straight or pivot move."""
        self._lineGo.put(False)
        self._motors_on()
        self._snap_seg()
        self._seg_target_mm = target_mm
        self._set_wheels(left_sp, right_sp)

    # ── Bump helpers ───────────────────────────────────────────────────────────

    def _bump_active(self):
        """Return True if the crash queue has data or any bump pin is low."""
        if not self._crash.empty():
            return True
        for p in self._bump_pins:
            if p.value() == 0:
                return True
        return False

    def _clear_bump(self):
        """Drain the crash queue before a new segment."""
        while not self._crash.empty():
            self._crash.get()

    def _abort(self):
        """Return True if masterGo has been cleared (user pressed 'x')."""
        return not self._masterGo.get()

    def _soft_idle_stop(self):
        """Disable line and trajectory signals without stopping the motors hard."""
        self._lineGo.put(False)
        self._trajGo.put(False)

    # ── State machine ──────────────────────────────────────────────────────────

    def run(self):
        while True:

            if self._state == S0_IDLE:
                if not self._idle_stopped:
                    self._soft_idle_stop()
                    self._idle_stopped = True
                if self._masterGo.get():
                    self._idle_stopped = False
                    self._clear_bump()
                    self._start_line_follow(LINE_FOLLOW_MM, LINE_SPEED_S1)
                    self._state = S1_LINE_A

            elif self._state == S1_LINE_A:
                if self._abort():
                    self._motors_off(); self._idle_stopped = False
                    self._state = S0_IDLE; yield self._state; continue
                if self._lf_tick():
                    self._start_move(TURN_SPEED, -TURN_SPEED, TURN_S2_MM)
                    self._state = S2_TURN_RIGHT

            elif self._state == S2_TURN_RIGHT:
                if self._abort():
                    self._motors_off(); self._idle_stopped = False
                    self._state = S0_IDLE; yield self._state; continue
                if self._seg_done():
                    self._clear_bump()
                    self._start_move(DRIVE_SPEED_S3, DRIVE_SPEED_S3, WALL_MAX_MM)
                    self._state = S3_TO_WALL

            elif self._state == S3_TO_WALL:
                if self._abort():
                    self._motors_off(); self._idle_stopped = False
                    self._state = S0_IDLE; yield self._state; continue
                if self._bump_active() or self._seg_done():
                    self._clear_bump()
                    self._start_move(-TURN_SPEED, TURN_SPEED, TURN_S4_MM)
                    self._state = S4_TURN_LEFT

            elif self._state == S4_TURN_LEFT:
                if self._abort():
                    self._motors_off(); self._idle_stopped = False
                    self._state = S0_IDLE; yield self._state; continue
                if self._seg_done():
                    self._start_line_follow(SOUTH_DRIVE_MM, LINE_SPEED_S5)
                    self._state = S5_LINE_B

            elif self._state == S5_LINE_B:
                if self._abort():
                    self._motors_off(); self._idle_stopped = False
                    self._state = S0_IDLE; yield self._state; continue
                if self._lf_tick():
                    self._start_move(TURN_SPEED, -TURN_SPEED, TURN_S6_MM)
                    self._state = S6_TURN_RIGHT2

            elif self._state == S6_TURN_RIGHT2:
                if self._abort():
                    self._motors_off(); self._idle_stopped = False
                    self._state = S0_IDLE; yield self._state; continue
                if self._seg_done():
                    self._start_line_follow(WEST_DRIVE_MM, LINE_SPEED_S7)
                    self._state = S7_LINE_C

            elif self._state == S7_LINE_C:
                if self._abort():
                    self._motors_off(); self._idle_stopped = False
                    self._state = S0_IDLE; yield self._state; continue
                if self._lf_tick():
                    self._start_move(-TURN_SPEED, TURN_SPEED, TURN_S8_MM)
                    self._state = S8_TURN_180

            elif self._state == S8_TURN_180:
                if self._abort():
                    self._motors_off(); self._idle_stopped = False
                    self._state = S0_IDLE; yield self._state; continue
                if self._seg_done():
                    self._start_line_follow(SLALOM_DRIVE_MM, LINE_SPEED_S9)
                    self._state = S9_LINE_D

            elif self._state == S9_LINE_D:
                if self._abort():
                    self._motors_off(); self._idle_stopped = False
                    self._state = S0_IDLE; yield self._state; continue
                if self._lf_tick():
                    self._state = S10_DONE

            elif self._state == S10_DONE:
                self._motors_off()
                self._idle_stopped = False
                self._masterGo.put(False)
                print("Course complete!")
                self._state = S0_IDLE

            yield self._state
