import micropython
import math as _math
from utime      import ticks_us, ticks_diff
from task_share import Share

# ── State constants ───────────────────────────────────────────────────────────
S0_IDLE     = micropython.const(0)
S1_GENERATE = micropython.const(1)
S2_FOLLOW   = micropython.const(2)
S3_DONE     = micropython.const(3)

# ── Physical constants ────────────────────────────────────────────────────────
_WHEEL_RADIUS_MM  = 35.0
_WHEEL_BASE_MM    = 140.0
_LOOKAHEAD_S      = 1.5
_UPDATE_PERIOD_S  = 0.5

# ── Trajectory library ────────────────────────────────────────────────────────
# All trajectories are in LOCAL coordinates — (0,0) is wherever the robot is
# when trajectoryGo goes True, facing the direction it was heading at that moment.
# X = forward in the robot's entry heading, Y = left of that heading.
# Units: mm and mm/s.
#
# GARAGE trajectory:
#   Robot enters facing south (down). Local X is therefore southward.
#   Phase 1 (t=0 to t=1.2s): right-hand 90deg turn.
#     In local coords a right turn sweeps +X (forward) and -Y (right).
#   Phase 2 (t=1.2s onward): straight ahead (now facing east in world,
#     but still +X in local frame after the turn is complete).
#     750mm to the wall at ~150mm/s gives ~5s drive time.
#     Bump will interrupt before the table ends — that is intentional.

TRAJ_GARAGE = (
    # t       x      y      vx      vy
    # ── Phase 1: right 90° turn (sweep right = negative Y) ───────────────────
    (0.0,     0.0,    0.0,  150.0,    0.0),
    (0.4,    55.0,  -35.0,  130.0,  -80.0),
    (0.8,   100.0,  -90.0,   80.0, -120.0),
    (1.2,   120.0, -155.0,    0.0, -150.0),  # pointing east (−Y direction)
    # ── Phase 2: straight east toward wall, ~750mm ────────────────────────────
    (2.0,   120.0, -305.0,    0.0, -150.0),
    (3.0,   120.0, -455.0,    0.0, -150.0),
    (4.0,   120.0, -605.0,    0.0, -150.0),
    (5.2,   120.0, -755.0,    0.0, -150.0),  # bump fires well before here
)

# ── Kept for testing / future use ─────────────────────────────────────────────
TRAJ_STRAIGHT = (
    (0.0,   0.0,   0.0, 200.0,  0.0),
    (0.5, 100.0,   0.0, 200.0,  0.0),
    (1.0, 200.0,   0.0, 200.0,  0.0),
    (1.5, 300.0,   0.0, 200.0,  0.0),
    (2.0, 400.0,   0.0, 200.0,  0.0),
)

TRAJ_CURVE_LEFT = (
    (0.0,   0.0,   0.0, 200.0,   0.0),
    (0.5,  95.0,  25.0, 190.0,  50.0),
    (1.0, 175.0,  95.0, 160.0,  90.0),
    (1.5, 230.0, 190.0, 110.0, 130.0),
    (2.0, 255.0, 300.0,  50.0, 160.0),
)

TRAJ_AVOID = (
    (0.0,   0.0,   0.0, 200.0,   0.0),
    (0.5,  90.0,  60.0, 170.0,  80.0),
    (1.0, 200.0,  80.0, 200.0,   0.0),
    (1.5, 310.0,  60.0, 170.0, -80.0),
    (2.0, 400.0,   0.0, 200.0,   0.0),
)


# ── Matrix helpers ────────────────────────────────────────────────────────────
def _inv4(M):
    A = [[float(M[r][c]) for c in range(4)] + [1.0 if r == c else 0.0 for c in range(4)]
         for r in range(4)]
    for col in range(4):
        max_row = max(range(col, 4), key=lambda r: abs(A[r][col]))
        A[col], A[max_row] = A[max_row], A[col]
        pivot = A[col][col]
        if abs(pivot) < 1e-12:
            return None
        inv_p = 1.0 / pivot
        for c in range(8):
            A[col][c] *= inv_p
        for r in range(4):
            if r != col:
                f = A[r][col]
                for c in range(8):
                    A[r][c] -= f * A[col][c]
    return [[A[r][c + 4] for c in range(4)] for r in range(4)]


def _matvec4(M, v):
    return [sum(M[r][c] * v[c] for c in range(4)) for r in range(4)]


def _build_spline_coeffs(q0, dq0, q1, dq1, t0, t1):
    t0sq = t0*t0; t0cu = t0sq*t0
    t1sq = t1*t1; t1cu = t1sq*t1
    M = (
        (1.0, t0,  t0sq,       t0cu      ),
        (0.0, 1.0, 2.0*t0,     3.0*t0sq  ),
        (1.0, t1,  t1sq,       t1cu      ),
        (0.0, 1.0, 2.0*t1,     3.0*t1sq  ),
    )
    Minv = _inv4(M)
    if Minv is None:
        return None
    return _matvec4(Minv, [q0, dq0, q1, dq1])


def _eval_spline_vel(coeffs, t):
    _, b, c, d = coeffs
    return b + 2.0*c*t + 3.0*d*t*t


def _interp_trajectory(traj, t):
    if t <= traj[0][0]:
        return traj[0][1], traj[0][2], traj[0][3], traj[0][4]
    if t >= traj[-1][0]:
        return traj[-1][1], traj[-1][2], traj[-1][3], traj[-1][4]
    for i in range(len(traj) - 1):
        t0, x0, y0, vx0, vy0 = traj[i]
        t1, x1, y1, vx1, vy1 = traj[i + 1]
        if t0 <= t <= t1:
            a = (t - t0) / (t1 - t0)
            return (x0 + a*(x1-x0), y0 + a*(y1-y0),
                    vx0 + a*(vx1-vx0), vy0 + a*(vy1-vy0))
    return traj[-1][1], traj[-1][2], traj[-1][3], traj[-1][4]


def _velocity_to_wheels(vx, vy, psi):
    v     =  vx*_math.cos(psi) + vy*_math.sin(psi)
    v_lat = -vx*_math.sin(psi) + vy*_math.cos(psi)
    omega = v_lat / (_WHEEL_BASE_MM * 0.5)
    half  = _WHEEL_BASE_MM * 0.5
    vL = v - omega * half
    vR = v + omega * half
    return vL, vR


# ── Main task class ───────────────────────────────────────────────────────────
class task_trajectory:
    """
    Trajectory following task.

    All trajectory tables are in LOCAL coordinates relative to the
    robot's pose when trajectoryGo goes True. At S1_GENERATE we snapshot the
    world pose (x_origin, y_origin, psi_origin) and rotate all estimator
    readings into that local frame before doing the spline math.
    This means TRAJ_GARAGE always starts at (0,0) regardless of where on the
    track the robot is when it enters the garage.
    """

    def __init__(self,
                 x_pos_hat,
                 y_pos_hat,
                 psi_hat,
                 omL_hat,
                 omR_hat,
                 leftSetPoint,
                 rightSetPoint,
                 trajectoryGo,
                 trajectoryDone,
                 trajectory=None):

        self._x_hat   = x_pos_hat
        self._y_hat   = y_pos_hat
        self._psi_hat = psi_hat
        self._omL_hat = omL_hat
        self._omR_hat = omR_hat
        self._leftSP  = leftSetPoint
        self._rightSP = rightSetPoint
        self._go      = trajectoryGo
        self._done    = trajectoryDone
        self._traj    = trajectory if trajectory is not None else TRAJ_STRAIGHT

        self._cx = None
        self._cy = None
        self._t_start       = 0
        self._t_last_update = 0
        self._t_now_s       = 0.0

        # Local frame origin — set at S1_GENERATE
        self._x_origin   = 0.0
        self._y_origin   = 0.0
        self._psi_origin = 0.0   # heading when trajectory started (rad)

        self._state = S0_IDLE
        print("Trajectory Task object instantiated")

    def set_trajectory(self, new_traj):
        self._traj = new_traj

    def _elapsed_s(self):
        return ticks_diff(ticks_us(), self._t_start) / 1_000_000.0

    def _world_to_local(self, xw, yw):
        """Rotate world-frame position into local trajectory frame."""
        dx = xw - self._x_origin
        dy = yw - self._y_origin
        cp = _math.cos(-self._psi_origin)
        sp = _math.sin(-self._psi_origin)
        return dx*cp - dy*sp, dx*sp + dy*cp

    def _local_vel_to_world(self, vxl, vyl):
        """Rotate local-frame velocity back to world frame."""
        cp = _math.cos(self._psi_origin)
        sp = _math.sin(self._psi_origin)
        return vxl*cp - vyl*sp, vxl*sp + vyl*cp

    def _recompute_spline(self, t_now_s):
        # Current world pose
        xw  = self._x_hat.get()
        yw  = self._y_hat.get()
        psi = self._psi_hat.get()

        # Convert to local frame
        x0, y0 = self._world_to_local(xw, yw)

        # Current velocity in local frame
        omL = self._omL_hat.get()
        omR = self._omR_hat.get()
        # omL/omR from the observer are in mm/s — no _WHEEL_RADIUS_MM factor.
        # (The observer's arc inputs are in mm so its wheel-speed outputs
        #  carry mm/s units directly. Multiplying by _WHEEL_RADIUS_MM again
        #  would inflate the initial velocity by 35x in the spline.)
        v   = 0.5 * (omL + omR)
        vx0_w = v * _math.cos(psi)
        vy0_w = v * _math.sin(psi)
        vx0, vy0 = self._world_to_local(xw + vx0_w, yw + vy0_w)
        vx0 -= x0; vy0 -= y0   # just the velocity vector rotated

        # Lookahead in local frame (trajectory table is already local)
        t_look = t_now_s + _LOOKAHEAD_S
        x1, y1, vx1, vy1 = _interp_trajectory(self._traj, t_look)

        cx = _build_spline_coeffs(x0, vx0, x1, vx1, t_now_s, t_look)
        cy = _build_spline_coeffs(y0, vy0, y1, vy1, t_now_s, t_look)
        if cx is not None and cy is not None:
            self._cx = cx
            self._cy = cy

    def _command_from_spline(self, t_now_s):
        if self._cx is None or self._cy is None:
            return
        # Velocity in local frame
        vxl = _eval_spline_vel(self._cx, t_now_s)
        vyl = _eval_spline_vel(self._cy, t_now_s)
        # Rotate to world frame for kinematics
        vxw, vyw = self._local_vel_to_world(vxl, vyl)
        psi = self._psi_hat.get()
        vL, vR = _velocity_to_wheels(vxw, vyw, psi)
        max_speed = 400.0
        vL = max(-max_speed, min(max_speed, vL))
        vR = max(-max_speed, min(max_speed, vR))
        self._leftSP.put(vL)
        self._rightSP.put(vR)

    def _trajectory_ended(self, t_now_s):
        return t_now_s >= self._traj[-1][0]

    def run(self):
        while True:

            if self._state == S0_IDLE:
                if self._go.get():
                    self._t_start       = ticks_us()
                    self._t_last_update = ticks_us()
                    self._cx            = None
                    self._cy            = None
                    self._done.put(False)
                    self._state = S1_GENERATE

            elif self._state == S1_GENERATE:
                # Capture local frame origin once at start of segment
                self._x_origin   = self._x_hat.get()
                self._y_origin   = self._y_hat.get()
                self._psi_origin = self._psi_hat.get()
                self._t_now_s    = self._elapsed_s()
                self._recompute_spline(self._t_now_s)
                self._t_last_update = ticks_us()
                self._state = S2_FOLLOW

            elif self._state == S2_FOLLOW:
                if not self._go.get():
                    self._leftSP.put(0.0)
                    self._rightSP.put(0.0)
                    self._state = S0_IDLE
                    yield self._state
                    continue

                self._t_now_s = self._elapsed_s()

                since_update = ticks_diff(ticks_us(), self._t_last_update) / 1_000_000.0
                if since_update >= _UPDATE_PERIOD_S:
                    self._recompute_spline(self._t_now_s)
                    self._t_last_update = ticks_us()

                self._command_from_spline(self._t_now_s)

                if self._trajectory_ended(self._t_now_s):
                    self._state = S3_DONE

            elif self._state == S3_DONE:
                self._leftSP.put(0.0)
                self._rightSP.put(0.0)
                self._go.put(False)
                self._done.put(True)
                self._state = S0_IDLE

            yield self._state