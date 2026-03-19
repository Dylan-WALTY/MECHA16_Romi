"""
task_trajectory.py — cubic spline trajectory generator and follower.

Generates smooth velocity profiles between waypoints defined in a local
coordinate frame. At each update interval a cubic Hermite spline is fit
between the robot's current estimated state and a lookahead point in the
trajectory table. The spline velocity is differentiated and converted to
differential wheel speed setpoints.

This module is available but was not used in the final competition run.
The master task sequences the course entirely by encoder distance.
"""

import micropython
import math as _math
from utime      import ticks_us, ticks_diff
from task_share import Share

# ── State constants ────────────────────────────────────────────────────────────

S0_IDLE     = micropython.const(0)
S1_GENERATE = micropython.const(1)
S2_FOLLOW   = micropython.const(2)
S3_DONE     = micropython.const(3)

# ── Physical constants ─────────────────────────────────────────────────────────

_WHEEL_RADIUS_MM = 35.0
_WHEEL_BASE_MM   = 140.0
_LOOKAHEAD_S     = 0.8   # lookahead window in seconds
_UPDATE_PERIOD_S = 0.3   # spline recompute interval in seconds

# ── Trajectory tables ──────────────────────────────────────────────────────────
# All waypoints are in a local frame: origin at the robot's pose when
# trajectoryGo goes True, X forward, Y left. Columns: (t, x, y, vx, vy).

TRAJ_GARAGE = (
    # Straight drive into the garage at 150 mm/s. The bump sensor fires
    # before the trajectory ends; the table extends far enough as a fallback.
    (0.0,    0.0,   0.0,  150.0,  0.0),
    (1.0,  150.0,   0.0,  150.0,  0.0),
    (2.0,  300.0,   0.0,  150.0,  0.0),
    (3.0,  450.0,   0.0,  150.0,  0.0),
    (4.0,  600.0,   0.0,  150.0,  0.0),
    (5.5,  825.0,   0.0,  150.0,  0.0),
)

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

# ── Matrix helpers ─────────────────────────────────────────────────────────────

def _inv4(M):
    """Return the inverse of a 4×4 matrix using Gauss-Jordan elimination.

    Returns None if the matrix is singular.
    """
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
    """Multiply a 4×4 matrix by a 4-element vector."""
    return [sum(M[r][c] * v[c] for c in range(4)) for r in range(4)]


def _build_spline_coeffs(q0, dq0, q1, dq1, t0, t1):
    """Fit a cubic polynomial matching position and velocity at t0 and t1.

    Returns the coefficient vector [a, b, c, d] for q(t) = a + bt + ct² + dt³,
    or None if the time span is degenerate.
    """
    t0sq = t0*t0; t0cu = t0sq*t0
    t1sq = t1*t1; t1cu = t1sq*t1
    M = (
        (1.0, t0,  t0sq,      t0cu     ),
        (0.0, 1.0, 2.0*t0,    3.0*t0sq ),
        (1.0, t1,  t1sq,      t1cu     ),
        (0.0, 1.0, 2.0*t1,    3.0*t1sq ),
    )
    Minv = _inv4(M)
    if Minv is None:
        return None
    return _matvec4(Minv, [q0, dq0, q1, dq1])


def _eval_spline_vel(coeffs, t):
    """Evaluate the first derivative of a cubic spline at time t."""
    _, b, c, d = coeffs
    return b + 2.0*c*t + 3.0*d*t*t


def _interp_trajectory(traj, t):
    """Linearly interpolate position and velocity from the waypoint table at time t."""
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
    """Convert world-frame velocity to left/right wheel speeds.

    Decomposes the velocity into forward and lateral components, converts
    lateral velocity to a yaw rate, then applies differential kinematics.
    """
    v     =  vx*_math.cos(psi) + vy*_math.sin(psi)
    v_lat = -vx*_math.sin(psi) + vy*_math.cos(psi)
    omega = v_lat / (_WHEEL_BASE_MM * 0.5)
    half  = _WHEEL_BASE_MM * 0.5
    return v - omega * half, v + omega * half


# ── Task class ─────────────────────────────────────────────────────────────────

class task_trajectory:
    """Cubic spline trajectory follower task.

    Operates in a local coordinate frame whose origin is snapped to the
    robot's estimated pose at the moment trajectoryGo goes True. This makes
    every trajectory table reusable regardless of where on the course the
    manoeuvre begins.

    At each _UPDATE_PERIOD_S interval a new cubic spline is computed between
    the current estimated state and the lookahead point _LOOKAHEAD_S ahead
    in the table. The spline is differentiated to obtain a velocity command,
    which is converted to differential wheel setpoints via inverse kinematics.

    States
    ------
    S0_IDLE     : Wait for trajectoryGo.
    S1_GENERATE : Capture local frame origin and compute the first spline.
    S2_FOLLOW   : Execute the trajectory; recompute spline periodically.
    S3_DONE     : Zero setpoints, signal completion, return to idle.
    """

    def __init__(self,
                 x_pos_hat, y_pos_hat, psi_hat, omL_hat, omR_hat,
                 leftSetPoint, rightSetPoint,
                 trajectoryGo, trajectoryDone,
                 trajectory=None):
        """Bind observer and setpoint shares; set the initial trajectory table."""
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

        self._x_origin   = 0.0
        self._y_origin   = 0.0
        self._psi_origin = 0.0

        self._state = S0_IDLE
        print("Trajectory Task object instantiated")

    def set_trajectory(self, new_traj):
        """Swap the active waypoint table; takes effect at the next S1_GENERATE."""
        self._traj = new_traj

    def _elapsed_s(self):
        """Return seconds elapsed since the trajectory started."""
        return ticks_diff(ticks_us(), self._t_start) / 1_000_000.0

    def _world_to_local(self, xw, yw):
        """Rotate a world-frame position into the trajectory's local frame."""
        dx = xw - self._x_origin
        dy = yw - self._y_origin
        cp = _math.cos(-self._psi_origin)
        sp = _math.sin(-self._psi_origin)
        return dx*cp - dy*sp, dx*sp + dy*cp

    def _local_vel_to_world(self, vxl, vyl):
        """Rotate a local-frame velocity vector back to the world frame."""
        cp = _math.cos(self._psi_origin)
        sp = _math.sin(self._psi_origin)
        return vxl*cp - vyl*sp, vxl*sp + vyl*cp

    def _recompute_spline(self, t_now_s):
        """Fit new splines from the current estimated state to the lookahead point."""
        xw  = self._x_hat.get()
        yw  = self._y_hat.get()
        psi = self._psi_hat.get()

        x0, y0 = self._world_to_local(xw, yw)

        omL = self._omL_hat.get()
        omR = self._omR_hat.get()
        v   = 0.5 * (omL + omR)
        vx0_w = v * _math.cos(psi)
        vy0_w = v * _math.sin(psi)
        vx0, vy0 = self._world_to_local(xw + vx0_w, yw + vy0_w)
        vx0 -= x0; vy0 -= y0

        t_look = t_now_s + _LOOKAHEAD_S
        x1, y1, vx1, vy1 = _interp_trajectory(self._traj, t_look)

        cx = _build_spline_coeffs(x0, vx0, x1, vx1, t_now_s, t_look)
        cy = _build_spline_coeffs(y0, vy0, y1, vy1, t_now_s, t_look)
        if cx is not None and cy is not None:
            self._cx = cx
            self._cy = cy

    def _command_from_spline(self, t_now_s):
        """Evaluate the current spline and write wheel speed setpoints."""
        if self._cx is None or self._cy is None:
            return
        vxl = _eval_spline_vel(self._cx, t_now_s)
        vyl = _eval_spline_vel(self._cy, t_now_s)
        vxw, vyw = self._local_vel_to_world(vxl, vyl)
        psi = self._psi_hat.get()
        vL, vR = _velocity_to_wheels(vxw, vyw, psi)
        max_speed = 400.0
        vL = max(-max_speed, min(max_speed, vL))
        vR = max(-max_speed, min(max_speed, vR))
        self._leftSP.put(vL)
        self._rightSP.put(vR)

    def _trajectory_ended(self, t_now_s):
        """Return True when elapsed time has passed the last waypoint."""
        return t_now_s >= self._traj[-1][0]

    def run(self):
        while True:

            # S0: idle — wait for the go signal
            if self._state == S0_IDLE:
                if self._go.get():
                    self._t_start       = ticks_us()
                    self._t_last_update = ticks_us()
                    self._cx            = None
                    self._cy            = None
                    self._done.put(False)
                    self._state = S1_GENERATE

            # S1: capture local frame origin and compute the first spline
            elif self._state == S1_GENERATE:
                self._x_origin   = self._x_hat.get()
                self._y_origin   = self._y_hat.get()
                self._psi_origin = self._psi_hat.get()
                self._t_now_s    = self._elapsed_s()
                self._recompute_spline(self._t_now_s)
                self._t_last_update = ticks_us()
                self._state = S2_FOLLOW

            # S2: follow trajectory, recomputing the spline every _UPDATE_PERIOD_S
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

            # S3: zero setpoints and signal completion
            elif self._state == S3_DONE:
                self._leftSP.put(0.0)
                self._rightSP.put(0.0)
                self._go.put(False)
                self._done.put(True)
                self._state = S0_IDLE

            yield self._state
