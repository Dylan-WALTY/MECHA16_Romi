from task_share import Share
import micropython
import math as _math

def _zeros(n):
    return [0.0] * n

def _dot(M, v):
    out = [0.0] * len(M)
    for i, row in enumerate(M):
        s = 0.0
        for j, a in enumerate(row):
            s += a * v[j]
        out[i] = s
    return out

S0_WAIT   = micropython.const(0)
S1_UPDATE = micropython.const(1)

# ── Discretized Observer Matrices ────────────────────────────────────────────
# Ts = 0.02 s  |  r=35mm, w=140mm, tau=0.11s, Km=3.49 rad/V/s
# State: [s, psi, Omega_L, Omega_R]
#   s       = average arc length (mm) — blended from encoders
#   psi     = heading (rad)           — NOTE: x_hat[1] is NOT published as psi_hat
#             psi_hat is always the raw IMU reading (see below)
#   Omega_L = left  wheel speed (mm/s)
#   Omega_R = right wheel speed (mm/s)
# Input u*: [u_L, u_R, s_L, s_R, psi, dpsi]
#   u_L/u_R  = motor voltages (V)
#   s_L/s_R  = per-tick encoder arc deltas (mm) — NOT cumulative totals
#   psi      = IMU heading (rad)
#   dpsi     = IMU yaw rate (rad/s)

# Matrices from MATLAB observer tuning
A_D = (
    ( 0.6999,  0.0000, 0.2684,  0.2684),
    ( 0.0000,  0.7228, -0.003,   0.003),
    (-0.0332,  3.6733, 0.7773,  0.0445),
    (-0.0332, -3.6733, 0.0445,  0.7773),
)

B_D = (
    ( 0.0933,  0.0933,  0.15,    0.15,   0.0000, 0.0000),
    (-0.0011,  0.0011, -0.0008,  0.0008, 0.1640, 0.0039),
    ( 0.5624,  0.0151,  0.0272,  0.0059,-2.1732,-0.1814),
    ( 0.0151,  0.5624,  0.0059,  0.0272, 2.1732, 0.1814),
)

C_D = (
    (1.0000,  -70.5000,  0.0000,  0.0000),
    (1.0000,   70.5000,  0.0000,  0.0000),
    (0.0000,   1.0000,   0.0000,  0.0000),
    (0.0000,   0.0000,  -0.2482,  0.2482),
)

D_D = (
    (0.0000,   0.0000,  0.0000,  0.0000, 0.0000, 0.0000),
    (0.0000,   0.0000,  0.0000,  0.0000, 0.0000, 0.0000),
    (0.0000,   0.0000,  0.0000,  0.0000, 0.0000, 0.0000),
    (0.0000,   0.0000,  0.0000,  0.0000, 0.0000, 0.0000),
)

TS = 0.02   # s — must match Task period=20 in main.py

# Minimum voltage threshold to consider motors as "running".
# Below this value on BOTH motors the estimator freezes its state so that
# IMU noise does not drift the estimates while the robot is stationary.
_V_THRESHOLD = 0.05  # V  (~0.7% of 7.4V battery)


class task_observer:
    """
    Discrete-time Luenberger state observer.
    """

    def __init__(self,
                 left_arc:    Share,
                 right_arc:   Share,
                 left_volts:  Share,
                 right_volts: Share,
                 psi_share:   Share,
                 dpsi_share:  Share,
                 imu_ready:   Share,
                 estimatorGo: Share,
                 s_hat:       Share,
                 psi_hat:     Share,
                 omL_hat:     Share,
                 omR_hat:     Share,
                 x_pos_hat:   Share,
                 y_pos_hat:   Share):

        self._left_arc    = left_arc
        self._right_arc   = right_arc
        self._left_volts  = left_volts
        self._right_volts = right_volts
        self._psi_share   = psi_share
        self._dpsi_share  = dpsi_share
        self._imu_ready   = imu_ready
        self._estimatorGo = estimatorGo

        self._s_hat     = s_hat
        self._psi_hat   = psi_hat      # published = raw IMU psi, not x_hat[1]
        self._omL_hat   = omL_hat
        self._omR_hat   = omR_hat
        self._x_pos_hat = x_pos_hat
        self._y_pos_hat = y_pos_hat

        self._x_hat = _zeros(4)   # [s, psi_obs, Omega_L, Omega_R]
        self._X     = 0.0
        self._Y     = 0.0
        # psi_prev and v_prev are now derived from IMU + observer wheel speeds
        self._psi_prev = 0.0
        self._v_prev   = 0.0

        self._was_running = False
        self._was_active  = False

        self._state = S0_WAIT
        print("Observer Task object instantiated")

    def _reset(self):
        self._x_hat    = _zeros(4)
        self._X        = 0.0
        self._Y        = 0.0
        self._psi_prev = 0.0
        self._v_prev   = 0.0

    def run(self):
        while True:

            # ── S0: wait for IMU ready ────────────────────────────────────────
            if self._state == S0_WAIT:
                if self._imu_ready.get():
                    psi_0 = self._psi_share.get()
                    # Seed observer internal heading to match IMU so the first
                    # few update ticks don't have a large heading error.
                    self._x_hat[1] = psi_0
                    self._psi_prev = psi_0
                    self._state = S1_UPDATE
                yield self._state

            # ── S1: observer update ───────────────────────────────────────────
            elif self._state == S1_UPDATE:

                # If estimator is globally disabled, reset and hold
                if not self._estimatorGo.get():
                    if self._was_running:
                        self._reset()
                        self._was_running = False
                    yield self._state
                    continue

                self._was_running = True

                # Read inputs
                u_L  = self._left_volts.get()
                u_R  = self._right_volts.get()
                # s_L / s_R are per-tick arc deltas in mm (Bug 2 fix)
                s_L  = self._left_arc.get()
                s_R  = self._right_arc.get()
                # Use raw IMU for both heading and yaw-rate inputs
                psi  = self._psi_share.get()    # rad
                dpsi = self._dpsi_share.get()   # rad/s

                # ── FREEZE GUARD ──────────────────────────────────────────────
                # When both motors are off, the B_D psi/dpsi columns amplify
                # IMU noise into wheel speed estimates and then into X/Y. Hold
                # the current state until the motors are actually commanded.
                motors_active = (abs(u_L) > _V_THRESHOLD or
                                 abs(u_R) > _V_THRESHOLD)

                if not motors_active:
                    if self._was_active:
                        # Transition: running → stopped. Zero wheel speeds so
                        # the frozen publish is clean.
                        self._x_hat[2] = 0.0
                        self._x_hat[3] = 0.0
                        self._v_prev   = 0.0
                        self._was_active = False

                    # Publish frozen state — use raw IMU for heading (Bug 1/3 fix)
                    self._s_hat.put(float(self._x_hat[0]))
                    self._psi_hat.put(psi)                   # ← IMU, not x_hat[1]
                    self._omL_hat.put(float(self._x_hat[2]))
                    self._omR_hat.put(float(self._x_hat[3]))
                    self._x_pos_hat.put(self._X)
                    self._y_pos_hat.put(self._Y)
                    yield self._state
                    continue
                # ── END FREEZE GUARD ──────────────────────────────────────────

                self._was_active = True

                # Observer update:  x̂_{k+1} = A_D · x̂_k + B_D · u*_k
                # s_L, s_R are already per-tick deltas (mm) — matches the
                # discretization assumption used when the matrices were built.
                u_star = (u_L, u_R, s_L, s_R, psi, dpsi)

                # Innovation (kept for debugging — how far the model is from sensors)
                Cx = _dot(C_D, self._x_hat)
                Du = _dot(D_D, u_star)
                self._y_hat = [Cx[i] + Du[i] for i in range(4)]
                self._innovate = [
                    s_L  - self._y_hat[0],
                    s_R  - self._y_hat[1],
                    psi  - self._y_hat[2],
                    dpsi - self._y_hat[3],
                ]

                Ax = _dot(A_D, self._x_hat)
                Bu = _dot(B_D, u_star)
                self._x_hat = [Ax[i] + Bu[i] for i in range(4)]

                # Integrate X, Y using IMU heading + observer wheel speeds
                self._integrate_position(psi)

                # Publish — heading is always raw IMU (Bug 1/3 fix)
                self._s_hat.put(float(self._x_hat[0]))
                self._psi_hat.put(psi)                       # ← IMU, not x_hat[1]
                self._omL_hat.put(float(self._x_hat[2]))
                self._omR_hat.put(float(self._x_hat[3]))
                self._x_pos_hat.put(self._X)
                self._y_pos_hat.put(self._Y)

            yield self._state

    def _integrate_position(self, psi_now):
        """
        Trapezoidal integration of global X, Y.

        dX/dt = v · cos(psi),   dY/dt = v · sin(psi)

         velocity units:
            Omega_L/R from the observer are in mm/s (the arc inputs s_L/s_R
            fed into B_D are in mm, Ts in seconds, so the output wheel speeds
            carry mm/s units). The old code multiplied by _R_WHEEL a second
            time, which double-scaled the velocity. Correct formula:
                v = 0.5 * (OmL + OmR)   [mm/s]

        heading:
            psi_now is passed in from the raw IMU reading, not x_hat[1].
            This ensures turns (including right turns / negative yaw) are
            correctly reflected in the X/Y integration.
        """
        OmL = float(self._x_hat[2])
        OmR = float(self._x_hat[3])

        # mm/s — no _R_WHEEL factor (Bug 5 fix)
        v_now = 0.5 * (OmL + OmR)

        psi_mid = 0.5 * (self._psi_prev + psi_now)
        v_avg   = 0.5 * (self._v_prev   + v_now)

        self._X += v_avg * _math.cos(psi_mid) * TS
        self._Y += v_avg * _math.sin(psi_mid) * TS

        self._psi_prev = psi_now
        self._v_prev   = v_now