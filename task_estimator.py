from task_share import Share
import micropython
import math as _math


def _zeros(n):
    """Return a list of n zero floats."""
    return [0.0] * n


def _dot(M, v):
    """Compute the matrix-vector product M·v."""
    out = [0.0] * len(M)
    for i, row in enumerate(M):
        s = 0.0
        for j, a in enumerate(row):
            s += a * v[j]
        out[i] = s
    return out


S0_WAIT   = micropython.const(0)
S1_UPDATE = micropython.const(1)

# ── Discrete-time observer matrices (Ts = 20 ms) ──────────────────────────────
# Designed in MATLAB for: r = 35 mm, wheelbase = 140 mm,
# motor time constant τ = 0.11 s, Km = 3.49 rad/V/s.
# State vector: x̂ = [s, ψ, Ω_L, Ω_R]
# Input vector: u* = [u_L, u_R, s_L, s_R, ψ_IMU, dψ_IMU]

A_D = (
    ( 0.6999,  0.0000,  0.2684,  0.2684),
    ( 0.0000,  0.7228, -0.003,   0.003 ),
    (-0.0332,  3.6733,  0.7773,  0.0445),
    (-0.0332, -3.6733,  0.0445,  0.7773),
)

B_D = (
    ( 0.0933,  0.0933,  0.15,    0.15,   0.0000,  0.0000),
    (-0.0011,  0.0011, -0.0008,  0.0008, 0.1640,  0.0039),
    ( 0.5624,  0.0151,  0.0272,  0.0059, -2.1732, -0.1814),
    ( 0.0151,  0.5624,  0.0059,  0.0272,  2.1732,  0.1814),
)

C_D = (
    (1.0000, -70.5000, 0.0000,  0.0000),
    (1.0000,  70.5000, 0.0000,  0.0000),
    (0.0000,   1.0000, 0.0000,  0.0000),
    (0.0000,   0.0000, -0.2482, 0.2482),
)

D_D = (
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
)

TS = 0.02           # observer sample period in seconds — must match task period
_V_THRESHOLD = 0.05 # minimum motor voltage below which the update is frozen


class task_observer:
    """Discrete-time Luenberger state observer for robot pose estimation.

    Fuses per-tick encoder arc deltas, motor voltages, IMU heading, and
    IMU yaw rate to estimate the state vector x̂ = [s, ψ, Ω_L, Ω_R] using
    the update law x̂_{k+1} = A_D·x̂_k + B_D·u*_k.

    Global X/Y position is integrated from the observer wheel-speed
    estimates using the trapezoidal rule and the raw IMU heading.

    When both motor voltages are below _V_THRESHOLD the update is frozen
    to prevent B_D IMU-column amplification of gyro noise accumulating
    into the position estimate while the robot is stationary.

    The published heading (psi_hat) is always the raw IMU reading, not
    the observer's internal ψ state, because the A_D[1][1] = 0.7228 gain
    causes the internal heading to decay toward zero rather than track
    absolute orientation.
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
        """Bind all input and output shares and initialise observer state."""
        self._left_arc    = left_arc
        self._right_arc   = right_arc
        self._left_volts  = left_volts
        self._right_volts = right_volts
        self._psi_share   = psi_share
        self._dpsi_share  = dpsi_share
        self._imu_ready   = imu_ready
        self._estimatorGo = estimatorGo

        self._s_hat     = s_hat
        self._psi_hat   = psi_hat
        self._omL_hat   = omL_hat
        self._omR_hat   = omR_hat
        self._x_pos_hat = x_pos_hat
        self._y_pos_hat = y_pos_hat

        self._x_hat = _zeros(4)
        self._X     = 0.0
        self._Y     = 0.0

        self._psi_prev   = 0.0
        self._v_prev     = 0.0
        self._was_running = False
        self._was_active  = False

        self._state = S0_WAIT
        print("Observer Task object instantiated")

    def _reset(self):
        """Zero the state vector and position integrator."""
        self._x_hat    = _zeros(4)
        self._X        = 0.0
        self._Y        = 0.0
        self._psi_prev = 0.0
        self._v_prev   = 0.0

    def run(self):
        while True:

            # S0: wait for IMU to be ready, then seed the heading state
            if self._state == S0_WAIT:
                if self._imu_ready.get():
                    psi_0          = self._psi_share.get()
                    self._x_hat[1] = psi_0
                    self._psi_prev = psi_0
                    self._state    = S1_UPDATE
                yield self._state

            # S1: run observer update every tick
            elif self._state == S1_UPDATE:

                if not self._estimatorGo.get():
                    if self._was_running:
                        self._reset()
                        self._was_running = False
                    yield self._state
                    continue

                self._was_running = True

                u_L  = self._left_volts.get()
                u_R  = self._right_volts.get()
                s_L  = self._left_arc.get()
                s_R  = self._right_arc.get()
                psi  = self._psi_share.get()
                dpsi = self._dpsi_share.get()

                motors_active = (abs(u_L) > _V_THRESHOLD or
                                 abs(u_R) > _V_THRESHOLD)

                # Freeze update when motors are off to prevent gyro-noise drift
                if not motors_active:
                    if self._was_active:
                        self._x_hat[2] = 0.0
                        self._x_hat[3] = 0.0
                        self._v_prev   = 0.0
                        self._was_active = False
                    self._s_hat.put(float(self._x_hat[0]))
                    self._psi_hat.put(psi)
                    self._omL_hat.put(float(self._x_hat[2]))
                    self._omR_hat.put(float(self._x_hat[3]))
                    self._x_pos_hat.put(self._X)
                    self._y_pos_hat.put(self._Y)
                    yield self._state
                    continue

                self._was_active = True

                u_star = (u_L, u_R, s_L, s_R, psi, dpsi)

                # Propagate state: x̂_{k+1} = A_D·x̂_k + B_D·u*_k
                Ax = _dot(A_D, self._x_hat)
                Bu = _dot(B_D, u_star)
                self._x_hat = [Ax[i] + Bu[i] for i in range(4)]

                self._integrate_position(psi)

                # Publish heading as raw IMU (not observer internal state)
                self._s_hat.put(float(self._x_hat[0]))
                self._psi_hat.put(psi)
                self._omL_hat.put(float(self._x_hat[2]))
                self._omR_hat.put(float(self._x_hat[3]))
                self._x_pos_hat.put(self._X)
                self._y_pos_hat.put(self._Y)

            yield self._state

    def _integrate_position(self, psi_now):
        """Integrate X/Y using trapezoidal averaging of speed and heading."""
        OmL = float(self._x_hat[2])
        OmR = float(self._x_hat[3])

        v_now   = 0.5 * (OmL + OmR)
        psi_mid = 0.5 * (self._psi_prev + psi_now)
        v_avg   = 0.5 * (self._v_prev   + v_now)

        self._X += v_avg * _math.cos(psi_mid) * TS
        self._Y += v_avg * _math.sin(psi_mid) * TS

        self._psi_prev = psi_now
        self._v_prev   = v_now
