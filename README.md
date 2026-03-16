# ME 405 — Romi Robot Term Project

**Platform:** Pololu Romi chassis · **Controller:** STM32-based Nucleo (MicroPython)  
**Course:** ME 405 — Mechatronics · Cal Poly SLO

---

## Table of Contents
1. [Project Overview](#1-project-overview)  
2. [Hardware & Wiring](#2-hardware--wiring)  
3. [Software Architecture](#3-software-architecture)  
4. [Task Diagram](#4-task-diagram)  
5. [State Machine — Master Task](#5-state-machine--master-task)  
6. [State Estimator (Luenberger Observer)](#6-state-estimator-luenberger-observer)  
7. [Motor Control & Closed-Loop Driver](#7-motor-control--closed-loop-driver)  
8. [Line Sensor & PID Steering](#8-line-sensor--pid-steering)  
9. [IMU Driver (BNO055)](#9-imu-driver-bno055)  
10. [Trajectory Following](#10-trajectory-following)  
11. [Bump Detection](#11-bump-detection)  
12. [User Interface](#12-user-interface)  
13. [Tuned Parameters](#13-tuned-parameters)  
14. [File Reference](#14-file-reference)  
15. [How to Run](#15-how-to-run)  
16. [Video Demo](#16-video-demo)

---

## 1. Project Overview

This project implements a fully autonomous differential-drive robot using the Pololu Romi chassis.
The robot navigates a closed course by:

- **Line following** using an 8-element IR sensor array and a PID steering controller
- **Dead-reckoning position estimation** using a discrete-time Luenberger observer fusing encoder arcs, motor voltages, IMU heading, and IMU yaw rate
- **Autonomous garage entry** — when the line disappears early in the run, the robot executes a pre-planned cubic spline trajectory (a 90° right turn + straight drive) to reach the garage wall
- **Wall detection** via three interrupt-driven bump sensors, followed by a closed-loop IMU 90° left turn
- **Line reacquisition** by creeping forward until the sensor array detects the track again
- **Course completion detection** — a second line-loss event beyond 2000 mm of travel signals the finish

All tasks run cooperatively under a priority scheduler (`cotask`) and communicate through shared variables and queues (`task_share`).

---

## 2. Hardware & Wiring

### Microcontroller Pin Assignments

| Peripheral | Signal | STM32 Pin |
|---|---|---|
| Left Motor | PWM | B1 |
| Left Motor | DIR | B13 |
| Left Motor | nSLP (enable) | B14 |
| Right Motor | PWM | B0 |
| Right Motor | DIR | B7 |
| Right Motor | nSLP (enable) | H0 |
| Left Encoder | Channel A | A0 |
| Left Encoder | Channel B | A1 |
| Right Encoder | Channel A | A8 |
| Right Encoder | Channel B | A9 |
| IR Sensor | ADC 0–7 | A6, A7, C5, C0, C1, A4, C3, C2 |
| IR Sensor | CTRL (odd emitters) | B15 |
| IMU (BNO055) | I2C SCL/SDA | I2C bus 1 |
| IMU (BNO055) | RST | H1 |
| Bump Switch 1 | Interrupt | B3 |
| Bump Switch 2 | Interrupt | B5 |
| Bump Switch 3 | Interrupt | B4 |

### Motor Driver
The Romi uses **DRV8838** H-bridge motor drivers built into the chassis PCB.
Motors are driven via a shared 20 kHz PWM timer (Timer 3).
A `set_effort(value)` call in the range −100 to +100 sets direction and duty cycle.

### Encoders
Quadrature encoders run at 360 PPR (1440 counts/rev after 4× decoding).
Each wheel has a radius of 35 mm → circumference ≈ 219.9 mm.
Timer 2 (left) and Timer 1 (right) are configured in `ENC_AB` mode for hardware quadrature decoding, with 16-bit overflow handling for rollover.

### IMU
The **BNO055** 9-DOF sensor is operated in **IMU mode (0x08)** — accelerometer + gyroscope fusion, no magnetometer.
It communicates over I2C at 100 kHz with up to 5 retries per transaction (50 ms recovery between retries to handle I2C glitches caused by chassis vibration).
Gyroscope calibration coefficients are saved to `calibration.txt` on first boot and restored automatically on subsequent boots.

---

## 3. Software Architecture

The firmware is organized as a cooperative multitasking system.
All tasks are generator functions that `yield` their current state integer at the end of each execution slice.
The `cotask` priority scheduler calls whichever ready task has the highest priority each loop iteration.
Tasks communicate exclusively through `Share` (single value) and `Queue` (FIFO buffer) objects from `task_share`.

### Source Files

| File | Role |
|---|---|
| `main.py` | Hardware init, shared variable creation, task instantiation, main scheduler loop |
| `motor_driver.py` | Low-level PWM + direction motor driver class |
| `encoder_driver.py` | Quadrature encoder with overflow handling, position, velocity |
| `closed_loop_driver.py` | Generic PID + feedforward controller |
| `line_sensor_driver.py` | 8-channel IR array, calibration, centroid calculation |
| `imu_driver.py` | BNO055 I2C driver, calibration save/load, Euler + gyro reads |
| `task_motor.py` | Closed-loop velocity control task for one motor |
| `task_line.py` | Line sensor reading and PID steering correction task |
| `task_estimator.py` | Discrete Luenberger observer for position/heading estimation |
| `task_trajectory.py` | Cubic spline trajectory generation and following |
| `task_master.py` | Supervisor FSM — sequences all modes through the course |
| `task_bump.py` | Interrupt-driven bump sensor with debounce |
| `task_user.py` | Serial UI for tuning, calibration, and run control |
| `cotask.py` | Cooperative priority task scheduler |
| `task_share.py` | Inter-task `Share` and `Queue` primitives |
| `boot.py` | MicroPython boot configuration |
| `calibration.txt` | Persisted BNO055 gyro calibration coefficients |

---

## 4. Task Diagram

The diagram below shows all tasks, their priorities and periods, and the shared variables / queues that connect them.

```
                         ┌─────────────────────────────────────────────────────────────┐
                         │                        MAIN LOOP                            │
                         │  imu.get_yaw() → psi_share                                  │
                         │  imu.get_yaw_rate() → dpsi_share       (every scheduler tick)│
                         └──────────────────────────┬──────────────────────────────────┘
                                                    │ pri_sched()
          ┌─────────────┬──────────────┬────────────┼────────────┬─────────────┬────────────────┐
          ▼             ▼              ▼            ▼            ▼             ▼                ▼
   ┌─────────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐  ┌─────────────┐
   │ Left Motor  │ │Right Mot.│ │Line Sens.│ │State Est.│ │Bump Sens.│ │Trajectory│  │   Master    │
   │ Pri=1 T=50ms│ │Pri=1 T=50│ │Pri=2 T=50│ │Pri=3 T=20│ │Pri=1 T=40│ │Pri=2 T=50│  │Pri=1 T=100ms│
   └──────┬──────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘  └──────┬──────┘
          │              │            │             │             │            │               │
   left_arc,        right_arc,   centroid,      psi_hat,    crash_detect  leftSetPoint,   lineGo,
   left_volts       right_volts  line_detected  x/y_pos_hat  (Queue)      rightSetPoint   trajGo,
          └──────────────┴────────────┴─────────────┘                         └───────────────┘
                         Shared variables flowing into State Estimator
```

**Priority legend (higher number = higher priority):**

| Task | Priority | Period |
|---|---|---|
| User Interface | 0 | event-driven |
| Left Motor | 1 | 50 ms |
| Right Motor | 1 | 50 ms |
| Bump Sensor | 1 | 40 ms |
| Master | 1 | 100 ms |
| Line Sensor | 2 | 50 ms |
| Trajectory | 2 | 50 ms |
| State Estimator | 3 | 20 ms |

The State Estimator runs at the highest priority and shortest period because accurate timing is critical for the discretized observer matrices (designed at Ts = 20 ms).

---

## 5. State Machine — Master Task

`task_master` is the top-level supervisor. It sequences the robot through all phases of the course. Pressing `y` in the serial terminal starts the run; pressing `x` aborts at any point.

```
                      ┌──────────────────┐
             boot ──► │    S0 IDLE       │ ◄── x (abort from any state)
                      │  wait for 'y'    │
                      └────────┬─────────┘
                               │ masterGo = True
                               ▼
                      ┌──────────────────┐
                      │  S1 LINE FOLLOW  │ ◄──────────────────────────┐
                      │  PID on centroid │                             │
                      └────────┬─────────┘                             │
                               │ line lost for 5 ticks                 │
                    ┌──────────┴──────────┐                            │
            dist < 2000mm          dist > 2000mm                       │
                    │                     │                             │
                    ▼                     ▼                             │
           ┌──────────────┐      ┌──────────────┐                      │
           │  S2 TRAJ     │      │   S6 DONE    │                      │
           │  PREP        │      │  stop motors │                      │
           └──────┬───────┘      └──────────────┘                      │
                  │ set TRAJ_GARAGE, trajGo=True                        │
                  ▼                                                      │
         ┌──────────────────┐                                           │
         │  S3 TRAJ GARAGE  │                                           │
         │  cubic spline    │                                           │
         │  to wall         │                                           │
         └────────┬─────────┘                                           │
                  │ bump detected                                        │
                  ▼                                                      │
         ┌──────────────────┐                                           │
         │  S4 BUMP RECOVER │                                           │
         │  IMU 90° left    │                                           │
         │  closed-loop turn│                                           │
         └────────┬─────────┘                                           │
                  │ heading within 4.5° of target                       │
                  ▼                                                      │
         ┌──────────────────┐                                           │
         │  S5 LINE FIND    │                                           │
         │  creep forward   │                                           │
         │  at 80 mm/s      ├───────────────────────────────────────────┘
         └──────────────────┘  line_detected = True → resume S1
```

### Key thresholds

| Parameter | Value | Meaning |
|---|---|---|
| `_LINE_LOST_TICKS` | 5 | Consecutive no-line ticks before reacting |
| `_FINISH_THRESHOLD_MM` | 2000 mm | Distance beyond which line-loss = finish |
| `_TURN_TARGET_RAD` | π/2 rad | Target left turn after wall bump |
| `_TURN_TOL_RAD` | 0.08 rad (~4.5°) | Angular tolerance to exit turn state |
| `_TURN_SPEED` | 80 mm/s | Wheel speed during IMU-guided turn |
| `_CREEP_SPEED` | 80 mm/s | Forward speed during line-find crawl |

---

## 6. State Estimator (Luenberger Observer)

The robot's global pose (X, Y, heading) is tracked by a discrete-time Luenberger state observer implemented in `task_estimator.py`.

### State vector and inputs

The observer state is:

```
x̂ = [ s,  ψ,  Ω_L,  Ω_R ]
```

- **s** — average arc length (mm), blended from both encoders  
- **ψ** — internal heading estimate (rad)  
- **Ω_L** — left wheel speed (mm/s)  
- **Ω_R** — right wheel speed (mm/s)  

The input vector is:

```
u* = [ u_L,  u_R,  s_L,  s_R,  ψ_IMU,  dψ_IMU ]
```

- **u_L / u_R** — motor voltages (V), converted from PWM effort  
- **s_L / s_R** — per-tick encoder arc deltas (mm) — *not cumulative*  
- **ψ_IMU** — BNO055 heading (rad)  
- **dψ_IMU** — BNO055 yaw rate (rad/s)  

### Discretized matrices (Ts = 20 ms)

Designed in MATLAB for: r = 35 mm, wheelbase = 140 mm, motor time constant τ = 0.11 s, Km = 3.49 rad/V/s.

```
       A_D (4×4)                          B_D (4×6)
┌                          ┐     ┌                                         ┐
│  0.6999  0.0000  0.2684  0.2684│     │  0.0933  0.0933  0.15    0.15    0.0000  0.0000 │
│  0.0000  0.7228 -0.003   0.003 │     │ -0.0011  0.0011 -0.0008  0.0008  0.1640  0.0039 │
│ -0.0332  3.6733  0.7773  0.0445│     │  0.5624  0.0151  0.0272  0.0059 -2.1732 -0.1814 │
│ -0.0332 -3.6733  0.0445  0.7773│     │  0.0151  0.5624  0.0059  0.0272  2.1732  0.1814 │
└                          ┘     └                                         ┘
```

Update equation each 20 ms tick:

```
x̂_{k+1} = A_D · x̂_k + B_D · u*_k
```

### Position integration

Global X, Y are integrated using the trapezoidal rule every observer tick:

```
v_avg  = 0.5 · (v_{k-1} + v_k)         where  v = 0.5 · (Ω_L + Ω_R)  [mm/s]
ψ_mid  = 0.5 · (ψ_{k-1} + ψ_k)         where  ψ = raw IMU heading

X += v_avg · cos(ψ_mid) · Ts
Y += v_avg · sin(ψ_mid) · Ts
```

> **Design note:** `psi_hat` published to all other tasks is always the raw IMU heading — not the observer's internal `x̂[1]`. The `A_D[1][1]` gain of 0.7228 causes the observer's internal heading to decay toward zero rather than tracking absolute orientation, so using it for the integrator or trajectory task would cause the robot to gradually ignore turns. The IMU reading is used directly instead.

### Freeze guard

When `|u_L| < 0.05 V` **and** `|u_R| < 0.05 V`, the observer update is skipped.  
The `B_D` column gains of ±2.17 on ψ/dψ amplify even small gyro noise (~0.05 deg/s) into the wheel-speed estimates and then into X/Y when the robot is stationary. Freezing prevents this drift.

---

## 7. Motor Control & Closed-Loop Driver

Each motor runs its own instance of `task_motor`, which wraps:

- **`motor_driver`** — DRV8838 PWM + direction control (`motor_driver.py`)
- **`Encoder`** — hardware quadrature decode with 16-bit overflow handling (`encoder_driver.py`)
- **`closed_loop_control`** — PID + feedforward (`closed_loop_driver.py`)

### PID + Feedforward control law

```
e(t)       = setpoint − velocity_measured
integral  += e · dt          (anti-windup clamp: ±100)
derivative = (e − e_prev) / dt
output     = Kp·e + Ki·integral + Kd·derivative + Kff·setpoint
```

The **feedforward term** (`Kff · setpoint`) provides an open-loop estimate of the needed effort, allowing the PID terms to focus only on correcting residual error. This significantly improves step-response rise time.

### Steering correction

During line following the master speed setpoint is modified per-wheel:

```
left  setpoint = baseSpeed − steeringCorrection
right setpoint = baseSpeed + steeringCorrection
```

The `steeringCorrection` value is the output of the line sensor PID controller operating on the centroid error.

### Arc delta publishing

Each motor task publishes a **per-tick arc delta** (mm moved this step) to the observer — not a cumulative total. This matches the discretization assumption of the B_D matrix and prevents the large initial-value jump that would occur if a growing total were fed in on the first active tick.

---

## 8. Line Sensor & PID Steering

`task_line` reads the 8-element IR sensor array and computes a steering correction.

### Centroid calculation

Each of the 8 sensors is normalized to [0, 1] using per-sensor white/black calibration values:

```
normalized[i] = (raw[i] − white[i]) / (black[i] − white[i])
```

The centroid is the weighted average of sensor positions, centered so that 0 = line is centered:

```
centroid = (Σ i · normalized[i]) / (Σ normalized[i])  −  3.5
```

Returns `None` if `Σ normalized[i] = 0` (no line detected).

### PID steering

```
steeringCorrection = Kp · centroid + Ki · ∫centroid dt + Kd · (d/dt centroid)
```

Tuned values: **Kp = 40.0, Ki = 0.0, Kd = 2.0**

The correction is applied with opposite signs to each wheel, creating differential speed that steers the robot toward the line.

---

## 9. IMU Driver (BNO055)

The BNO055 is operated in **IMU mode (0x08)** — gyroscope + accelerometer fusion only, no magnetometer.

### Calibration flow

On every boot, `main.py` attempts to load saved calibration from `calibration.txt`. If the file exists and contains exactly 11 coefficients, they are written to the sensor in CONFIG mode before switching to fusion mode. This avoids the 5–10 second still-hold gyro calibration on every power cycle.

If no file is found:
1. Sensor switches to IMU mode
2. `manual_calibrate()` blocks until gyro reaches 3/3
3. Coefficients are read back and saved to `calibration.txt`

```
calibration.txt format:
0,0,0,0,0,0,-1,-1,-1,1000,480   ← 11 signed 16-bit integers
```

### Heading reference

`main.py` snapshots the heading at startup:
```python
_imu_heading_offset = imu.get_yaw()
```

Every main-loop iteration publishes:
```python
psi_share.put(_PSI_SIGN * (imu.get_yaw() - _imu_heading_offset) * DEG_TO_RAD)
```

`_PSI_SIGN = -1` corrects for the BNO055 Euler heading increasing clockwise (opposite to the mathematical positive convention used in the observer).

### I2C robustness

All register reads and writes retry up to 5 times with a 50 ms recovery delay. This is necessary because motor vibration causes persistent I2C glitches — a single retry would not be enough.

---

## 10. Trajectory Following

When the robot loses the line early in the course (garage entry), `task_master` arms the `TRAJ_GARAGE` trajectory.

### Trajectory table (local coordinates)

All trajectory waypoints are defined in a **local frame** — origin at the robot's pose when `trajectoryGo` goes True, X forward, Y left. This means the same table works regardless of where on the track the garage entry happens.

```
TRAJ_GARAGE waypoints:
 t(s)    x(mm)   y(mm)   vx(mm/s)  vy(mm/s)
  0.0      0.0     0.0    150.0       0.0     ← entry, heading straight
  0.4     55.0   -35.0    130.0     -80.0     ← begin right-hand arc
  0.8    100.0   -90.0     80.0    -120.0
  1.2    120.0  -155.0      0.0    -150.0     ← pointing right (90° turn complete)
  2.0    120.0  -305.0      0.0    -150.0     ← straight toward wall
  3.0    120.0  -455.0      0.0    -150.0
  4.0    120.0  -605.0      0.0    -150.0
  5.2    120.0  -755.0      0.0    -150.0     ← bump expected well before here
```

### Cubic spline generation

At each 0.5 s update interval, a cubic spline is fit between the robot's current estimated state and the lookahead point (1.5 s ahead in the table):

```
q(t) = a + b·t + c·t² + d·t³

Boundary conditions:
  q(t_now)    = current position (local frame)
  q̇(t_now)   = current velocity (local frame)
  q(t_look)   = table position at t_now + 1.5s
  q̇(t_look)  = table velocity at t_now + 1.5s
```

The spline velocity at `t_now` is differentiated and converted to differential wheel speed setpoints via:

```
v   =  vx·cos(ψ) + vy·sin(ψ)      (forward component)
ω   = lateral_velocity / (wheelbase/2)
vL  = v − ω · (wheelbase/2)
vR  = v + ω · (wheelbase/2)
```

---

## 11. Bump Detection

`task_bump` uses three **interrupt-driven** bump switches (falling-edge `ExtInt`) to detect wall contact.

A two-stage software debounce prevents multiple triggers:
1. The ISR fires, sets a bitmask bit for that pin, and **disables** the interrupt on that channel
2. On the next task tick, the previous debounce mask is checked and any pending channels are **re-enabled**
3. The bumper is only registered if the pin is still low when the ISR fires

Detected bumps are written to the `crash_detect` Queue. `task_master` polls this queue; when non-empty it immediately stops the trajectory and begins the 90° IMU turn.

---

## 12. User Interface

`task_user` provides a serial terminal interface over Bluetooth (HC-05 at 230400 baud) for tuning and control. It falls back to USB for data output.

### Command menu

```
+------------------------------------------------------------------------------+
| ME 405 Romi Tuning Interface                                                 |
+---+--------------------------------------------------------------------------+
| h | Print this help menu                                                     |
| k | Enter new gain values                                                    |
| s | Choose a new setpoint                                                    |
| l | Trigger left motor step response and print results                       |
| r | Trigger right motor step response and print results                      |
| c | Calibrate line sensor (place on white first, then black)                 |
| d | Lock on to line and drive (press x to stop)                              |
| e | Toggle state estimator ON / OFF                                          |
| p | Print current estimator state (X, Y, heading, wheel speeds)             |
| y | START full track run  (master task takes over; press x to abort)        |
+---+--------------------------------------------------------------------------+
```

### Startup sequence for a full run

1. Power on — IMU loads calibration from `calibration.txt` (or prompts for gyro cal if missing)
2. Open a serial terminal and connect
3. Press `c` twice to calibrate line sensors (white surface first, then black)
4. Place robot at start position on the line
5. Press `y` — master task takes over, motors start, estimator resets to zero

---

## 13. Tuned Parameters

### Motor PID + Feedforward

| Gain | Left | Right |
|---|---|---|
| Kp | 0.037 | 0.037 |
| Ki | 0.010 | 0.010 |
| Kd | 0.000 | 0.000 |
| Kff | 0.096 | 0.096 |

### Line Sensor PID

| Gain | Value |
|---|---|
| Kp | 40.0 |
| Ki | 0.0 |
| Kd | 2.0 |
| Base speed | 100 mm/s |

The derivative term (Kd = 2.0) was critical for stable tracking through tight curves. Without it the robot oscillated badly at higher speeds.

---

## 14. File Reference

```
romi/
├── main.py               — Hardware init, task wiring, scheduler loop
├── boot.py               — MicroPython USB/filesystem boot config
├── calibration.txt       — Persisted BNO055 gyro calibration coefficients
│
├── motor_driver.py       — DRV8838 PWM motor driver
├── encoder_driver.py     — Quadrature encoder with overflow handling
├── closed_loop_driver.py — PID + feedforward controller
├── line_sensor_driver.py — 8-channel IR array driver, centroid
├── imu_driver.py         — BNO055 I2C driver, calibration save/load
│
├── task_motor.py         — Motor velocity control task
├── task_line.py          — Line sensor + PID steering task
├── task_estimator.py     — Luenberger observer, X/Y/heading estimation
├── task_trajectory.py    — Cubic spline trajectory generator/follower
├── task_master.py        — Top-level course supervisor FSM
├── task_bump.py          — Interrupt-driven bump detection + debounce
├── task_user.py          — Serial terminal UI (Bluetooth + USB)
│
├── cotask.py             — Cooperative priority task scheduler
└── task_share.py         — Inter-task Share and Queue primitives
```

---

## 15. How to Run

### Requirements
- Pololu Romi chassis with STM32 Nucleo (MicroPython firmware)
- BNO055 IMU breakout board wired to I2C bus 1
- HC-05 Bluetooth module on UART 5 at 230400 baud (optional — USB works for commands)
- 8-element IR sensor array connected to the ADC pins listed above

### Deploying

Copy all `.py` files and `calibration.txt` to the root of the MicroPython filesystem. The board will automatically execute `main.py` on power-up.

```bash
# Using mpremote (recommended):
mpremote connect /dev/ttyACM0 cp *.py :
mpremote connect /dev/ttyACM0 cp calibration.txt :
```

### First boot (no calibration file)

On first boot the robot will print:
```
No calibration file found. Starting gyro calibration...
GYRO : Set the robot on a flat surface and hold it PERFECTLY STILL
       Wait for GYRO to reach 3/3. This usually takes 5-10 seconds.
GYRO:0/3  (waiting for 3/3 -- hold still)
```
Hold the robot completely still. Once `GYRO:3/3` is reached, `calibration.txt` is saved and the robot is ready. Subsequent boots skip this step automatically.

### Running the course

1. Connect a serial terminal at 230400 baud (Bluetooth) or via USB
2. Press `c` — place robot on **white surface**, press Enter
3. Press `c` again — place robot on **black line**, press Enter
4. Place robot at the course start, centered on the line
5. Press `y` to begin the autonomous run
6. Press `x` at any time to abort and stop all motors

---

## 16. Video Demo

> 📹 **[Insert YouTube/video link here]**

---

*Repository maintained as part of ME 405 — Mechatronics, Cal Poly SLO.*
