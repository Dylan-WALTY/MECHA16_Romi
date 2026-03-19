# ME 405 — Romi Robot Term Project

**Platform:** Pololu Romi chassis · **Controller:** STM32-based Nucleo · **Firmware:** MicroPython  
**Course:** ME 405 — Mechatronics · Cal Poly SLO

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Hardware & Wiring](#2-hardware--wiring)
3. [Software Architecture](#3-software-architecture)
4. [Task Diagram](#4-task-diagram)
5. [State Machine — Master Task](#5-state-machine--master-task)
6. [Motor Control & PID Driver](#6-motor-control--pid-driver)
7. [Line Sensor & PID Steering](#7-line-sensor--pid-steering)
8. [State Estimator — Luenberger Observer](#8-state-estimator--luenberger-observer)
9. [IMU Driver (BNO055)](#9-imu-driver-bno055)
10. [Bump Detection](#10-bump-detection)
11. [User Interface](#11-user-interface)
12. [Tuned Parameters & Results](#12-tuned-parameters--results)
13. [Challenges & Reflection](#13-challenges--reflection)
14. [File Reference](#14-file-reference)
15. [How to Run](#15-how-to-run)
16. [Video Demo](#16-video-demo)

---

## 1. Project Overview

This project implements a fully autonomous differential-drive robot on the Pololu Romi chassis, controlled by an STM32 Nucleo running MicroPython. The robot navigates a closed competition course without any human input after the start command is issued.

The course consists of several distinct segments driven by a 10-state encoder-distance sequencer:

- **Line following** in multiple directions using an 8-element IR sensor array and a PID steering controller
- **Precision pivot turns** using encoder-counted wheel arc distance to achieve repeatable 90° and 180° headings
- **Wall detection** via three interrupt-driven bump sensors, which trigger a pivot turn into the next course segment
- **Dead-reckoning position estimation** using a discrete-time Luenberger observer fusing encoder arcs, motor voltages, IMU heading, and IMU yaw rate

All tasks run cooperatively under a priority scheduler (`cotask`) and communicate exclusively through shared variables and queues (`task_share`). No global variables are used for inter-task data.

The robot successfully completed the full course during the final demonstration.

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
| IR Sensor | CTRL (emitter enable) | B15 |
| IMU (BNO055) | I2C bus 1 | SCL/SDA (hardware) |
| IMU (BNO055) | RST | H1 |
| Bump Switch 0 | Interrupt | B3 |
| Bump Switch 1 | Interrupt | B5 |
| Bump Switch 2 | Interrupt | B4 |

### Motor Driver

The Romi uses **DRV8838** H-bridge drivers built into the chassis PCB. Both motors share a 20 kHz PWM timer (Timer 3). The `Motor.set_effort(value)` method accepts a signed percentage in [−100, 100], handling direction pin polarity automatically.

### Encoders

Quadrature encoders run at 12 PPR, decoded to 1440 counts/rev by the STM32's hardware `ENC_AB` timer mode due to the motor-wheel gear ratio. Each wheel has a radius of 35 mm giving a circumference of ≈ 219.9 mm. Timer 2 drives the left encoder and Timer 1 drives the right. The driver handles 16-bit counter overflow in both directions.

### IMU

The **BNO055** 9-DOF sensor runs in **IMU mode (0x08)** — gyroscope + accelerometer fusion only, no magnetometer. It communicates over I2C at 100 kHz. All register transactions retry up to 5 times with a 50 ms recovery delay to handle I2C glitches caused by motor vibration. Calibration coefficients are saved to `calibration.txt` on first boot and restored on every subsequent boot, eliminating the 5–10 second gyro still-hold.

### Photos

| | |
|---|---|
| ![Romi top view](docs/romi_top.jpg) | ![Romi sensor array](docs/romi_sensors.jpg) |
| *Top view — Nucleo, IMU, and Bluetooth module* | *Front — 8-element IR array and bump switches* |

---

## 3. Software Architecture

The firmware is a cooperative multitasking system. Every task is a Python generator function that `yield`s its current state integer at the end of each execution slice. The `cotask` priority scheduler calls the highest-priority ready task on each main-loop iteration. Tasks share data exclusively through `Share` (single-value, interrupt-safe) and `Queue` (FIFO ring buffer) objects.

### Source Files

| File | Role |
|---|---|
| `main.py` | Hardware init, shared variable creation, task wiring, scheduler loop |
| `motor_driver.py` | DRV8838 PWM + direction motor driver |
| `encoder_driver.py` | Quadrature encoder with 16-bit overflow handling, position and velocity |
| `closed_loop_driver.py` | Generic PID + feedforward controller with anti-windup |
| `line_sensor_driver.py` | 8-channel IR array, per-sensor calibration, centroid computation |
| `imu_driver.py` | BNO055 I2C driver, calibration save/load, Euler and gyro reads |
| `task_motor.py` | Closed-loop velocity control task for one motor |
| `task_line.py` | Line sensor reading, PID steering correction, IR calibration |
| `task_estimator.py` | Discrete Luenberger observer — position, heading, wheel speed |
| `task_master.py` | 10-state supervisor FSM — sequences the full competition course |
| `task_bump.py` | Interrupt-driven bump sensor with two-stage software debounce |
| `task_user.py` | Serial terminal UI for tuning, calibration, and run control |
| `task_trajectory.py` | Cubic spline trajectory generator (available, not used in final run) |
| `cotask.py` | Cooperative priority task scheduler (provided framework) |
| `task_share.py` | Interrupt-safe `Share` and `Queue` primitives (provided framework) |
| `boot.py` | MicroPython USB/filesystem boot configuration |

---

## 4. Task Diagram

The diagram below shows every task, its priority and period, and the inter-task shares that connect them. The IMU is polled directly in the main loop because its I2C reads can block for up to 50 ms on a glitch retry; placing this outside the task scheduler prevents one bad I2C transaction from starving all other tasks.

```
                   ┌──────────────────────────────────────────────────────┐
                   │                     MAIN LOOP                        │
                   │  imu.get_yaw()     → psi_share  (rad)               │
                   │  imu.get_yaw_rate()→ dpsi_share (rad/s)             │
                   └───────────────────────┬──────────────────────────────┘
                                           │ pri_sched()
        ┌───────────┬──────────┬───────────┼──────────┬──────────┬────────┐
        ▼           ▼          ▼           ▼          ▼          ▼        ▼
  ┌──────────┐ ┌─────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌──────┐ ┌──────┐
  │Left Motor│ │Rt Motor │ │  Line  │ │  State │ │  Bump  │ │Master│ │ User │
  │Pri=1 50ms│ │Pri=1 50 │ │Pri=2 50│ │Est.    │ │Pri=1   │ │Pri=1 │ │Pri=0 │
  │          │ │         │ │        │ │Pri=3 20│ │40ms    │ │100ms │ │event │
  └────┬─────┘ └────┬────┘ └───┬────┘ └───┬────┘ └───┬────┘ └──┬───┘ └──┬───┘
       │             │          │           │           │         │        │
  left_arc      right_arc  centroid,    psi_hat,   crash_    lineGo,  gainsUpd,
  left_volts   right_volts line_detect  x/y_pos    detect    leftSP,  calFlag,
       └─────────────┴──────────┤           │    (Queue)  rightSP   masterGo
                                └───────────┘
                        All four feeds into State Estimator
```

**Task priority and timing:**

| Task | Priority | Period | Notes |
|---|---|---|---|
| User Interface | 0 | event-driven | Lowest — yields immediately if no serial input |
| Left Motor | 1 | 50 ms | Runs PID + encoder update |
| Right Motor | 1 | 50 ms | Runs PID + encoder update |
| Bump Sensor | 1 | 40 ms | Re-enables debounced interrupt channels |
| Master | 1 | 100 ms | Sequences all course states |
| Line Sensor | 2 | 50 ms | Reads IR array, computes centroid |
| State Estimator | 3 | 20 ms | Highest — timing-critical for observer matrices |

The State Estimator runs at the highest priority and shortest period because the discretized observer matrices were designed at exactly Ts = 20 ms. Running it slower or with jitter would cause the estimated state to drift.

---

## 5. State Machine — Master Task

`task_master` is the top-level supervisor. All transitions are driven by encoder distance, not time or line-loss events, which makes the sequence robust to speed variations and line-sensor noise. Pressing `y` in the serial terminal starts the run; pressing `x` aborts and stops all motors immediately.

```
                  ┌─────────────────────┐
         boot ──► │     S0  IDLE        │ ◄── x (abort from any state)
                  │   wait for 'y'      │
                  └──────────┬──────────┘
                             │ masterGo = True
                             ▼
                  ┌─────────────────────┐
                  │  S1  LINE FOLLOW A  │  line follow east, 1775 mm
                  │  (top curve)        │
                  └──────────┬──────────┘
                             │ encoder distance reached
                             ▼
                  ┌─────────────────────┐
                  │  S2  TURN RIGHT     │  pivot 90° right, 112 mm arc
                  │  (face west)        │
                  └──────────┬──────────┘
                             │ encoder distance reached
                             ▼
                  ┌─────────────────────┐
                  │  S3  DRIVE TO WALL  │  straight west, 150 mm/s
                  │                     │  exit: bump OR 600 mm fallback
                  └──────────┬──────────┘
                             │ bump detected or max distance
                             ▼
                  ┌─────────────────────┐
                  │  S4  TURN LEFT      │  pivot 90° left, 127 mm arc
                  │  (face south)       │
                  └──────────┬──────────┘
                             │ encoder distance reached
                             ▼
                  ┌─────────────────────┐
                  │  S5  LINE FOLLOW B  │  line follow south, 368 mm
                  └──────────┬──────────┘
                             │ encoder distance reached
                             ▼
                  ┌─────────────────────┐
                  │  S6  TURN RIGHT 2   │  pivot 90° right, 110 mm arc
                  │  (face west)        │
                  └──────────┬──────────┘
                             │ encoder distance reached
                             ▼
                  ┌─────────────────────┐
                  │  S7  LINE FOLLOW C  │  line follow west, 2000 mm
                  │  (fixed distance)   │  no line-loss detection
                  └──────────┬──────────┘
                             │ encoder distance reached
                             ▼
                  ┌─────────────────────┐
                  │  S8  TURN 180°      │  pivot 180° left, 170 mm arc
                  │  (face east)        │
                  └──────────┬──────────┘
                             │ encoder distance reached
                             ▼
                  ┌─────────────────────┐
                  │  S9  LINE FOLLOW D  │  line follow east, 750 mm
                  │  (slalom section)   │
                  └──────────┬──────────┘
                             │ encoder distance reached
                             ▼
                  ┌─────────────────────┐
                  │  S10  DONE          │  motors off, masterGo = False
                  └─────────────────────┘
```

### Distance measurement

`task_master` never calls `enc.update()` — the motor tasks update the encoders every 50 ms. The master reads `enc.get_position()` directly, which is always fresh. At the start of each segment both encoder positions are snapped, and the termination condition is checked against the **average** of the left and right arc deltas:

```
dist = (|pos_L - snap_L| + |pos_R - snap_R|) / 2
```

Averaging both wheels makes the distance estimate robust to steering-induced asymmetry during line-follow segments, where the two wheels may travel different arcs through a curve.

### Pivot turn calibration

Each pivot arc distance was tuned empirically on the actual course:

| Turn | Direction | Arc (mm) | Resulting heading |
|---|---|---|---|
| S2 | RIGHT 90° | 112 | West |
| S4 | LEFT 90° | 127 | South |
| S6 | RIGHT 90° | 110 | West |
| S8 | LEFT 180° | 170 | East |

---

## 6. Motor Control & PID Driver

Each motor runs its own instance of `task_motor`, which wraps `motor_driver`, `encoder_driver`, and `closed_loop_control` into a three-state FSM.

### PID + Feedforward control law

```
e(t)        = setpoint − velocity_measured
integral   += e · dt          (clamped to ±100 for anti-windup)
derivative  = (e − e_prev) / dt
output      = Kp·e + Ki·integral + Kd·derivative + Kff·setpoint
```

The **feedforward term** (`Kff · setpoint`) provides an open-loop estimate of the effort needed to achieve the target speed, so the PID terms correct only the residual error. This significantly improves step-response rise time without requiring a large Kp that would cause overshoot.

The output is clamped to [−100, 100] before being sent to the motor driver.

### Steering correction

During line following the master sets a `baseSpeed` and the line sensor PID produces a `steeringCorrection`. The motor task applies it with opposite signs per wheel:

```
left  setpoint = baseSpeed − steeringCorrection
right setpoint = baseSpeed + steeringCorrection
```

The effective setpoint is clamped to ±350 mm/s before the PID to prevent runaway commands from stale trajectory values if state transitions occur mid-tick.

### Arc delta publishing

Each motor task computes the **per-tick arc delta** (mm moved since last tick) and publishes it to the state estimator. Using a delta — not a running total — correctly matches the discretization assumption of the B_D observer matrix and prevents a large initial-value step on the first active tick.

### Step-response data

Motor velocity step-response data is collected via the `l` (left) and `r` (right) commands. The motor runs to the setpoint, buffers 30 velocity/timestamp pairs, then stops and streams the data as `time_ms, velocity_mm_s` CSV over USB VCP. This data was plotted in Python to evaluate rise time, overshoot, and steady-state error before finalising the gains below.

> 📊 **Replace this line with your step response plot.**  
> Add the image to your repo (e.g. `docs/step_response.png`) and update the line below.

![Motor step response](docs/step_response.png)  
*Motor step response at 150 mm/s setpoint. Gains: Kp = 0.037, Ki = 0.010, Kff = 0.096.*

---

## 7. Line Sensor & PID Steering

`task_line` reads the 8-element IR sensor array every 50 ms and computes a signed centroid error used as the process variable for the line-following PID controller.

### Centroid calculation

Each sensor is normalised to [0.0, 1.0] using stored white and black calibration values:

```
normalized[i] = (raw[i] − white[i]) / (black[i] − white[i])
```

The weighted centroid is computed and centred so that 0 = line directly under the array midpoint, negative = line to the left, positive = line to the right:

```
centroid = (Σ i · normalized[i]) / (Σ normalized[i])  −  3.5
```

`get_centroid()` returns `None` when the total normalised weight is zero — indicating that no sensor detects the line. The motor task treats `None` as `0` (hold last correction) to avoid a derivative spike on line re-acquisition.

### PID steering

```
steeringCorrection = Kp · centroid + Ki · ∫centroid dt + Kd · (d/dt centroid)
```

Tuned values: **Kp = 34.0, Ki = 0.1, Kd = 0.0**

The correction is applied with opposite signs to each wheel setpoint. Because the setpoint is a speed in mm/s, the steering gain must be large enough to shift wheel speeds meaningfully relative to the base speed.

### Centroid data logging

While in manual line-follow mode (`d` command), centroid values and timestamps are buffered and streamed over USB after the run stops — useful for diagnosing oscillation or sensor placement issues.

![Centroid vs time](docs/centroid_plot.png)
*Centroid signal during a full manual line-follow run. Oscillations through tight curves are visible; they stayed within ±1.5 sensor widths.*

---

## 8. State Estimator — Luenberger Observer

`task_estimator` implements a discrete-time Luenberger state observer that tracks robot pose without GPS. It fuses motor voltages, encoder arc deltas, IMU heading, and IMU yaw rate to estimate the state vector:

```
x̂ = [ s,  ψ,  Ω_L,  Ω_R ]
```

- **s** — average arc length (mm)
- **ψ** — internal heading estimate (rad)
- **Ω_L / Ω_R** — left and right wheel speeds (mm/s)

### Input vector

```
u* = [ u_L,  u_R,  s_L,  s_R,  ψ_IMU,  dψ_IMU ]
```

- **u_L / u_R** — motor voltages converted from PWM effort (V)
- **s_L / s_R** — per-tick encoder arc deltas (mm)
- **ψ_IMU** — BNO055 heading (rad)
- **dψ_IMU** — BNO055 yaw rate (rad/s)

### Observer matrices (Ts = 20 ms)

Designed in MATLAB for r = 35 mm, wheelbase = 140 mm, motor time constant τ = 0.11 s, Km = 3.49 rad/V/s:

```
       A_D (4×4)                              B_D (4×6)
┌                              ┐   ┌                                           ┐
│  0.6999  0.0000  0.2684  0.2684│   │ 0.0933  0.0933  0.15    0.15   0.000  0.000│
│  0.0000  0.7228 -0.003   0.003 │   │-0.0011  0.0011 -0.0008  0.0008 0.164  0.004│
│ -0.0332  3.6733  0.7773  0.0445│   │ 0.5624  0.0151  0.0272  0.0059-2.173 -0.181│
│ -0.0332 -3.6733  0.0445  0.7773│   │ 0.0151  0.5624  0.0059  0.0272 2.173  0.181│
└                              ┘   └                                           ┘
```

Update equation each 20 ms tick:

```
x̂_{k+1} = A_D · x̂_k + B_D · u*_k
```

### Position integration

Global X, Y position is integrated using the trapezoidal rule:

```
v_avg  = 0.5 · (v_{k-1} + v_k)      where v = 0.5 · (Ω_L + Ω_R)
ψ_mid  = 0.5 · (ψ_{k-1} + ψ_k)      using raw IMU heading

X += v_avg · cos(ψ_mid) · Ts
Y += v_avg · sin(ψ_mid) · Ts
```

### Design notes

**Published heading:** `psi_hat` shared to other tasks is always the raw IMU heading, not the observer's internal `x̂[1]`. The `A_D[1][1]` gain of 0.7228 causes the internal heading to decay toward zero rather than tracking absolute orientation, so using it for X/Y integration or turn control would cause the robot to gradually ignore rotation. The IMU reading is used directly instead.

**Freeze guard:** When both motor voltages fall below 0.05 V, the observer update is skipped. The large ψ/dψ columns in B_D (±2.17) amplify even small gyro noise into the wheel-speed estimates and then into X/Y when the robot is stationary. Freezing the update during motor-off periods eliminates this drift.

---

## 9. IMU Driver (BNO055)

The BNO055 is operated in **IMU mode (0x08)** — gyroscope + accelerometer fusion only. This mode was chosen over NDOF (which includes magnetometer) because the competition field had unpredictable magnetic interference that made absolute heading unreliable.

### Calibration flow

```
Boot
 │
 ├─ calibration.txt exists and valid? ──── YES ──► write coefficients in CONFIG mode
 │                                                  switch to IMU mode (0x08) → ready
 └─ NO ──► switch to IMU mode
           manual_calibrate() blocks until GYRO = 3/3
           save_calibration() writes calibration.txt
           → ready
```

Once saved, subsequent boots load the file and skip the still-hold entirely. The file stores 11 signed 16-bit integers as comma-separated text:

```
calibration.txt example:
0,0,0,0,0,0,-1,-1,-1,1000,480
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

`_PSI_SIGN = -1` corrects for the BNO055 Euler heading increasing clockwise, which is opposite to the mathematical positive convention used in the observer matrices.

---

## 10. Bump Detection

`task_bump` uses three bump switches wired to falling-edge external interrupts (`ExtInt`). A two-stage software debounce prevents multiple events from a single physical contact:

1. **ISR fires** — sets a bitmask bit for the pin number, disables the interrupt channel for that pin, and enqueues the pin number in `crash_detect` if the pin reads low
2. **Next task tick** — the previous cycle's pending bitmask is transferred to a re-enable mask, and all flagged channels are re-enabled

This one-cycle delay gives the switch time to stop bouncing before the interrupt is re-armed. Checking `pin.value() == 0` inside the ISR rejects noise spikes that self-resolve before the ISR can run.

`task_master` polls the `crash_detect` Queue and also reads the bump pins directly as a fallback, since a very brief contact might drain from the queue before master checks it.

---

## 11. User Interface

`task_user` provides a serial terminal over Bluetooth (UART 5 at 230400 baud). Velocity and centroid data are routed to USB VCP for capture at higher throughput. The task runs at priority 0 (lowest) and only does work when a serial byte is available.

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

1. Power on — IMU loads calibration from `calibration.txt` (or runs gyro cal if missing)
2. Connect a serial terminal at 230400 baud over Bluetooth (or USB)
3. Press `c` — robot must be on the **white surface**. Press Enter.
4. Press `c` again — robot must be on the **black line**. Press Enter.
5. Place the robot at the start position, centred on the line
6. Press `y` — the master task takes over and the run begins

---

## 12. Tuned Parameters & Results

### Motor PID + Feedforward

These gains were tuned by running step responses with the `l` and `r` commands and plotting the resulting CSV data. A low Kp combined with a feedforward term gave the fastest rise with no overshoot.

| Gain | Left | Right |
|---|---|---|
| Kp | 0.037 | 0.037 |
| Ki | 0.010 | 0.010 |
| Kd | 0.000 | 0.000 |
| Kff | 0.096 | 0.096 |

### Line Sensor PID

| Gain | Value |
|---|---|
| Kp | 34.0 |
| Ki | 0.1 |
| Kd | 0.0 |
| Base speed | 150 mm/s |

A pure proportional controller (Kd = 0) proved sufficient at 150 mm/s. Earlier testing at higher speeds required a small derivative term, but the added noise sensitivity outweighed the stability benefit at the chosen operating speed.

### Pivot turn distances

Pivot arc distances were determined empirically by placing the robot at a known heading marker and adjusting until it returned to within ±5°. The 180° turn (S8) required the most iteration — slight asymmetry between the motors meant the naive `2 × 90°` distance did not work.

### Course completion

The robot completed the full 10-state course sequence in the final demonstration. All four line-follow segments tracked successfully, both wall bumps in S3 triggered cleanly, and all four pivot turns landed within the tolerance needed for the next segment to re-acquire the line.

---

## 13. Challenges & Reflection

**I2C reliability under vibration:** The BNO055 driver originally used a single-attempt read, which failed sporadically once the motors were running. Motor vibration causes persistent multi-read I2C bus errors, not one-shot glitches — a single retry was insufficient. The 5-retry, 50 ms recovery implementation resolved this entirely.

**Motor task race condition:** An early version of `task_motor` used a one-shot flag to detect step-test mode vs. autonomous mode. Because the motor task period (50 ms) is faster than the master task (100 ms), the motor task could check the flag between master's writes to `goFlag` and `lineGo`, misclassify the run as a step test, fill the data buffer, and auto-stop mid-course. The fix was a latching `_autonomous_mode` flag that can only go True during an activation, not back to False.

**First PID tick dt spike:** On the first tick after a motor enable, the `dt` computed from `ticks_diff` spanned the entire idle period — sometimes hundreds of milliseconds — causing a large derivative kick and integral step. Skipping the PID on the first tick and only resetting the timestamp solved this cleanly.

**Observer heading drift:** Early versions of the state estimator used the observer's internal heading state `x̂[1]` for X/Y integration. The `A_D[1][1] = 0.7228` decay caused this estimate to drift back toward zero over a long straight, making the position estimate curve even when the robot was driving straight. Switching to raw IMU heading for both integration and publishing eliminated the drift.

**Pivot calibration sensitivity:** The encoder-based pivot turns are sensitive to surface conditions — carpet vs. hard floor changed the required arc by 10–15%. Final calibration was done on the actual competition surface.

---

## 14. File Reference

```
romi/
├── main.py               — Hardware init, task wiring, scheduler loop
├── boot.py               — MicroPython USB/filesystem boot config
├── calibration.txt       — BNO055 gyro calibration coefficients (auto-generated)
│
├── motor_driver.py       — DRV8838 PWM motor driver
├── encoder_driver.py     — Quadrature encoder, overflow handling, velocity
├── closed_loop_driver.py — PID + feedforward controller with anti-windup
├── line_sensor_driver.py — 8-channel IR array, calibration, centroid
├── imu_driver.py         — BNO055 I2C driver, calibration save/load
│
├── task_motor.py         — Motor closed-loop velocity control task
├── task_line.py          — IR sensor reading + PID steering task
├── task_estimator.py     — Luenberger observer, X/Y/heading estimation
├── task_master.py        — 10-state encoder-distance course sequencer
├── task_bump.py          — Interrupt-driven bump detection with debounce
├── task_user.py          — Serial terminal UI (Bluetooth + USB)
├── task_trajectory.py    — Cubic spline trajectory planner (not used in final run)
│
├── cotask.py             — Cooperative priority task scheduler (JR Ridgely)
└── task_share.py         — Interrupt-safe Share and Queue primitives (JR Ridgely)
```

---

## 15. How to Run

### Requirements

- Pololu Romi chassis with STM32 Nucleo running MicroPython firmware
- BNO055 IMU breakout board wired to I2C bus 1 and reset pin H1
- HC-05 Bluetooth module on UART 5 at 230400 baud (USB fallback works for commands)
- 8-element IR sensor array on the ADC pins listed in Section 2

### Deploying

Copy all `.py` files to the root of the MicroPython filesystem. The board executes `main.py` automatically on power-up.

```bash
# Using mpremote (recommended):
mpremote connect /dev/ttyACM0 cp *.py :
```

### First boot (no calibration file)

```
No calibration file found. Starting gyro calibration...
GYRO : Set the robot on a flat surface and hold it PERFECTLY STILL
       Wait for GYRO to reach 3/3. This usually takes 5-10 seconds.
GYRO:0/3  (waiting for 3/3 -- hold still)
```

Hold the robot still until `GYRO:3/3` is displayed. `calibration.txt` is written automatically; all subsequent boots skip this step.

### Running the course

1. Connect a serial terminal at 230400 baud
2. Press `c` — place robot on **white surface** and press Enter
3. Press `c` again — place robot on the **black line** and press Enter
4. Position the robot at the course start, centred on the line and facing the correct direction
5. Press `y` to begin — press `x` at any time to abort and stop all motors

---

## 16. Video Demo

> 📹 **[(https://youtu.be/iQ_Vchzl95k)]**

---

*ME 405 — Mechatronics · Cal Poly SLO*
