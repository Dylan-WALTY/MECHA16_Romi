"""
main.py — Romi top-level entry point.

Initialises all hardware, creates inter-task shared variables and queues,
instantiates task objects, and runs the cooperative priority scheduler.
The IMU is polled directly in the main loop rather than inside a task so
that I2C retry delays (up to 50 ms) cannot starve other tasks.
"""

from pyb import Pin, Timer, I2C
from gc import collect
from utime import sleep_ms

from cotask      import Task, TaskList
from task_share  import Share, Queue, show_all

from motor_driver       import Motor
from encoder_driver     import Encoder
from closed_loop_driver import closed_loop_control
from line_sensor_driver import line_sensor
from imu_driver         import BNO055
collect()

from task_motor     import task_motor
from task_user      import task_user
collect()
from task_line      import task_line
from task_estimator import task_observer
collect()
from task_bump   import task_crash
from task_master import task_master
collect()

_DEG_TO_RAD = 3.14159 / 180.0
_PSI_SIGN   = -1   # BNO055 heading increases CW; observer uses CCW convention

task_list = TaskList()

# ── Hardware initialisation ────────────────────────────────────────────────────

pwm_tim = Timer(3, freq=20000)

leftMotor  = Motor(Pin.cpu.B1, Pin.cpu.B13, Pin.cpu.B14, pwm_tim, 4)
rightMotor = Motor(Pin.cpu.B0, Pin.cpu.B7,  Pin.cpu.H0,  pwm_tim, 3)
leftMotor.enable()
rightMotor.enable()

leftEncoder  = Encoder(2, Pin.cpu.A0, Pin.cpu.A1)
rightEncoder = Encoder(1, Pin.cpu.A8, Pin.cpu.A9)

ir_pins   = [Pin.cpu.A6, Pin.cpu.A7, Pin.cpu.C5, Pin.cpu.C0,
             Pin.cpu.C1, Pin.cpu.A4, Pin.cpu.C3, Pin.cpu.C2]
ir_sensor = line_sensor(ir_pins, Pin.cpu.B15)

i2c_bus = I2C(1, I2C.MASTER, baudrate=100000)
imu     = BNO055(i2c_bus, Pin.cpu.H1)

bump_pins = [Pin.cpu.B3, Pin.cpu.B5, Pin.cpu.B4]

# ── Inter-task shared variables ────────────────────────────────────────────────

leftMotorGo  = Share("B", name="Left Motor Go");  leftMotorGo.put(False)
rightMotorGo = Share("B", name="Right Motor Go"); rightMotorGo.put(False)

leftSetPoint  = Share("f", name="Left SP");  leftSetPoint.put(100.0)
rightSetPoint = Share("f", name="Right SP"); rightSetPoint.put(100.0)

# Motor PID gains
leftKp  = Share("f", name="Left Kp");  leftKp.put(0.037)
leftKi  = Share("f", name="Left Ki");  leftKi.put(0.01)
leftKd  = Share("f", name="Left Kd");  leftKd.put(0.0)
leftKff = Share("f", name="Left Kff"); leftKff.put(0.096)

rightKp  = Share("f", name="Right Kp");  rightKp.put(0.037)
rightKi  = Share("f", name="Right Ki");  rightKi.put(0.01)
rightKd  = Share("f", name="Right Kd");  rightKd.put(0.0)
rightKff = Share("f", name="Right Kff"); rightKff.put(0.096)

# Line sensor PID gains
lineKp  = Share("f", name="Line Kp");  lineKp.put(34.0)
lineKi  = Share("f", name="Line Ki");  lineKi.put(0.1)
lineKd  = Share("f", name="Line Kd");  lineKd.put(0.0)
lineKff = Share("f", name="Line Kff"); lineKff.put(0.0)

steeringCorrection = Share("f", name="Steering Correction"); steeringCorrection.put(0.0)
baseSpeed          = Share("f", name="Base Speed");          baseSpeed.put(150.0)

# Control flow flags
calibrateIRFlag = Share("B", name="Cal IR");       calibrateIRFlag.put(False)
lineGo          = Share("B", name="Line Go");       lineGo.put(False)
gainsUpdated    = Share("B", name="Gains Updated"); gainsUpdated.put(False)

# Step-test and centroid data buffers
dataValues = Queue("f", 30,  name="Data Buffer")
timeValues = Queue("L", 30,  name="Time Buffer")
cen_data   = Queue("f", 100, name="Centroid Data")
cen_time   = Queue("L", 100, name="Centroid Time")

# IMU and estimator control
imu_ready   = Share("B", name="IMU Ready");    imu_ready.put(False)
estimatorGo = Share("B", name="Estimator Go"); estimatorGo.put(False)

crash_detect = Queue("B", 3, name="Bump Flag")

# Trajectory shares (unused in final run; kept so task_user compiles unchanged)
trajectoryGo   = Share("B", name="Traj Go");   trajectoryGo.put(False)
trajectoryDone = Share("B", name="Traj Done"); trajectoryDone.put(False)
masterGo       = Share("B", name="Master Go"); masterGo.put(False)

# Line detection shares for master task
centroid_share = Share("f", name="Centroid");      centroid_share.put(0.0)
line_detected  = Share("B", name="Line Detected"); line_detected.put(False)

_CAL_FILE = "calibration.txt"

# ── IMU startup and calibration ────────────────────────────────────────────────

if imu.load_and_apply_calibration(_CAL_FILE):
    print("Calibration loaded. Switching to fusion mode...")
    imu.mode_fusion(0x08)
    sleep_ms(500)
else:
    print("No calibration file found. Starting gyro calibration...")
    imu.mode_fusion(0x08)
    sleep_ms(500)
    imu.manual_calibrate(check_mag=False)
    imu.save_calibration(_CAL_FILE)
    sleep_ms(500)
    print("Calibration complete and saved.")

_imu_heading_offset = imu.get_yaw()
imu_ready.put(True)

# ── Observer input shares ──────────────────────────────────────────────────────

left_arc    = Share("f", name="Left Arc");    left_arc.put(0.0)
right_arc   = Share("f", name="Right Arc");   right_arc.put(0.0)
left_volts  = Share("f", name="Left Volts");  left_volts.put(0.0)
right_volts = Share("f", name="Right Volts"); right_volts.put(0.0)

psi_share  = Share("f", name="IMU Heading");  psi_share.put(0.0)
dpsi_share = Share("f", name="IMU Yaw Rate"); dpsi_share.put(0.0)

# Observer output shares
s_hat     = Share("f", name="Est. Disp");        s_hat.put(0.0)
psi_hat   = Share("f", name="Est. Heading");      psi_hat.put(0.0)
omL_hat   = Share("f", name="Est. Left Speed");   omL_hat.put(0.0)
omR_hat   = Share("f", name="Est. Right Speed");  omR_hat.put(0.0)
x_pos_hat = Share("f", name="Est. X Pos");        x_pos_hat.put(0.0)
y_pos_hat = Share("f", name="Est. Y Pos");        y_pos_hat.put(0.0)

# ── Task instantiation ─────────────────────────────────────────────────────────

leftMotorTask = task_motor(
    leftMotor, leftEncoder,
    closed_loop_control(leftKp.get(), leftKi.get(), leftKd.get(), leftKff.get()),
    leftMotorGo, dataValues, timeValues,
    leftSetPoint, leftKp, leftKi, leftKd, leftKff, gainsUpdated,
    steeringCorrection, baseSpeed, lineGo,
    correctionSign=-1, arc_share=left_arc, volts_share=left_volts,
    trajectoryGo=trajectoryGo
)

rightMotorTask = task_motor(
    rightMotor, rightEncoder,
    closed_loop_control(rightKp.get(), rightKi.get(), rightKd.get(), rightKff.get()),
    rightMotorGo, dataValues, timeValues,
    rightSetPoint, rightKp, rightKi, rightKd, rightKff, gainsUpdated,
    steeringCorrection, baseSpeed, lineGo,
    correctionSign=1, arc_share=right_arc, volts_share=right_volts,
    trajectoryGo=trajectoryGo
)

userTask = task_user(
    leftMotorGo, rightMotorGo, dataValues, timeValues,
    leftSetPoint, rightSetPoint,
    leftKp, leftKi, leftKd, leftKff,
    rightKp, rightKi, rightKd, rightKff,
    gainsUpdated, calibrateIRFlag, lineGo, cen_data, cen_time,
    estimatorGo,
    x_pos_hat, y_pos_hat, psi_hat, omL_hat, omR_hat, crash_detect, masterGo,
    trajectoryGo=trajectoryGo
)

lineSensorTask = task_line(
    ir_sensor,
    closed_loop_control(lineKp.get(), lineKi.get(), lineKd.get(), lineKff.get()),
    calibrateIRFlag, steeringCorrection, baseSpeed, lineGo, cen_data, cen_time,
    centroid_share=centroid_share,
    line_detected_share=line_detected
)

estimatorTask = task_observer(
    left_arc, right_arc, left_volts, right_volts,
    psi_share, dpsi_share, imu_ready, estimatorGo,
    s_hat, psi_hat, omL_hat, omR_hat, x_pos_hat, y_pos_hat
)

bumpTask = task_crash(crash_detect, bump_pins)

masterTask = task_master(
    lineGo         = lineGo,
    trajectoryGo   = trajectoryGo,
    trajectoryDone = trajectoryDone,
    crash_detect   = crash_detect,
    x_pos_hat      = x_pos_hat,
    y_pos_hat      = y_pos_hat,
    masterGo       = masterGo,
    trajTask       = None,
    leftMotorGo    = leftMotorGo,
    rightMotorGo   = rightMotorGo,
    leftSetPoint   = leftSetPoint,
    rightSetPoint  = rightSetPoint,
    psi_hat        = psi_hat,
    line_detected  = line_detected,
    estimatorGo    = estimatorGo,
    left_encoder   = leftEncoder,
    right_encoder  = rightEncoder,
    bump_pins      = bump_pins,
)

# ── Task list ──────────────────────────────────────────────────────────────────

task_list.append(Task(leftMotorTask.run,  name="Left Mot.",   priority=1, period=50,  profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot.",  priority=1, period=50,  profile=True))
task_list.append(Task(userTask.run,       name="User Int.",   priority=0, period=0,   profile=False))
task_list.append(Task(lineSensorTask.run, name="Line Sensor", priority=2, period=50,  profile=False))
task_list.append(Task(estimatorTask.run,  name="State Est.",  priority=3, period=20,  profile=False))
task_list.append(Task(bumpTask.run,       name="Bump Sensor", priority=1, period=40,  profile=False))
task_list.append(Task(masterTask.run,     name="Master",      priority=1, period=100, profile=False))

collect()

# ── Main scheduler loop ────────────────────────────────────────────────────────

while True:
    try:
        # Poll IMU outside the task scheduler to avoid blocking other tasks
        # during I2C retry delays. Apply sign correction and convert to radians.
        heading_deg  = imu.get_yaw() - _imu_heading_offset
        yaw_rate_dps = imu.get_yaw_rate()
        psi_share.put(_PSI_SIGN  * heading_deg  * _DEG_TO_RAD)
        dpsi_share.put(_PSI_SIGN * yaw_rate_dps * _DEG_TO_RAD)
        task_list.pri_sched()

    except OSError:
        # I2C glitch — skip IMU update this tick and continue scheduling
        task_list.pri_sched()

    except KeyboardInterrupt:
        print("Terminating.")
        leftMotor.disable()
        rightMotor.disable()
        break

print("\n")
print(task_list)
print(show_all())
