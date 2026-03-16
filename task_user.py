# task_user.py

from pyb import USB_VCP, UART
from task_share import Share, Queue, BaseShare
import micropython

S0_INIT  = micropython.const(0)
S1_CMD   = micropython.const(1)
S2_COL   = micropython.const(2)
S3_DIS   = micropython.const(3)
S4_MCD   = micropython.const(4)
S5_DRIVE = micropython.const(5)
S6_CEN   = micropython.const(6)
S7_RUN   = micropython.const(7)   # ← NEW: full track run, master task in control

UI_prompt = ">: "


class task_user:
    '''
    UI task:
    - Able to switch between communitcation modes (USB for command input and debugging, Bluetooth for data streaming)
    - Menu system for entering new gains and set points, triggering step responses, calibrating line
        sensors, and toggling the state estimator on/off.
    - Press 'y' to hand control to the master task and run the full track.
      Press 'x' at any time to abort the run and stop all motors.
    '''

    def __init__(self, leftMotorGo, rightMotorGo, dataValues, timeValues,
                 leftSetPoint, rightSetPoint, leftKp, leftKi, leftKd, leftKff,
                 rightKp, rightKi, rightKd, rightKff, gainsUpdated,
                 calibrateIRFlag, lineGo, cen_data, cen_time,
                 estimatorGo: Share,
                 x_pos_hat: Share, y_pos_hat: Share, psi_hat: Share,
                 omL_hat: Share, omR_hat: Share, crash_detect: Queue,
                 masterGo: Share):

        self._state: int = S0_INIT

        self._leftMotorGo: Share  = leftMotorGo
        self._rightMotorGo: Share = rightMotorGo

        self._leftSetPoint: Share  = leftSetPoint
        self._rightSetPoint: Share = rightSetPoint

        self._leftKp:  Share = leftKp
        self._leftKi:  Share = leftKi
        self._leftKd:  Share = leftKd
        self._leftKff: Share = leftKff

        self._rightKp:  Share = rightKp
        self._rightKi:  Share = rightKi
        self._rightKd:  Share = rightKd
        self._rightKff: Share = rightKff

        self._gainsUpdated: Share = gainsUpdated
        self._estimatorGo:  Share = estimatorGo

        # Estimator output shares — used by 'p' command to verify the observer
        self._x_pos_hat: Share = x_pos_hat
        self._y_pos_hat: Share = y_pos_hat
        self._psi_hat:   Share = psi_hat
        self._omL_hat:   Share = omL_hat
        self._omR_hat:   Share = omR_hat

        # Master task control — 'y' sets this True to start the full track run
        self._masterGo: Share = masterGo   # ← NEW
        self._run_started: bool = False    # guard against immediate false-exit in S7_RUN

        # USB for UI/debug
        self._usb = USB_VCP()

        # Bluetooth data stream (HC-05 configured to 230400 baud)
        self._bt = UART(5, 230400, timeout=0)

        self._io = self._bt
        self._dataIO = self._usb

        # Share used to pass completed numeric input out of S4_MCD
        self._out_share: BaseShare = Share('f', name="A float share")

        self._char_buf: str = ""
        self._digits = "0123456789"
        self._term   = {"\r", "\n"}
        self._done: bool = False

        # Gain entry flags
        self._leftKpflag:    bool = False
        self._leftKiflag:    bool = False
        self._leftKdflag:    bool = False
        self._leftKffflag:   bool = False
        self._rightKpflag:   bool = False
        self._rightKiflag:   bool = False
        self._rightKdflag:   bool = False
        self._rightKffflag:  bool = False

        # Setpoint entry flags
        self._leftSPflag:  bool = False
        self._rightSPflag: bool = False

        # Test flags
        self._leftTestflag:  bool = False
        self._rightTestflag: bool = False

        # Line sensor
        self._calibrateIRFlag: Share = calibrateIRFlag
        self._lineGo: Share = lineGo
        self._calStep = 0

        self._dataValues: Queue = dataValues
        self._timeValues: Queue = timeValues
        self._cen_data:   Queue = cen_data
        self._cen_time:   Queue = cen_time

        # Bump sensor
        self._bumpFlag: Queue = crash_detect

        self._io.write("User Task object instantiated\r\n")

    # ──────────────────────────────────────────────────────────────────────────
    def _print_help(self):
        self._io.write(
        "+------------------------------------------------------------------------------+\r\n"
        "| ME 405 Romi Tuning Interface                                                 |\r\n"
        "+---+--------------------------------------------------------------------------+\r\n"
        "| h | Print this help menu                                                     |\r\n"
        "| k | Enter new gain values                                                    |\r\n"
        "| s | Choose a new setpoint                                                    |\r\n"
        "| l | Trigger left motor step response and print results                       |\r\n"
        "| r | Trigger right motor step response and print results                      |\r\n"
        "| c | Calibrate line sensor (place on white first, then black)                 |\r\n"
        "| d | Lock on to line and drive (press x to stop)                              |\r\n"
        "| e | Toggle state estimator ON / OFF                                          |\r\n"
        "| p | Print current estimator state (X, Y, heading, wheel speeds)             |\r\n"
        "| y | START full track run  (master task takes over; press x to abort)        |\r\n"  # ← NEW
        "+---+--------------------------------------------------------------------------+\r\n"
    )

    # ──────────────────────────────────────────────────────────────────────────
    def _reset_input_flags(self):
        self._leftKpflag   = False
        self._leftKiflag   = False
        self._leftKdflag   = False
        self._leftKffflag  = False
        self._rightKpflag  = False
        self._rightKiflag  = False
        self._rightKdflag  = False
        self._rightKffflag = False
        self._leftSPflag   = False
        self._rightSPflag  = False
        self._char_buf     = ""
        self._done         = False

    def _print_estimator_state(self):
        '''Print the current estimator outputs over USB.'''
        on = self._estimatorGo.get()
        status = "ON" if on else "OFF (press e to enable)"
        x   = self._x_pos_hat.get()
        y   = self._y_pos_hat.get()
        psi = self._psi_hat.get() * (180.0 / 3.14159)
        omL = self._omL_hat.get()
        omR = self._omR_hat.get()
        self._io.write("\r\n--- State Estimator [{}] ---\r\n".format(status))
        self._io.write("  X pos  : {:.1f} mm\r\n".format(x))
        self._io.write("  Y pos  : {:.1f} mm\r\n".format(y))
        self._io.write("  Heading: {:.1f} deg\r\n".format(psi))
        self._io.write("  omL    : {:.2f} rad/s\r\n".format(omL))
        self._io.write("  omR    : {:.2f} rad/s\r\n".format(omR))
        self._io.write("-----------------------------------\r\n")

    def _stop_full_run(self):
        '''Stop everything and return to idle menu. Used by abort (x) in S7_RUN.'''
        self._masterGo.put(False)
        self._leftMotorGo.put(False)
        self._rightMotorGo.put(False)
        self._lineGo.put(False)
        self._io.write("\r\nRun aborted. All motors stopped.\r\n")
        self._print_help()
        self._io.write(UI_prompt)

    # ──────────────────────────────────────────────────────────────────────────
    def run(self):
        while True:

            # ── S0: boot print ────────────────────────────────────────────────
            if self._state == S0_INIT:
                self._io.write("Initializing user task\r\n")
                self._print_help()
                self._io.write(UI_prompt)
                self._state = S1_CMD

            # ── S1: command dispatch ──────────────────────────────────────────
            elif self._state == S1_CMD:
                if self._io.any():
                    inChar = (self._io.read(1) or b"").decode()

                    if inChar in {"h", "H"}:
                        self._print_help()
                        self._io.write(UI_prompt)

                    elif inChar in {"k", "K"}:
                        self._io.write("Entering new gain values...\r\n")
                        self._leftKpflag = True
                        self._io.write("Enter left Kp value:\r\n")
                        self._state = S4_MCD

                    elif inChar in {"s", "S"}:
                        self._io.write("Entering new set point values...\r\n")
                        self._leftSPflag = True
                        self._io.write("Enter left motor set point value:\r\n")
                        self._state = S4_MCD

                    elif inChar in {"l", "L"}:
                        self._io.write("{}\r\n".format(inChar))
                        self._leftMotorGo.put(True)
                        self._leftTestflag = True
                        self._gainsUpdated.put(True)
                        self._io.write("Starting left motor step response...\r\n")
                        self._io.write("Please wait...\r\n")
                        self._state = S2_COL

                    elif inChar in {"r", "R"}:
                        self._io.write("{}\r\n".format(inChar))
                        self._rightMotorGo.put(True)
                        self._rightTestflag = True
                        self._gainsUpdated.put(True)
                        self._io.write("Starting right motor step response...\r\n")
                        self._io.write("Please wait...\r\n")
                        self._state = S2_COL

                    elif inChar in {"c", "C"}:
                        self._io.write("{}\r\n".format(inChar))
                        self._calibrateIRFlag.put(True)
                        if self._calStep == 0:
                            self._io.write("White calibration done. Place on black and press 'c' again.\r\n")
                            self._calStep = 1
                        else:
                            self._io.write("Black calibration done. Ready to drive.\r\n")
                            self._calStep = 0
                        self._io.write(UI_prompt)

                    elif inChar in {"d", "D"}:
                        self._io.write("{}\r\n".format(inChar))
                        self._leftMotorGo.put(True)
                        self._rightMotorGo.put(True)
                        self._lineGo.put(True)
                        self._io.write("Line following started. Press 'x' to stop, 'p' to print position.\r\n")
                        self._state = S5_DRIVE

                    elif inChar in {"e", "E"}:
                        currently_on = self._estimatorGo.get()
                        self._estimatorGo.put(not currently_on)
                        if not currently_on:
                            self._io.write("State estimator ON. Press 'p' to print estimated state.\r\n")
                        else:
                            self._io.write("State estimator OFF.\r\n")
                        self._io.write(UI_prompt)

                    elif inChar in {"p", "P"}:
                        self._print_estimator_state()
                        self._io.write(UI_prompt)

                    elif inChar in {"y", "Y"}:          # ── NEW ──
                        self._io.write("{}\r\n".format(inChar))
                        # master task's _start_run() arms motors, estimator,
                        # and line following — do NOT set them here to avoid
                        # a race where the estimator Go flag is True but the
                        # estimator hasn't reset from a previous run yet.
                        self._masterGo.put(True)
                        self._run_started = False   # will be set True next tick
                        self._io.write("GO! Master task running. Press 'x' to abort.\r\n")
                        self._state = S7_RUN

                elif self._bumpFlag.any():
                    self._bumpFlag.get()
                    self._io.write("Bump detected! Flag raised.\r\n")
                    self._io.write(UI_prompt)

            # ── S2: wait for motor data collection to finish ──────────────────
            elif self._state == S2_COL:
                if self._io.any():
                    self._io.read(1)  # discard input during collection

                if not self._leftMotorGo.get() and not self._rightMotorGo.get():
                    self._io.write("Data collection complete. Sending over Bluetooth...\r\n")

                    if self._leftTestflag:
                        self._io.write("Left SP={} Kp={} Ki={} Kd={} Kff={}\r\n".format(
                            self._leftSetPoint.get(), self._leftKp.get(),
                            self._leftKi.get(), self._leftKd.get(), self._leftKff.get()))
                        self._leftTestflag = False

                    if self._rightTestflag:
                        self._io.write("Right SP={} Kp={} Ki={} Kd={} Kff={}\r\n".format(
                            self._rightSetPoint.get(), self._rightKp.get(),
                            self._rightKi.get(), self._rightKd.get(), self._rightKff.get()))
                        self._rightTestflag = False

                    self._dataIO.write("Time [ms], Velocity [mm/s]\r\n")
                    self._state = S3_DIS

            # ── S3: stream data over Bluetooth ────────────────────────────────
            elif self._state == S3_DIS:
                BATCH_LINES = 25
                batch = ""
                n = 0
                while self._dataValues.any() and self._timeValues.any():
                    batch += "{},{}\r\n".format(self._timeValues.get(), self._dataValues.get())
                    n += 1
                    if n >= BATCH_LINES:
                        self._dataIO.write(batch)
                        batch = ""
                        n = 0
                if batch:
                    self._dataIO.write(batch)
                if not self._dataValues.any():
                    self._dataIO.write("--------------------\r\n")
                    self._io.write("Data sent.\r\n")
                    self._print_help()
                    self._io.write(UI_prompt)
                    self._state = S1_CMD

            # ── S4: multi-character numeric input ─────────────────────────────
            elif self._state == S4_MCD:
                if not self._done:
                    if self._io.any():
                        char_in = (self._io.read(1) or b"").decode()

                        if char_in in self._digits:
                            self._io.write(char_in)
                            self._char_buf += char_in

                        elif char_in == "." and "." not in self._char_buf:
                            self._io.write(char_in)
                            self._char_buf += char_in

                        elif char_in == "-" and len(self._char_buf) == 0:
                            self._io.write(char_in)
                            self._char_buf += char_in

                        elif char_in == "\x7f" and len(self._char_buf) > 0:
                            self._io.write("\x7f")
                            self._char_buf = self._char_buf[:-1]

                        elif char_in in self._term:
                            if len(self._char_buf) == 0:
                                self._io.write("\r\nNo value entered. Returning to menu.\r\n")
                                self._reset_input_flags()
                                self._print_help()
                                self._io.write(UI_prompt)
                                self._state = S1_CMD
                            elif self._char_buf not in {"-", "."}:
                                self._io.write("\r\n")
                                self._out_share.put(float(self._char_buf))
                                self._char_buf = ""
                                self._done = True

                    yield self._state
                    continue

                new_val = self._out_share.get()
                self._done = False

                if self._leftKpflag:
                    self._leftKp.put(new_val);  self._leftKpflag = False
                    self._leftKiflag = True
                    self._io.write("Left Kp = {}\r\nEnter left Ki:\r\n".format(new_val))
                    self._state = S4_MCD

                elif self._leftKiflag:
                    self._leftKi.put(new_val);  self._leftKiflag = False
                    self._leftKdflag = True
                    self._io.write("Left Ki = {}\r\nEnter left Kd:\r\n".format(new_val))
                    self._state = S4_MCD

                elif self._leftKdflag:
                    self._leftKd.put(new_val);  self._leftKdflag = False
                    self._leftKffflag = True
                    self._io.write("Left Kd = {}\r\nEnter left Kff:\r\n".format(new_val))
                    self._state = S4_MCD

                elif self._leftKffflag:
                    self._leftKff.put(new_val); self._leftKffflag = False
                    self._rightKpflag = True
                    self._io.write("Left Kff = {}\r\nEnter right Kp:\r\n".format(new_val))
                    self._state = S4_MCD

                elif self._rightKpflag:
                    self._rightKp.put(new_val); self._rightKpflag = False
                    self._rightKiflag = True
                    self._io.write("Right Kp = {}\r\nEnter right Ki:\r\n".format(new_val))
                    self._state = S4_MCD

                elif self._rightKiflag:
                    self._rightKi.put(new_val); self._rightKiflag = False
                    self._rightKdflag = True
                    self._io.write("Right Ki = {}\r\nEnter right Kd:\r\n".format(new_val))
                    self._state = S4_MCD

                elif self._rightKdflag:
                    self._rightKd.put(new_val); self._rightKdflag = False
                    self._rightKffflag = True
                    self._io.write("Right Kd = {}\r\nEnter right Kff:\r\n".format(new_val))
                    self._state = S4_MCD

                elif self._rightKffflag:
                    self._rightKff.put(new_val); self._rightKffflag = False
                    self._gainsUpdated.put(True)
                    self._io.write("Right Kff = {}\r\nAll gains updated.\r\n".format(new_val))
                    self._print_help()
                    self._io.write(UI_prompt)
                    self._state = S1_CMD

                elif self._leftSPflag:
                    self._leftSetPoint.put(new_val); self._leftSPflag = False
                    self._rightSPflag = True
                    self._io.write("Left SP = {}\r\nEnter right SP:\r\n".format(new_val))
                    self._state = S4_MCD

                elif self._rightSPflag:
                    self._rightSetPoint.put(new_val); self._rightSPflag = False
                    self._io.write("Right SP = {}\r\n".format(new_val))
                    self._print_help()
                    self._io.write(UI_prompt)
                    self._state = S1_CMD

            # ── S5: manual line following — wait for stop command ─────────────
            elif self._state == S5_DRIVE:
                if self._io.any():
                    inChar = (self._io.read(1) or b"").decode()
                    if inChar in {"x", "X"}:
                        self._leftMotorGo.put(False)
                        self._rightMotorGo.put(False)
                        self._lineGo.put(False)
                        self._io.write("Stopped. Sending centroid data...\r\n")
                        self._dataIO.write("Time [ms], Centroid\r\n")
                        self._state = S6_CEN
                    elif inChar in {"p", "P"}:
                        self._print_estimator_state()

            # ── S6: stream centroid data over Bluetooth ───────────────────────
            elif self._state == S6_CEN:
                if self._cen_data.any() and self._cen_time.any():
                    batch = ""
                    n = 0
                    while self._cen_data.any() and self._cen_time.any() and n < 5:
                        batch += "{},{}\r\n".format(self._cen_time.get(), self._cen_data.get())
                        n += 1
                    self._dataIO.write(batch)
                elif not self._cen_data.any():
                    self._dataIO.write("--------------------\r\n")
                    self._io.write("Data sent.\r\n")
                    self._print_help()
                    self._io.write(UI_prompt)
                    self._state = S1_CMD

            # ── S7: full track run — master task is in control ────────────────
            elif self._state == S7_RUN:                         # ← NEW STATE
                if self._io.any():
                    inChar = (self._io.read(1) or b"").decode()
                    if inChar in {"x", "X"}:
                        # User abort — kill everything immediately
                        self._stop_full_run()
                        self._state = S1_CMD
                    elif inChar in {"p", "P"}:
                        # Allow position peek during run without interrupting it
                        self._print_estimator_state()

                # Master task clears masterGo when the course is complete
                # _run_started prevents a false trigger on the very first tick
                self._run_started = self._run_started or self._masterGo.get()
                if self._run_started and not self._masterGo.get():
                    self._leftMotorGo.put(False)
                    self._rightMotorGo.put(False)
                    self._io.write("\r\nCourse complete! Motors stopped.\r\n")
                    self._print_help()
                    self._io.write(UI_prompt)
                    self._state = S1_CMD

            yield self._state