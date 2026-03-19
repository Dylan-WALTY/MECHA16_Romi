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
S7_RUN   = micropython.const(7)

UI_prompt = ">: "


class task_user:
    """Serial terminal interface for tuning, calibration, and run control.

    Communicates over Bluetooth (UART 5 at 230400 baud) for commands and
    over USB VCP for data output. Exposes a single-character command menu
    and a multi-character numeric input sub-state for entering gain and
    setpoint values.

    States
    ------
    S0_INIT  : Print boot message and help menu, then move to S1_CMD.
    S1_CMD   : Wait for a single command character and dispatch.
    S2_COL   : Poll until motor data collection completes.
    S3_DIS   : Stream velocity data to USB in batches of 25 lines.
    S4_MCD   : Accept multi-character numeric input (gains or setpoints).
    S5_DRIVE : Manual line-follow active — wait for 'x' to stop.
    S6_CEN   : Stream centroid data to USB after a manual drive.
    S7_RUN   : Full autonomous run active — watch for 'x' abort or completion.
    """

    def __init__(self, leftMotorGo, rightMotorGo, dataValues, timeValues,
                 leftSetPoint, rightSetPoint, leftKp, leftKi, leftKd, leftKff,
                 rightKp, rightKi, rightKd, rightKff, gainsUpdated,
                 calibrateIRFlag, lineGo, cen_data, cen_time,
                 estimatorGo: Share,
                 x_pos_hat: Share, y_pos_hat: Share, psi_hat: Share,
                 omL_hat: Share, omR_hat: Share, crash_detect: Queue,
                 masterGo: Share,
                 trajectoryGo: Share = None):
        """Bind all inter-task shares and open serial ports."""
        self._state: int = S0_INIT

        self._leftMotorGo:  Share = leftMotorGo
        self._rightMotorGo: Share = rightMotorGo
        self._leftSetPoint:  Share = leftSetPoint
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

        self._x_pos_hat: Share = x_pos_hat
        self._y_pos_hat: Share = y_pos_hat
        self._psi_hat:   Share = psi_hat
        self._omL_hat:   Share = omL_hat
        self._omR_hat:   Share = omR_hat

        self._masterGo:     Share = masterGo
        self._trajectoryGo: Share = trajectoryGo
        self._lineGo:       Share = lineGo

        self._run_started: bool = False

        self._usb = USB_VCP()
        self._bt  = UART(5, 230400, timeout=0)
        self._io  = self._bt      # commands via Bluetooth
        self._dataIO = self._usb  # data dumps via USB

        self._out_share: BaseShare = Share('f', name="A float share")
        self._char_buf:  str  = ""
        self._digits          = "0123456789"
        self._term            = {"\r", "\n"}
        self._done:      bool = False

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
        self._leftTestflag  = False
        self._rightTestflag = False

        self._calibrateIRFlag: Share = calibrateIRFlag
        self._calStep = 0

        self._dataValues: Queue = dataValues
        self._timeValues: Queue = timeValues
        self._cen_data:   Queue = cen_data
        self._cen_time:   Queue = cen_time
        self._bumpFlag:   Queue = crash_detect

        self._io.write("User Task object instantiated\r\n")

    def _print_help(self):
        """Print the command reference table to the serial terminal."""
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
            "| y | START full track run  (master task takes over; press x to abort)        |\r\n"
            "+---+--------------------------------------------------------------------------+\r\n"
        )

    def _reset_input_flags(self):
        """Clear all gain/setpoint input flags and the character buffer."""
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
        """Print current X, Y, heading, and wheel speed estimates to the terminal."""
        on     = self._estimatorGo.get()
        status = "ON" if on else "OFF (press e to enable)"
        x      = self._x_pos_hat.get()
        y      = self._y_pos_hat.get()
        psi    = self._psi_hat.get() * (180.0 / 3.14159)
        omL    = self._omL_hat.get()
        omR    = self._omR_hat.get()
        self._io.write("\r\n--- State Estimator [{}] ---\r\n".format(status))
        self._io.write("  X pos  : {:.1f} mm\r\n".format(x))
        self._io.write("  Y pos  : {:.1f} mm\r\n".format(y))
        self._io.write("  Heading: {:.1f} deg\r\n".format(psi))
        self._io.write("  omL    : {:.2f} rad/s\r\n".format(omL))
        self._io.write("  omR    : {:.2f} rad/s\r\n".format(omR))
        self._io.write("-----------------------------------\r\n")

    def _stop_full_run(self):
        """Hard-stop all motors and reset every autonomous-control share."""
        self._masterGo.put(False)
        self._lineGo.put(False)
        if self._trajectoryGo is not None:
            self._trajectoryGo.put(False)
        self._leftSetPoint.put(0.0)
        self._rightSetPoint.put(0.0)
        self._leftMotorGo.put(False)
        self._rightMotorGo.put(False)
        self._estimatorGo.put(False)
        self._run_started = False
        self._io.write("\r\nRun aborted. All motors stopped.\r\n")
        self._print_help()
        self._io.write(UI_prompt)

    def run(self):
        while True:

            # S0: one-time boot message
            if self._state == S0_INIT:
                self._io.write("Initializing user task\r\n")
                self._print_help()
                self._io.write(UI_prompt)
                self._state = S1_CMD

            # S1: wait for and dispatch a single command character
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
                        self._io.write("Starting left motor step response...\r\nPlease wait...\r\n")
                        self._state = S2_COL

                    elif inChar in {"r", "R"}:
                        self._io.write("{}\r\n".format(inChar))
                        self._rightMotorGo.put(True)
                        self._rightTestflag = True
                        self._gainsUpdated.put(True)
                        self._io.write("Starting right motor step response...\r\nPlease wait...\r\n")
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
                        self._io.write("Line following started. Press 'x' to stop.\r\n")
                        self._state = S5_DRIVE

                    elif inChar in {"e", "E"}:
                        currently_on = self._estimatorGo.get()
                        self._estimatorGo.put(not currently_on)
                        self._io.write("State estimator {}.\r\n".format("ON" if not currently_on else "OFF"))
                        self._io.write(UI_prompt)

                    elif inChar in {"p", "P"}:
                        self._print_estimator_state()
                        self._io.write(UI_prompt)

                    elif inChar in {"y", "Y"}:
                        self._io.write("{}\r\n".format(inChar))
                        self._run_started = False
                        self._masterGo.put(True)
                        self._io.write("GO! Master task running. Press 'x' to abort.\r\n")
                        self._state = S7_RUN

                elif self._bumpFlag.any():
                    self._bumpFlag.get()
                    self._io.write("Bump detected!\r\n")
                    self._io.write(UI_prompt)

            # S2: wait for step-test data collection to finish
            elif self._state == S2_COL:
                if self._io.any():
                    self._io.read(1)

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

            # S3: stream step-test data to USB in batches of 25 lines
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

            # S4: multi-character numeric input for gains and setpoints
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

                new_val    = self._out_share.get()
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

            # S5: manual line-follow active — wait for 'x' to stop
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

            # S6: stream centroid data to USB (5 lines per tick)
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

            # S7: autonomous run — watch for 'x' abort or master completion
            elif self._state == S7_RUN:
                if self._io.any():
                    inChar = (self._io.read(1) or b"").decode()
                    if inChar in {"x", "X"}:
                        self._stop_full_run()
                        self._state = S1_CMD
                        yield self._state
                        continue
                    elif inChar in {"p", "P"}:
                        self._print_estimator_state()

                self._run_started = self._run_started or self._masterGo.get()
                if self._run_started and not self._masterGo.get():
                    self._leftMotorGo.put(False)
                    self._rightMotorGo.put(False)
                    self._run_started = False
                    self._io.write("\r\nCourse complete! Motors stopped.\r\n")
                    self._print_help()
                    self._io.write(UI_prompt)
                    self._state = S1_CMD

            yield self._state
