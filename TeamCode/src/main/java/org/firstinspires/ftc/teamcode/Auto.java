/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import ftclib.*;
import hallib.*;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto", group="Pushbot")
//@Disabled
public class Auto extends LinearOpMode implements FtcMenu.MenuButtons {
    public enum Alliance {
        ALLIANCE_RED,
        ALLIANCE_BLUE
    }
    public enum StartPosition {
        STARTPOSITION1,
        STARTPOSITION2
    }
    public enum EndPosition {
        ENDCENTER,
        ENDCORNER,
        ENDNONE
    }
    public enum RunMode {
        RUNMODE_AUTO,
        RUNMODE_DEBUG
    }
    // Menu option variables
    RunMode runmode = RunMode.RUNMODE_AUTO;
    Alliance alliance = Alliance.ALLIANCE_RED;
    int delay = 0;
    StartPosition startposition = StartPosition.STARTPOSITION1;
    int balls = 2;
    int beacon = 2;
    EndPosition endposition = EndPosition.ENDCENTER;


    /* Declare OpMode members. */
    private HalDashboard dashboard;
    OurHardware         robot   = new OurHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    ModernRoboticsI2cGyro gyro    = null;                    // Additional Gyro device
    ModernRoboticsI2cColorSensor color = null;
    ModernRoboticsAnalogOpticalDistanceSensor ods = null;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // Neverrest 40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED_NORMAL      = 0.25;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.25;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.1;     // Larger is more responsive, but also less stable

    static final double     FORWARD_SPEED = 0.6;

    static final boolean    FIND_LINE_TRUE = true;
    static final boolean    FIND_LINE_FALSE = false;

    boolean runLonger = false;
    boolean longLaunch = false;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        dashboard = new HalDashboard(telemetry);

        // Init Gyro -------------------------------------------------------------------------------
        dashboard.displayPrintf(0,"Status: Calibrating Gyro");    //
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (gyro.isCalibrating())  {
            sleep(50);
            idle();
        }
        gyro.resetZAxisIntegrator();

        // Init Color Sensor -----------------------------------------------------------------------
        dashboard.displayPrintf(0,"Status: Init Color Sensor");
        color = (ModernRoboticsI2cColorSensor)hardwareMap.colorSensor.get("color");
        color.enableLed(false);

        // Init ods --------------------------------------------------------------------------------
        ods = (ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods");
        ods.enableLed(true);

        // Init presser ----------------------------------------------------------------------------
        robot.beaconpusher.setPosition(0.1);

        robot.launchingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launchingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.launchingMotor.setTargetPosition(0);
        robot.launchingMotor.setPower(0.0);

        // Wait for the game to start (driver presses PLAY)
        doMenus();
        dashboard.displayPrintf(0,"Status: Ready to start");
        dashboard.displayPrintf(3,"Gyro  %d", gyro.getIntegratedZValue());
        dashboard.displayPrintf(4,"Red   %d", color.red());
        dashboard.displayPrintf(5,"Green %d", color.green());
        dashboard.displayPrintf(6,"Blue  %d", color.blue());

        waitForStart();

        // AUTONOMOUS START-------------------------------------------------------------------------

        sleep(delay);
        if (beacon < 2) {
            longLaunch = true;
        }
        if (startposition == StartPosition.STARTPOSITION1) {
            if (DoTask("Setup Ball Launch", runmode)) {
                if (alliance == Alliance.ALLIANCE_BLUE && startposition == StartPosition.STARTPOSITION1)
                    DriveRobotPosition(.25, 30, FIND_LINE_FALSE);//really should be (-.25, 34, 0)
                else if (alliance == Alliance.ALLIANCE_RED && startposition == StartPosition.STARTPOSITION1)
                    DriveRobotPosition(-.25, -25, FIND_LINE_FALSE);//really should be (-.25, -34, 0)
            }
            if (DoTask("Ball Launch", runmode))
                BallLaunch(balls);

            // go to beacon
            if (DoTask("Go to beacon 1", runmode)) {
                if (beacon > 0) {
                    if (alliance == Alliance.ALLIANCE_BLUE && startposition == StartPosition.STARTPOSITION1) {
                        DriveRobotPosition(.5, 20, FIND_LINE_FALSE);
                        DriveTurngyro(0.25, 45.0);
                        DriveRobotPosition(.25, 3, FIND_LINE_FALSE);
                        DriveSidewaysTime(1000, .5);
                        DriveRobothug(0.15, 20, FIND_LINE_TRUE);
                    } else if (alliance == Alliance.ALLIANCE_RED && startposition == StartPosition.STARTPOSITION1) {
                        DriveRobotPosition(-.5, -27, FIND_LINE_FALSE);
                        DriveTurngyro(0.25, -45.0);
                        DriveRobotPosition(-.25, -3, FIND_LINE_FALSE);
                        DriveSidewaysTime(1000, .5);
                        DriveRobothug(-0.15, -20, FIND_LINE_TRUE);
                    }
                }
            }

            if (DoTask("Beacon 1 Push", runmode)) {
                BeaconPress(1);
            }

            if (DoTask("Drive to Beacon 2", runmode)) {
                if (beacon == 2) {
                    if (alliance == Alliance.ALLIANCE_BLUE) {
                        if (runLonger) {
                            DriveRobothug(.5, 49, FIND_LINE_FALSE);
                        } else {
                            DriveRobothug(.5, 42, FIND_LINE_FALSE);
                        }
                        DriveSidewaysTime(500, .5);
                        DriveRobothug(.15, 25, FIND_LINE_TRUE);
                    } else {
                        DriveRobothug(-.5, -35, FIND_LINE_FALSE);
                        DriveSidewaysTime(500, .5);
                        DriveRobothug(-.15, -25, FIND_LINE_TRUE);
                    }

                    if (DoTask("Beacon 2 Push", runmode)) {
                        BeaconPress(2);
                    }
                }
            }
            if (alliance == Alliance.ALLIANCE_RED) {
                if (endposition == EndPosition.ENDCORNER && beacon == 2) {
                    DriveSidewaysTime(1000, -.5);
                    DriveRobotPosition(.5, 80, FIND_LINE_FALSE);
                } else if (endposition == EndPosition.ENDCORNER && beacon == 1) {
                    DriveSidewaysTime(1000, -.5);
                    DriveRobotPosition(.5, 48, FIND_LINE_FALSE);
                } else if (endposition == EndPosition.ENDCENTER && beacon == 2) {
                    DriveSidewaysTime(1000, -.5);
                    DriveTurngyro(0.5, -13.0);
                    DriveRobotPosition(.6, 52, FIND_LINE_FALSE);
                } else if (endposition == EndPosition.ENDCENTER && beacon == 1) {
                    DriveSidewaysTime(2000, -.5);
                    DriveTurngyro(0.5, 0.0);
                    DriveRobotPosition(.5, 10, FIND_LINE_FALSE);
                }
            } else {
                if (endposition == EndPosition.ENDCORNER && beacon == 2) {
                    DriveSidewaysTime(1000, -.5);
                    DriveRobotPosition(.5, -80, FIND_LINE_FALSE);
                } else if (endposition == EndPosition.ENDCORNER && beacon == 1) {
                    DriveSidewaysTime(1000, -.5);
                    DriveRobotPosition(.5, -48, FIND_LINE_FALSE);
                } else if (endposition == EndPosition.ENDCENTER && beacon == 2) {
                    DriveSidewaysTime(1000, -.5);
                    DriveTurngyro(0.5, 13.0);
                    DriveRobotPosition(.6, -52, FIND_LINE_FALSE);
                } else if (endposition == EndPosition.ENDCENTER && beacon == 1) {
                    DriveSidewaysTime(2000, -.5);
                    DriveTurngyro(0.5, 0.0);
                    DriveRobotPosition(.5, 15, FIND_LINE_FALSE);
                    DriveSidewaysTime(1500, -.5);
                    DriveRobotPosition(.5, -30, FIND_LINE_FALSE);
                }
            }
        } else {
            longLaunch = true;
            DriveRobotPosition(.5, 20, FIND_LINE_FALSE);
            DriveTurngyro(0.1, -90.0);
            BallLaunch(balls);
            DriveTurngyro(0.1, 0.0);
            DriveRobotPosition(.5, 35, FIND_LINE_FALSE);
        }

        DoTask("End of autonomous", runmode);
    } // end of autonomous

    // FUNCTIONS -----------------------------------------------------------------------------------



    boolean DoTask (String taskname, RunMode debug)
    {
        dashboard.displayPrintf(0, taskname);
        if (debug == RunMode.RUNMODE_DEBUG)
        {
            dashboard.displayPrintf(1, "Press A to run, B to skip");
            while(opModeIsActive()) {
                if(gamepad1.a) {
                    dashboard.displayPrintf(1, "Run");
                    return true;
                }
                if(gamepad1.b) {
                    dashboard.displayPrintf(1, "Skip");
                    sleep(1000);
                    return false;
                }
            }
        }
        else
            return true;
        return true;
    }

    void BeaconPress (int beaconNumber)
    {
        int valueblue = color.blue();
        int valuered = color.red();
        dashboard.displayPrintf(4,"Red   %d", valuered);
        dashboard.displayPrintf(5,"Green %d", color.green());
        dashboard.displayPrintf(6,"Blue  %d", valueblue);
        dashboard.displayPrintf(7,"Pusher  %f", robot.beaconpusher.getPosition());
        if (alliance == Alliance.ALLIANCE_BLUE && valueblue < valuered) {
            DriveRobotPosition(0.15, -7, FIND_LINE_FALSE);
            runLonger = true;
        }
        else if (alliance == Alliance.ALLIANCE_RED && valuered < valueblue) {
            DriveRobotPosition(-0.15, -5, FIND_LINE_FALSE);
        }
        robot.beaconpusher.setPosition(.5);
        sleep(500);
        robot.beaconpusher.setPosition(0.1);
        if (beaconNumber == 2) {
            if (alliance == Alliance.ALLIANCE_BLUE && valueblue < valuered) {
                DriveRobotPosition(0.5, 7, FIND_LINE_FALSE);
            } else if (alliance == Alliance.ALLIANCE_RED && valuered < valueblue) {
                DriveRobotPosition(-0.5, 5, FIND_LINE_FALSE);
            }
        }
    }

    void BallLaunch(int ballnum) {
        if (ballnum == 0) {
            return;
        }
        else {
            dashboard.displayPrintf(3,"encoder: %d", robot.launchingMotor.getCurrentPosition());
            robot.launchingMotor.setTargetPosition(3350);
            robot.launchingMotor.setPower(.5);
            sleep(2500);
            robot.launchingMotor.setPower(0.0);
        }

        if (ballnum == 2) {
            robot.armMotor.setPower(1);
            if (longLaunch) {
                sleep(4000);
            } else {
                sleep(2000);
            }
            robot.armMotor.setPower(0);
            // sleep(1000);

            robot.launchingMotor.setTargetPosition(3350 * 2);
            robot.launchingMotor.setPower(.5);
            sleep(500);
//            robot.launchingMotor.setPower(0.0);
        }
    }


    void DriveRobotTime(int time, double power)
    {
        DrivePowerAll(-power);

        sleep(time);

        DrivePowerAll(0);

    }
// RobotDrivePosition (works best with .25 power)
    void DriveRobotPosition(double power, int inches, boolean findline)
    {
        int position = -inches*90 ;
        int counter = 0;
        double odsvalue = 0.0;
        boolean notFound = true;

        robot.leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DrivePowerAll(-power);

        robot.leftMotorF.setTargetPosition(position);
        robot.rightMotorF.setTargetPosition(position);
        robot.rightMotorB.setTargetPosition(position);
        robot.leftMotorB.setTargetPosition(position);

        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (notFound && (robot.leftMotorF.isBusy() && robot.rightMotorF.isBusy() && robot.rightMotorB.isBusy() && robot.leftMotorB.isBusy())) {
            // look for line
            if (findline)
            {
                counter += 1;
                odsvalue = ods.getRawLightDetected();
                if (odsvalue > .5) {
                    DrivePowerAll(0);
                    notFound = false;
                    dashboard.displayPrintf(5,"found ods: %5.2f",  odsvalue);
                }
            }
            dashboard.displayPrintf(3,"encoder: %d", robot.leftMotorF.getCurrentPosition());
            dashboard.displayPrintf(4,"ods: %5.2f",  odsvalue);
        }

        DrivePowerAll(0);

    }

    void DriveRobothug(double power, int inches, boolean findline)
    {
        int position = -inches*90 ;
        int counter = 0;
        double odsvalue = 0.0;
        boolean notFound = true;

        robot.leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (power > 0){
            robot.leftMotorF.setPower(-power*.9);
            robot.rightMotorF.setPower(-power);
            robot.rightMotorB.setPower(-power*.9);
            robot.leftMotorB.setPower(-power);
        }
        else
        {
            robot.leftMotorF.setPower(-power);
            robot.rightMotorF.setPower(-power*.9);
            robot.rightMotorB.setPower(-power);
            robot.leftMotorB.setPower(-power*.9);
        }

        robot.leftMotorF.setTargetPosition(position);
        robot.rightMotorF.setTargetPosition(position);
        robot.rightMotorB.setTargetPosition(position);
        robot.leftMotorB.setTargetPosition(position);

        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (notFound && (robot.leftMotorF.isBusy() && robot.rightMotorF.isBusy() && robot.rightMotorB.isBusy() && robot.leftMotorB.isBusy())) {
            // look for line
            if (findline)
            {
                counter += 1;
                odsvalue = ods.getRawLightDetected();
                if (odsvalue > .5) {
                    DrivePowerAll(0);
                    notFound = false;
                    dashboard.displayPrintf(5,"found ods: %5.2f",  odsvalue);
                }
            }
            dashboard.displayPrintf(3,"encoder: %d", robot.leftMotorF.getCurrentPosition());
            dashboard.displayPrintf(4,"ods: %5.2f",  odsvalue);
        }

        DrivePowerAll(0);

    }
//RobotTurn
    void DriveRobotTurn (int degree, double power)
    {
        int position = degree*19;

        robot.leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotorF.setPower(-power);
        robot.rightMotorF.setPower(power);
        robot.rightMotorB.setPower(power);
        robot.leftMotorB.setPower(-power);

        robot.leftMotorF.setTargetPosition(-position);
        robot.rightMotorF.setTargetPosition(position);
        robot.rightMotorB.setTargetPosition(position);
        robot.leftMotorB.setTargetPosition(-position);

        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.leftMotorF.isBusy() || robot.rightMotorF.isBusy() || robot.rightMotorB.isBusy() || robot.leftMotorB.isBusy()) {
            dashboard.displayPrintf(3,"encoder: %d", robot.leftMotorF.getCurrentPosition());
        }

        DrivePowerAll(0);

    }

    void DriveRobotSideways (int inches)
    {

        int position = inches;

        robot.rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightMotorF.setPower(-1);
        robot.rightMotorB.setPower(1);
        robot.leftMotorB.setPower(-1);
        robot.leftMotorF.setPower(1);

        robot.rightMotorF.setTargetPosition(-position);
        robot.rightMotorB.setTargetPosition(position);
        robot.leftMotorB.setTargetPosition(-position);
        robot.leftMotorF.setTargetPosition(position);

        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.rightMotorB.isBusy() || robot.leftMotorF.isBusy() || robot.leftMotorB.isBusy() || robot.rightMotorF.isBusy()) {
            dashboard.displayPrintf(3,"encoder: %d", robot.leftMotorF.getCurrentPosition());
        }

        DrivePowerAll(0);
    }

    void DrivePowerAll (double power)
    {
        robot.leftMotorF.setPower(power);
        robot.rightMotorF.setPower(power);
        robot.rightMotorB.setPower(power);
        robot.leftMotorB.setPower(power);
    }

    void DriveSidewaysTime (int time, double power)
    {
        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(power > 0) // right
        {
            robot.leftMotorF.setPower(-power);
            robot.leftMotorB.setPower(power);
            robot.rightMotorB.setPower(-power);
            robot.rightMotorF.setPower(power);
        }
        else // left
        {
            robot.rightMotorF.setPower(power);
            robot.rightMotorB.setPower(-power);
            robot.leftMotorB.setPower(power);
            robot.leftMotorF.setPower(-power);
        }


        sleep(time);

        DrivePowerAll(0);
    }

    void DriveUntilBlue (double power)
    {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        Color.RGBToHSV(color.red() * 8, color.green() * 8, color.blue() * 8, hsvValues);

          // send the info back to driver station using telemetry function.
        dashboard.displayPrintf(3,"Clear %d", color.alpha());
        dashboard.displayPrintf(4,"Red   %d", color.red());
        dashboard.displayPrintf(5,"Green %d", color.green());
        dashboard.displayPrintf(6,"Blue  %d", color.blue());
        dashboard.displayPrintf(7,"Hue   %d", hsvValues[0]);
        Boolean colorFound = false;

        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!colorFound)
        {
            DrivePowerAll(power);
            if (color.blue()>0)
            {
                colorFound = true;
            }

        }
        if (colorFound)
            DrivePowerAll(0);
    }

    void DriveUntilColor (boolean beaconB, double power) {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        Color.RGBToHSV(color.red() * 8, color.green() * 8, color.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        dashboard.displayPrintf(3,"Clear %d", color.alpha());
        dashboard.displayPrintf(4,"Red   %d", color.red());
        dashboard.displayPrintf(5,"Green %d", color.green());
        dashboard.displayPrintf(6,"Blue  %d", color.blue());
        dashboard.displayPrintf(7,"Hue   %d", hsvValues[0]);
        Boolean colorFound = false;

        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (beaconB) {
            while (!colorFound) {
                DrivePowerAll(power);
                if (color.blue() > 1) {
                    colorFound = true;
                }

            }
            if (colorFound)
                DrivePowerAll(0);
        }
        else {
            while (!colorFound) {
                DrivePowerAll(power);
                if (color.red() > 1) {
                    colorFound = true;
                }

            }
            if (colorFound)
                DrivePowerAll(0);

        }

    }

    void Sensebeacon (boolean Bluealliance) {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        Color.RGBToHSV(color.red() * 8, color.green() * 8, color.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        dashboard.displayPrintf(3,"Clear %d", color.alpha());
        dashboard.displayPrintf(4,"Red   %d", color.red());
        dashboard.displayPrintf(5,"Green %d", color.green());
        dashboard.displayPrintf(6,"Blue  %d", color.blue());
        dashboard.displayPrintf(7,"Hue   %d", hsvValues[0]);
        Boolean colorFound = false;

        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if (Bluealliance) {
            while (!colorFound) {
                DrivePowerAll(-.25);
                if (color.blue() > 1) {
                    colorFound = true;

                }
                else if (color.red() > 1){
                    DrivePowerAll(0);
                    sleep(5000);
                    DriveRobotTime(2000, -.25);
                    DriveRobotPosition(.25, 2, FIND_LINE_FALSE);
                    colorFound = true;
                }

            }
            if (colorFound)
                DrivePowerAll(0);
        }
        else
        {
            while (!colorFound) {
                DrivePowerAll(-.25);
                if (color.red() > 1) {
                    colorFound = true;
                }
                else if (color.blue() > 1)
                {
                    DrivePowerAll(0);
                    sleep(5000);
                    DriveRobotTime(2000, -.25);
                    DriveRobotPosition(.25, 2, FIND_LINE_FALSE);
                    colorFound = true;
                }

            }
            if (colorFound)
                DrivePowerAll(0);
        }
    }



// -------GYRO------------------------------------------------------------------------------------------
    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param findline   Boolean to find line
     */
    public void Drivegyro ( double speed,
                            double distance,
                            double angle,
                            boolean findline) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        boolean notFound = true;
        double odsvalue = 0.0;
        int counter = 0;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(-distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftMotorF.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightMotorF.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftMotorF.setTargetPosition(newLeftTarget);
            robot.rightMotorF.setTargetPosition(newRightTarget);
            robot.leftMotorB.setTargetPosition(newLeftTarget);
            robot.rightMotorB.setTargetPosition(newRightTarget);

            robot.leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = -speed;
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftMotorF.setPower(speed);
            robot.rightMotorF.setPower(speed);
            robot.leftMotorB.setPower(speed);
            robot.rightMotorB.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && notFound &&
                    (robot.leftMotorF.isBusy() && robot.rightMotorF.isBusy() && robot.rightMotorB.isBusy() && robot.rightMotorB.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);


                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftMotorF.setPower(leftSpeed);
                robot.rightMotorF.setPower(rightSpeed);
                robot.leftMotorB.setPower(leftSpeed);
                robot.rightMotorB.setPower(rightSpeed);

                // look for line
                if (findline)
                {
                    counter += 1;
                    odsvalue = ods.getRawLightDetected();
                    if (odsvalue > .5) {
                        DrivePowerAll(0);
                        notFound = false;
                        dashboard.displayPrintf(7,"found ods: %5.2f",  odsvalue);
                    }
                }

                // Display drive status for the driver.
                dashboard.displayPrintf(2,"Err/St: %5.1f/%5.1f",  error, steer);
                dashboard.displayPrintf(3,"Target: %7d:%7d",      newLeftTarget,  newRightTarget);
                dashboard.displayPrintf(4,"Actual: %7d:%7d",      robot.leftMotorF.getCurrentPosition(),
                        robot.rightMotorF.getCurrentPosition());
                dashboard.displayPrintf(5,"Speed: %5.2f:%5.2f",  leftSpeed, rightSpeed);
                dashboard.displayPrintf(6,"ods(%d): %5.2f",  counter, odsvalue);
            }

            // Stop all motion;
            DrivePowerAll(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            dashboard.displayPrintf(8,"end ods %5.2f", ods.getRawLightDetected());
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void DriveTurngyro (double speed, double angle) {

        // Turn off RUN_TO_POSITION
        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(-speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
        }

        // Stop all motion;
        DrivePowerAll(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            rightSpeed = Range.clip(rightSpeed, -1.0, 1.0);
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftMotorF.setPower(leftSpeed);
        robot.rightMotorF.setPower(rightSpeed);
        robot.leftMotorB.setPower(leftSpeed);
        robot.rightMotorB.setPower(rightSpeed);

        // Display it for the driver.
        dashboard.displayPrintf(3,"Target %5.2f", angle);
        dashboard.displayPrintf(4,"Err/St %5.2f/%5.2f", error, steer);
        dashboard.displayPrintf(5,"Speed. %5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1.0, 1.0);
    }


    // MENU ----------------------------------------------------------------------------------------
    @Override
    public boolean isMenuUpButton() {
        return gamepad1.dpad_up;
    }

    @Override
    public boolean isMenuDownButton() {
        return gamepad1.dpad_down;
    }

    @Override
    public boolean isMenuEnterButton() {
        return gamepad1.dpad_right;
    }

    @Override
    public boolean isMenuBackButton() {
        return gamepad1.dpad_left;
    }

    private void doMenus()  {
        FtcChoiceMenu modeMenu = new FtcChoiceMenu("Run Mode", null, this);
        FtcChoiceMenu allianceMenu = new FtcChoiceMenu("Alliance:", modeMenu, this);
        FtcChoiceMenu delayMenu = new FtcChoiceMenu("Delay:", allianceMenu, this);
        FtcChoiceMenu startpositionMenu = new FtcChoiceMenu("Start Position:", delayMenu, this);
        FtcChoiceMenu ballMenu = new FtcChoiceMenu("Balls:", startpositionMenu, this);
        FtcChoiceMenu beaconsMenu = new FtcChoiceMenu("Beacons:", ballMenu, this);
        FtcChoiceMenu endpositionMenu = new FtcChoiceMenu("End Position:", beaconsMenu, this);

        modeMenu.addChoice("Auto", RunMode.RUNMODE_AUTO, allianceMenu);
        modeMenu.addChoice("Debug", RunMode.RUNMODE_DEBUG, allianceMenu);

        allianceMenu.addChoice("Red", Alliance.ALLIANCE_RED, delayMenu);
        allianceMenu.addChoice("Blue", Alliance.ALLIANCE_BLUE, delayMenu);

        delayMenu.addChoice("0 seconds", 0, startpositionMenu);
        delayMenu.addChoice("5 seconds", 5000, startpositionMenu);
        delayMenu.addChoice("10 seconds", 10000, startpositionMenu);
        delayMenu.addChoice("15 seconds", 15000, startpositionMenu);

        startpositionMenu.addChoice("1", StartPosition.STARTPOSITION1, ballMenu);
        startpositionMenu.addChoice("2", StartPosition.STARTPOSITION2, ballMenu);

        ballMenu.addChoice("0 balls", 0, beaconsMenu);
        ballMenu.addChoice("1 ball", 1, beaconsMenu);
        ballMenu.addChoice("2 balls", 2, beaconsMenu);

        beaconsMenu.addChoice("0 beacons", 0, endpositionMenu);
        beaconsMenu.addChoice("1 beacon", 1, endpositionMenu);
        beaconsMenu.addChoice("2 beacons", 2, endpositionMenu);

        endpositionMenu.addChoice("Corner", EndPosition.ENDCORNER, null);
        endpositionMenu.addChoice("Center", EndPosition.ENDCENTER, null);


        FtcMenu.walkMenuTree(modeMenu);
        runmode = (RunMode) modeMenu.getCurrentChoiceObject();
        alliance = (Alliance) allianceMenu.getCurrentChoiceObject();
        delay = (int) delayMenu.getCurrentChoiceObject();
        startposition = (StartPosition) startpositionMenu.getCurrentChoiceObject();
        balls = (int) ballMenu.getCurrentChoiceObject();
        beacon = (int) beaconsMenu.getCurrentChoiceObject();
        endposition = (EndPosition) endpositionMenu.getCurrentChoiceObject();

        dashboard.displayPrintf(9, "Mode: %s (%s)",
                modeMenu.getCurrentChoiceText(), runmode.toString());
        dashboard.displayPrintf(10, "Alliance: %s (%s)",
                allianceMenu.getCurrentChoiceText(), alliance.toString());
        dashboard.displayPrintf(11, "Delay = %d msec", delay);
        dashboard.displayPrintf(12, "Start position: %s (%s)", startpositionMenu.getCurrentChoiceText(), startposition.toString());
        dashboard.displayPrintf(13, "Balls = %d ", balls);
        dashboard.displayPrintf(14, "Beacon = %d", beacon);
        dashboard.displayPrintf(15, "End Position = %s (%s)", endpositionMenu.getCurrentChoiceText(),endposition.toString());

    }

//-------------------------------------------------------------------------------------------------
}
