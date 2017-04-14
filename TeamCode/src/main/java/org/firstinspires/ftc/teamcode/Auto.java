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
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import ftclib.*;
import hallib.*;

/**
 * DOCUMENTME
 */

@Autonomous(name="Auto", group="Ourbot")
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
    RunMode         runmode       = RunMode.RUNMODE_AUTO;
    Alliance        alliance      = Alliance.ALLIANCE_RED;
    int             delay         = 0;
    StartPosition   startposition = StartPosition.STARTPOSITION1;
    int             balls         = 2;
    int             beacon        = 2;
    EndPosition     endposition   = EndPosition.ENDCENTER;

    /* Declare OpMode members. */
    private HalDashboard dashboard;
    OurHardware                               robot   = new OurHardware(); // Use Ourbot's hardware
    ModernRoboticsI2cGyro                     gyro    = null;    // Additional Gyro device
    ModernRoboticsI2cColorSensor              colorR  = null;
    ModernRoboticsI2cColorSensor              colorL  = null;
    ModernRoboticsAnalogOpticalDistanceSensor ods     = null;

    static final double     COUNTS_PER_MOTOR_REV      = 1120;    // Neverest 40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION      = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES     = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                        (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     HEADING_THRESHOLD         = 1;       // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF              = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF             = 0.1;     // Larger is more responsive, but also less stable

    static final boolean    FIND_LINE_TRUE            = true;
    static final boolean    FIND_LINE_FALSE           = false;

    boolean                 runLonger                 = false;
    boolean                 longBallLoad              = false;
    boolean                 ballLoaded                = true;

    /** These are the gear ratio constants.
     * After Supers, we decided to overhaul our robot, and changed the gears to sprockets and increase the speed.
     * We put these here to save us the work of changing every function.
     **/
    double                  gearRatio                 = -.6667;
    double                  powerRatio                = -1;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        // The init() method of the hardware class does all the work here
        robot.init(hardwareMap);

        // Initialize dashboard
        dashboard = HalDashboard.createInstance(telemetry);

        // Initialize gyro -------------------------------------------------------------------------
        /*dashboard.displayPrintf(0, "Calibrating Gyro");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

        // Wait for the gyro to finish calibration before continuing
        while (gyro.isCalibrating()) {
            sleep(50);
            idle();
        }
        gyro.resetZAxisIntegrator();*/

        // Initialize color sensor -----------------------------------------------------------------
        dashboard.displayPrintf(0, "Initializing Color Sensors");
        colorR = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("colorR");
        colorR.setI2cAddress(I2cAddr.create7bit(0x1f));
        colorL = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("colorL");
        colorL.setI2cAddress(I2cAddr.create7bit(0x1e));
        colorR.enableLed(false);
        colorL.enableLed(false);

        // Initialize optical distance sensor ------------------------------------------------------
        ods = (ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods");
        ods.enableLed(true);

        // Initialize beacon pusher ----------------------------------------------------------------
        robot.beaconpusherR.setPosition(0.1);
        robot.beaconpusherL.setPosition(0.9);
        robot.dagate.setPosition(.85);

        robot.launchingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launchingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.launchingMotor.setTargetPosition(0);
        robot.launchingMotor.setPower(0.0);
/*
        robot.rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorF.setPower(.3);
        robot.rightMotorF.setTargetPosition((int)(12*90*gearRatio));
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
        robot.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorB.setPower(.3);
        robot.rightMotorB.setTargetPosition((int)(12*90*gearRatio));
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
        robot.leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorF.setPower(.3);
        robot.leftMotorF.setTargetPosition((int)(12*90*gearRatio));
        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
        robot.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorB.setPower(.3);
        robot.leftMotorB.setTargetPosition((int)(12*90*gearRatio));
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
*/

        // Wait for the game to start (driver presses PLAY)
        doMenus();
        dashboard.displayPrintf(0, "Status: Ready to start");
        //dashboard.displayPrintf(3, "Gyro  %d", gyro.getIntegratedZValue());
        dashboard.displayPrintf(4, "Red R   %d", colorR.red());
        dashboard.displayPrintf(5, "Green R %d", colorR.green());
        dashboard.displayPrintf(6, "Blue R  %d", colorR.blue());
        dashboard.displayPrintf(7, "Red L   %d", colorL.red());
        dashboard.displayPrintf(8, "Green L %d", colorL.green());
        dashboard.displayPrintf(9, "Blue L  %d", colorL.blue());

        waitForStart();

        /*
         * ----- AUTONOMOUS START-------------------------------------------------------------------
         */

        // Pause the program for the selected delay period
        sleep(delay);

        if (startposition == StartPosition.STARTPOSITION1) {
            if (DoTask("Setup Ball Launch", runmode)) {
                DriveRobotPosition(.3, 13, FIND_LINE_FALSE);
            }
            if (DoTask("Ball Launch", runmode)) {
                sleep(250);
                BallLaunch(balls);
            }

            // go to beacon
            if (DoTask("Go to beacon 1", runmode)) {
                if (beacon > 0) {
                    if (alliance == Alliance.ALLIANCE_BLUE && startposition == StartPosition.STARTPOSITION1) {
                        DriveRobotTurn(.25, 45);
                        DriveRobotPosition(.75, 80, FIND_LINE_FALSE);
                        DriveRobotTurn(-.25, -40);
                        DriveSidewaysTime(750, 1);
                        DriveRobothug(0.125, 35, FIND_LINE_TRUE);
                    } else if (alliance == Alliance.ALLIANCE_RED && startposition == StartPosition.STARTPOSITION1) {
                        DriveRobotTurn(-.25, -45);
                        DriveRobotPosition(.75, 75,FIND_LINE_FALSE);
                        DriveRobotTurn(-.25, -140);
                        DriveSidewaysTime(1250, 1);
                        DriveRobothug(-0.15, -30, FIND_LINE_TRUE);
                        DriveRobothug(.125, 3, FIND_LINE_TRUE);
                    }
                }
            }

            if (DoTask("Beacon 1 Push", runmode)) {
                BeaconPress();
            }

            if (DoTask("Drive to Beacon 2", runmode)) {
                if (beacon == 2) {
                    if (alliance == Alliance.ALLIANCE_BLUE) {
                        DriveRobothug(-.75, -30, FIND_LINE_FALSE);
                        DriveSidewaysTime(1000, 1);
                        DriveRobothug(-.125, -25, FIND_LINE_TRUE);
                        DriveRobothug(.125, 3, FIND_LINE_TRUE);
                    } else {
                        DriveRobothug(.7, 35, FIND_LINE_FALSE);
                        DriveSidewaysTime(500, 1);
                        DriveRobothug(.15, 25, FIND_LINE_TRUE);
                    }

                    if (DoTask("Beacon 2 Push", runmode)) {
                        BeaconPress();
                    }
                }
            }
            if (alliance == Alliance.ALLIANCE_RED) {
                if (endposition == EndPosition.ENDCORNER && beacon == 1) {
                    DriveSidewaysTime(2000, -.75);
                    DriveRobotPosition(.5, 80, FIND_LINE_FALSE);
                } else if (endposition == EndPosition.ENDCORNER && beacon == 2) {
                    DriveSidewaysTime(1000, -.75);
                    DriveRobotPosition(.5, 30, FIND_LINE_FALSE);
                } else if (endposition == EndPosition.ENDCENTER && beacon == 1) {
                    DriveSidewaysTime(1000, -.75);
                    DriveRobotTurn(0.5, -45);
                    DriveRobotPosition(.6, 55, FIND_LINE_FALSE);
                } else if (endposition == EndPosition.ENDCENTER && beacon == 2) {
                    DriveRobothug(-1, -40, FIND_LINE_FALSE);
                    DriveSidewaysTime(1000, -1);
                    DriveRobotTurn(-1, -50);
                    DriveRobotPosition(1, 55, FIND_LINE_FALSE);
                }
            } else {
                if (endposition == EndPosition.ENDCORNER && beacon == 1) {
                    DriveSidewaysTime(1000, -.5);
                    DriveRobotPosition(.75, -80, FIND_LINE_FALSE);
                } else if (endposition == EndPosition.ENDCORNER && beacon == 2) {
                    DriveSidewaysTime(1000, -.75);
                    DriveRobotPosition(-.75, -40, FIND_LINE_FALSE);
                } else if (endposition == EndPosition.ENDCENTER && beacon == 1) {
                    DriveSidewaysTime(1000, -.5);
                    DriveRobotTurn(0.5, 13);
                    DriveRobotPosition(.6, -60, FIND_LINE_FALSE);
                } else if (endposition == EndPosition.ENDCENTER && beacon == 2) {
                    DriveRobothug(1, 40, FIND_LINE_FALSE);
                    DriveSidewaysTime(1000, -1);
                    DriveRobotTurn(1, 40);
                    DriveRobotPosition(-1, -55, FIND_LINE_FALSE);
                }
            }
        } else if (startposition == StartPosition.STARTPOSITION2) {
            if (DoTask("Setup Ball Launch", runmode)) {
                longBallLoad = true;
                DriveRobotPosition(.5, 24, FIND_LINE_FALSE);
            }
            if (DoTask("Ball Launch", runmode))
                BallLaunch(balls);
            /*if (DoTask("Go to Beacon 1", runmode)) {
                DriveTurngyro(0.1, 0.0);
                DriveRobotPosition(1, 35, FIND_LINE_FALSE);
                sleep(500);
                //DriveRobotPosition(.5, 65, FIND_LINE_FALSE);
            }*/
            if (alliance == Alliance.ALLIANCE_BLUE && endposition == EndPosition.ENDCORNER) {
                DriveRobotTurn(.25, 40);
                DriveRobotPosition(-.75, -70, FIND_LINE_FALSE);
                sleep(500);
            }
            else if (alliance == Alliance.ALLIANCE_RED && endposition == EndPosition.ENDCORNER) {
                DriveRobotTurn(0.25, -140);
                DriveRobotPosition(.75, -70, FIND_LINE_FALSE);
                sleep(500);
            }
            else if (endposition == EndPosition.ENDCENTER) {
                DriveRobotPosition(1, 35, FIND_LINE_FALSE);
                sleep(500);
            }
        }

        DoTask("End of autonomous", runmode);
    }// end of autonomous



    /**
     * FUNCTIONS -----------------------------------------------------------------------------------
     */


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

    void BeaconPress ()
    {
        int valueblueR = colorR.blue();
        int valueredR = colorR.red();
        int valueblueL = colorL.blue();
        int valueredL = colorL.red();
        int counter = 0;
        int tries = 2;
        dashboard.displayPrintf(3,"R%d B%d", valueredR,valueblueR);
        dashboard.displayPrintf(4,"R%d B%d", valueredL,valueblueL);
        dashboard.displayPrintf(7,"PusherR  %f", robot.beaconpusherR.getPosition());
        dashboard.displayPrintf(8,"PusherL  %f", robot.beaconpusherL.getPosition());
        if (endposition == EndPosition.ENDCORNER)
            tries = 3;
        if (alliance == Alliance.ALLIANCE_BLUE && valueblueR > valueredR) {

            while (colorL.red() > colorL.blue() && counter < tries){
                if (2*(counter%2)-1 == -1 && counter > 0){
                    DriveRobothug(-.15, -3, FIND_LINE_TRUE);
                }
                else {
                    DriveRobothug(.15, 3, FIND_LINE_TRUE);
                }

                robot.beaconpusherR.setPosition(1);
                sleep(1000);
                robot.beaconpusherR.setPosition(.1);
                counter = counter +1;
            }

        }
        else if (alliance == Alliance.ALLIANCE_BLUE && valueblueL > valueredL) {

            while (colorR.red() > colorR.blue() && counter < tries){
                if (2*(counter%2)-1 == -1 && counter > 0){
                    DriveRobothug(-.15, -3, FIND_LINE_TRUE);
                }
                else {
                    DriveRobothug(.15, 3, FIND_LINE_TRUE);
                }

                robot.beaconpusherL.setPosition(0);
                sleep(1000);
                robot.beaconpusherL.setPosition(1);
                counter = counter +1;
            }

        }
        else if (alliance == Alliance.ALLIANCE_RED && valueredL > valueblueL) {

            while (colorR.blue() > colorR.red() && counter < tries){
                if (2*(counter%2)-1 == -1 && counter > 0){
                    DriveRobothug(-.15, -3, FIND_LINE_TRUE);
                }
                else {
                    DriveRobothug(.15, 3, FIND_LINE_TRUE);
                }

                robot.beaconpusherL.setPosition(0);
                sleep(1000);
                robot.beaconpusherL.setPosition(1);
                counter = counter +1;
            }

        }
        else if (alliance == Alliance.ALLIANCE_RED && valueblueR < valueredR) {

            while (colorL.red() < colorL.blue() && counter < tries){
                if (2*(counter%2)-1 == -1 && counter > 0){
                    DriveRobothug(-.15, -3, FIND_LINE_TRUE);
                }
                else {
                    DriveRobothug(.15, 3, FIND_LINE_TRUE);
                }

                robot.beaconpusherR.setPosition(1);
                sleep(1000);
                robot.beaconpusherR.setPosition(.1);
                counter = counter +1;
            }

        }
        else if ((alliance == Alliance.ALLIANCE_BLUE && valueredR > valueblueR && valueredL > valueblueL) ||
                (alliance == Alliance.ALLIANCE_RED && valueredR < valueblueR && valueredL < valueblueL)){

            while (counter < 5 && ((alliance == Alliance.ALLIANCE_BLUE && colorR.red() > colorR.blue()) ||
                    (alliance == Alliance.ALLIANCE_RED && colorR.red() < colorR.blue()))){
                robot.beaconpusherR.setPosition(1);
                robot.beaconpusherL.setPosition(0);
                sleep(490);
                robot.beaconpusherR.setPosition(0);
                robot.beaconpusherL.setPosition(1);
                sleep(490);
                counter = counter +1;
            }

        }

    }

    void BallLaunch (int ballsToLaunch) {
        robot.launchingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int ballsLaunched = 0;
        while (ballsToLaunch > ballsLaunched) {
            if (!ballLoaded) {
                // Load ball
                robot.dagate.setPosition(0);
                sleep(500);
                robot.dagate.setPosition(.85);
            }

            // Display position data
            dashboard.displayPrintf(3,"encoder: %d", robot.launchingMotor.getCurrentPosition());

            // Launch ball
            robot.launchingMotor.setTargetPosition(3360 * (ballsLaunched + 1));
            robot.launchingMotor.setPower(0.5);
            sleep(2000);

            //
            ballsLaunched += 1;
            ballLoaded = false;
        }
    }


    void DriveRobotTime(int time, double power)
    {
        DrivePowerAll(-power);

        sleep(time);

        DrivePowerAll(0);

    }

    // Do not use-dangerous! (uses while (x.isBusy) loop)
    // RobotDrivePosition (works best with .25 power)
    void DriveRobotPosition(double power, int inches, boolean findline)
    {
        double position = -inches*90*gearRatio;
        int counter = 0;
        double odsvalue = 0.0;
        boolean notFound = true;

        robot.leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DrivePowerAll(-power*powerRatio);

        robot.leftMotorF.setTargetPosition((int)position);
        robot.rightMotorF.setTargetPosition((int)position);
        robot.rightMotorB.setTargetPosition((int)position);
        robot.leftMotorB.setTargetPosition((int)position);

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

    // Do not use-dangerous! (uses while (x.isBusy) loop)
    /**
     * DriveRobotHug is used to make the robot drive hugging a wall.
     * The robot will move mostly straight and slightly to the side,
     * so it will stay against the wall.
     * @param power Power to use while driving
     * @param inches How many inches to drive
     * @param findline Whether to stop if a white line is detected
     */
    void DriveRobothug (double power, int inches, boolean findline)
    {
        double position = -inches*90*gearRatio;
        int counter = 0;
        double odsvalue = 0.0;
        boolean notFound = true;

        robot.leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (power > 0){
            robot.leftMotorF.setPower(-power*.9*powerRatio);
            robot.rightMotorF.setPower(-power*powerRatio);
            robot.rightMotorB.setPower(-power*.9*powerRatio);
            robot.leftMotorB.setPower(-power*powerRatio);
        }
        else
        {
            robot.leftMotorF.setPower(-power*powerRatio);
            robot.rightMotorF.setPower(-power*.9*powerRatio);
            robot.rightMotorB.setPower(-power*powerRatio);
            robot.leftMotorB.setPower(-power*.9*powerRatio);
        }

        robot.leftMotorF.setTargetPosition((int)position);
        robot.rightMotorF.setTargetPosition((int)position);
        robot.rightMotorB.setTargetPosition((int)position);
        robot.leftMotorB.setTargetPosition((int)position);

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


    void DriveRobotTurn (double power, int degree)
    {
        double position = degree*19*gearRatio;

        robot.leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotorF.setPower(-power*powerRatio);
        robot.rightMotorF.setPower(power*powerRatio);
        robot.rightMotorB.setPower(power*powerRatio);
        robot.leftMotorB.setPower(-power*powerRatio);

        robot.leftMotorF.setTargetPosition(-(int)position);
        robot.rightMotorF.setTargetPosition((int)position);
        robot.rightMotorB.setTargetPosition((int)position);
        robot.leftMotorB.setTargetPosition(-(int)position);

        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.leftMotorF.isBusy() && robot.rightMotorF.isBusy() && robot.rightMotorB.isBusy() && robot.leftMotorB.isBusy()) {
            dashboard.displayPrintf(3,"encoder: %d", robot.leftMotorF.getCurrentPosition());
        }

        DrivePowerAll(0);

    }

    // Do not use-dangerous! (uses while (x.isBusy) loop)
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

    /**
     * DrivePowerAll sets all of the drive train motors to the specified power level.
     * @param power Power level to set motors to
     */
    void DrivePowerAll (double power)
    {
        robot.leftMotorF.setPower(power);
        robot.rightMotorF.setPower(power);
        robot.rightMotorB.setPower(power);
        robot.leftMotorB.setPower(power);
    }

    /**
     * DriveSidewaysTime makes the robot drive sideways for the specified time and power.
     * @param time How long to drive
     * @param power The power to use while driving,
     *              positive values go right and negative values go left
     */
    void DriveSidewaysTime (int time, double power)
    {
        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (power > 0) // Drive right
        {
            robot.leftMotorF.setPower(-power*powerRatio);
            robot.leftMotorB.setPower(power*powerRatio);
            robot.rightMotorF.setPower(power*powerRatio);
            robot.rightMotorB.setPower(-power*powerRatio);
        }
        else // Drive left
        {
            robot.leftMotorF.setPower(-power*powerRatio);
            robot.leftMotorB.setPower(power*powerRatio);
            robot.rightMotorF.setPower(power*powerRatio);
            robot.rightMotorB.setPower(-power*powerRatio);
        }
        // Continue driving for the specified amount of time, then stop
        sleep(time);
        DrivePowerAll(0);
    }

    void DriveUntilBlue (double power)
    {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        Color.RGBToHSV(colorR.red() * 8, colorR.green() * 8, colorR.blue() * 8, hsvValues);

          // send the info back to driver station using telemetry function.
        dashboard.displayPrintf(3,"Clear %d", colorR.alpha());
        dashboard.displayPrintf(4,"Red   %d", colorR.red());
        dashboard.displayPrintf(5,"Green %d", colorR.green());
        dashboard.displayPrintf(6,"Blue  %d", colorR.blue());
        dashboard.displayPrintf(7,"Hue   %d", hsvValues[0]);
        Boolean colorFound = false;

        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!colorFound)
        {
            DrivePowerAll(power*powerRatio);
            if (colorR.blue()>0)
            {
                colorFound = true;
            }

        }
        DrivePowerAll(0);
    }

    void DriveUntilColor (boolean beaconB, double power) {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        Color.RGBToHSV(colorR.red() * 8, colorR.green() * 8, colorR.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        dashboard.displayPrintf(3,"Clear %d", colorR.alpha());
        dashboard.displayPrintf(4,"Red   %d", colorR.red());
        dashboard.displayPrintf(5,"Green %d", colorR.green());
        dashboard.displayPrintf(6,"Blue  %d", colorR.blue());
        dashboard.displayPrintf(7,"Hue   %d", hsvValues[0]);
        Boolean colorFound = false;

        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (beaconB) {
            while (!colorFound) {
                DrivePowerAll(power*powerRatio);
                if (colorR.blue() > 1) {
                    colorFound = true;
                }

            }
            DrivePowerAll(0);
        }
        else {
            while (!colorFound) {
                DrivePowerAll(power*powerRatio);
                if (colorR.red() > 1) {
                    colorFound = true;
                }

            }
            DrivePowerAll(0);
        }

    }

    void Sensebeacon (boolean Bluealliance) {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        Color.RGBToHSV(colorR.red() * 8, colorR.green() * 8, colorR.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        dashboard.displayPrintf(3,"Clear %d", colorR.alpha());
        dashboard.displayPrintf(4,"Red   %d", colorR.red());
        dashboard.displayPrintf(5,"Green %d", colorR.green());
        dashboard.displayPrintf(6,"Blue  %d", colorR.blue());
        dashboard.displayPrintf(7,"Hue   %d", hsvValues[0]);
        Boolean colorFound = false;

        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if (Bluealliance) {
            while (!colorFound) {
                DrivePowerAll(-.25*powerRatio);
                if (colorR.blue() > 1) {
                    colorFound = true;

                }
                else if (colorR.red() > 1){
                    DrivePowerAll(0);
                    sleep(5000);
                    DriveRobotTime(2000, -.25*powerRatio);
                    DriveRobotPosition(.25, 2, FIND_LINE_FALSE);
                    colorFound = true;
                }

            }
            DrivePowerAll(0);
        }
        else
        {
            while (!colorFound) {
                DrivePowerAll(-.25*powerRatio);
                if (colorR.red() > 1) {
                    colorFound = true;
                }
                else if (colorR.blue() > 1)
                {
                    DrivePowerAll(0);
                    sleep(5000);
                    DriveRobotTime(2000, -.25*powerRatio);
                    DriveRobotPosition(.25, 2, FIND_LINE_FALSE);
                    colorFound = true;
                }

            }
            DrivePowerAll(0);
        }
    }



    // ------- GYRO DRIVE FUNCTIONS ----------------------------------------------------------------

    // Do not use-dangerous! (uses while (x.isBusy) loop)
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
            speed = Range.clip(Math.abs(speed), -1.0, 1.0);
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

                // Display drive status for the driver
                dashboard.displayPrintf(2,"Err/St: %5.1f/%5.1f",  error, steer);
                dashboard.displayPrintf(3,"Target: %7d:%7d",      newLeftTarget,  newRightTarget);
                dashboard.displayPrintf(4,"Actual: %7d:%7d",      robot.leftMotorF.getCurrentPosition(),
                        robot.rightMotorF.getCurrentPosition());
                dashboard.displayPrintf(5,"Speed: %5.2f:%5.2f",  leftSpeed, rightSpeed);
                dashboard.displayPrintf(6,"ods(%d): %5.2f",  counter, odsvalue);
            }

            // Stop all motion
            DrivePowerAll(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Display ODS data on dashboard
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
            // Update telemetry and allow time for other processes to run.
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
    // ------- END GYRO DRIVE FUNCTIONS ------------------------------------------------------------



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

    private void doMenus() {
        FtcChoiceMenu modeMenu = new FtcChoiceMenu("Run Mode", null, this);
        FtcChoiceMenu allianceMenu = new FtcChoiceMenu("Alliance:", modeMenu, this);
        FtcChoiceMenu delayMenu = new FtcChoiceMenu("Delay:", allianceMenu, this);
        FtcChoiceMenu startpositionMenu = new FtcChoiceMenu("Start Position:", delayMenu, this);
        FtcChoiceMenu ballMenu = new FtcChoiceMenu("Balls:", startpositionMenu, this);
        FtcChoiceMenu beaconsMenu = new FtcChoiceMenu("Beacons:", ballMenu, this);
        FtcChoiceMenu endpositionMenu = new FtcChoiceMenu("End Position:", beaconsMenu, this);

        modeMenu.addChoice("Auto", RunMode.RUNMODE_AUTO, true, allianceMenu);
        modeMenu.addChoice("Debug", RunMode.RUNMODE_DEBUG, false, allianceMenu);

        allianceMenu.addChoice("Red", Alliance.ALLIANCE_RED, true, delayMenu);
        allianceMenu.addChoice("Blue", Alliance.ALLIANCE_BLUE, false, delayMenu);

        delayMenu.addChoice("0 seconds", 0, true, startpositionMenu);
        delayMenu.addChoice("5 seconds", 5000, false, startpositionMenu);
        delayMenu.addChoice("10 seconds", 10000, false, startpositionMenu);
        delayMenu.addChoice("15 seconds", 15000, false, startpositionMenu);

        startpositionMenu.addChoice("1", StartPosition.STARTPOSITION1, true, ballMenu);
        startpositionMenu.addChoice("2", StartPosition.STARTPOSITION2, false, ballMenu);

        ballMenu.addChoice("0 balls", 0, false, beaconsMenu);
        ballMenu.addChoice("1 ball", 1, false, beaconsMenu);
        ballMenu.addChoice("2 balls", 2, true, beaconsMenu);

        beaconsMenu.addChoice("0 beacons", 0, false, endpositionMenu);
        beaconsMenu.addChoice("1 beacon", 1, false, endpositionMenu);
        beaconsMenu.addChoice("2 beacons", 2, true, endpositionMenu);

        endpositionMenu.addChoice("Corner", EndPosition.ENDCORNER, false, null);
        endpositionMenu.addChoice("Center", EndPosition.ENDCENTER, true, null);


        FtcMenu.walkMenuTree(modeMenu, this);
        runmode = (RunMode) modeMenu.getCurrentChoiceObject();
        alliance = (Alliance) allianceMenu.getCurrentChoiceObject();
        delay = (int) delayMenu.getCurrentChoiceObject();
        startposition = (StartPosition) startpositionMenu.getCurrentChoiceObject();
        balls = (int) ballMenu.getCurrentChoiceObject();
        beacon = (int) beaconsMenu.getCurrentChoiceObject();
        endposition = (EndPosition) endpositionMenu.getCurrentChoiceObject();

        dashboard.displayPrintf(9, "Mode: %s (%s)", modeMenu.getCurrentChoiceText(), runmode.toString());
        dashboard.displayPrintf(10, "Alliance: %s (%s)", allianceMenu.getCurrentChoiceText(), alliance.toString());
        dashboard.displayPrintf(11, "Delay = %d msec", delay);
        dashboard.displayPrintf(12, "Start position: %s (%s)", startpositionMenu.getCurrentChoiceText(), startposition.toString());
        dashboard.displayPrintf(13, "Balls = %d ", balls);
        dashboard.displayPrintf(14, "Beacon = %d", beacon);
        dashboard.displayPrintf(15, "End Position = %s (%s)", endpositionMenu.getCurrentChoiceText(), endposition.toString());
    }
    // END MENU ------------------------------------------------------------------------------------
}