/*
Copyright (c) 2016 Robert Atkinson (original code for Pushbot), FTC team #10298 Brain Stormz (making it fit our bot)

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
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file provides basic Teleop driving for our robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Ourbot hardware class to define the devices on the robot.
 * All device access is managed through the OurHardware class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for our robot.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="OurTeleop", group="Ourbot")
//@Disabled
public class OurTeleop extends OpMode {

    /* Declare OpMode members. */
    OurHardware robot       = new OurHardware(); // use the class created to define Ourbot's hardware
    ModernRoboticsAnalogOpticalDistanceSensor ods = null;
    /*
    //findlines
    static final boolean    FIND_LINE_TRUE        = true;
    static final boolean    FIND_LINE_FALSE       = false;
    */
    int                     counter               = 0;
    boolean                 FINDLINE              = false;
    double                  ODSvalue                = 0;

    Boolean launchAfterLoad = false;
    Boolean stillLoading = false;

    private ElapsedTime msAfterLaunch = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    // Code to run once when the driver hits INIT
    @Override
    public void init() {
        /*
         * Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Initialize optical distance sensor ------------------------------------------------------
        ods = (ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods");
        ods.enableLed(true);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Say", "Robot ready");
    }

    /*
     * Code to run in a loop after the driver hits play until they hit the stop button
     */
    @Override
    public void loop() {

        double leftFrontPower;
        double leftBackPower;
        double rightBackPower;
        double rightFrontPower;

        double TurnLeft;
        double TurnRight;

        double loaderPower;
        double launchPower;
        double servoPosition;

        // Send telemetry message to signify robot running
        if (gamepad1.x) {
            telemetry.addData("Say", "Running");
            telemetry.addData("ods", ODSvalue);
        }
      
        /* // START OF SIDE DRIVE
        double SideDriveL = gamepad1.left_trigger;
        double SideDriveR = gamepad1.right_trigger;
        if (SideDriveL > 0.1) {
            robot.leftMotorF.setPower(SideDriveL);
            robot.leftMotorB.setPower(-SideDriveL);
            robot.rightMotorF.setPower(-SideDriveL);
            robot.rightMotorB.setPower(SideDriveL);
        } else if (SideDriveR > 0.1) {
            robot.leftMotorF.setPower(-SideDriveR);
            robot.leftMotorB.setPower(SideDriveR);
            robot.rightMotorF.setPower(SideDriveR);
            robot.rightMotorB.setPower(-SideDriveR);
        }
        // END OF SIDE DRIVE
        else {
            // START OF TANK DRIVE
            leftFrontPower = gamepad1.left_stick_y;
            leftBackPower = gamepad1.left_stick_y;
            rightFrontPower = gamepad1.right_stick_y;
            rightBackPower = gamepad1.right_stick_y;

            robot.leftMotorF.setPower(leftFrontPower);
            robot.leftMotorB.setPower(leftBackPower);
            robot.rightMotorF.setPower(rightFrontPower);
            robot.rightMotorB.setPower(rightBackPower);
            // END OF TANK DRIVE
        }
        */

        // START OF DPAD DRIVE
        if (gamepad1.dpad_right) {
            robot.leftMotorF.setPower(-1);
            robot.leftMotorB.setPower(1);
            robot.rightMotorF.setPower(1);
            robot.rightMotorB.setPower(-1);
        }
        else if (gamepad1.dpad_left) {
            robot.leftMotorF.setPower(1);
            robot.leftMotorB.setPower(-1);
            robot.rightMotorF.setPower(-1);
            robot.rightMotorB.setPower(1);
        }
        else if (gamepad1.dpad_down) {
            robot.leftMotorF.setPower(1);
            robot.leftMotorB.setPower(1);
            robot.rightMotorF.setPower(1);
            robot.rightMotorB.setPower(1);
        }
        else if (gamepad1.dpad_up) {
            TurnRight = gamepad1.right_trigger;
            TurnLeft = gamepad1.left_trigger;

            if (gamepad1.right_trigger > 0.1) {
                robot.leftMotorF.setPower(-1);
                robot.leftMotorB.setPower(-1);
                robot.rightMotorF.setPower(-1 + TurnRight);
                robot.rightMotorB.setPower(-1 + TurnRight);
            }
            else if (gamepad1.left_trigger > 0.1) {
                robot.leftMotorF.setPower(-1 + TurnLeft);
                robot.leftMotorB.setPower(-1 + TurnLeft);
                robot.rightMotorF.setPower(-1);
                robot.rightMotorB.setPower(-1);
            }
            else {
                robot.leftMotorF.setPower(-1);
                robot.leftMotorB.setPower(-1);
                robot.rightMotorF.setPower(-1);
                robot.rightMotorB.setPower(-1);
            }
        }
        // END OF DPAD DRIVE

        /*
        else if (gamepad1.right_bumper) {
                robot.leftMotorF.setPower (-1);
                robot.leftMotorB.setPower (-1);
                robot.rightMotorF.setPower (1);
                robot.rightMotorB.setPower (1);
        }
        else if (gamepad1.left_bumper) {
                robot.leftMotorF.setPower (1);
                robot.leftMotorB.setPower (1);
                robot.rightMotorF.setPower (-1);
                robot.rightMotorB.setPower (-1);
        }
        */

        // START OF HUG
        else if (gamepad2.left_stick_x > .1){
            DriveRobothug(-gamepad2.left_stick_x);
        }
        else if (gamepad2.left_stick_x < -.1){
            DriveRobothug(-gamepad2.left_stick_x);
        }
        else if (gamepad2.left_stick_y < -.5 ){
            DriveSideways(-gamepad2.left_stick_y);
        }
        else if (gamepad2.left_stick_y > .5 ){
            DriveSideways(-gamepad2.left_stick_y);
        }
        else if (gamepad2.dpad_right){
            robot.rightMotorF.setPower(.75);
            robot.rightMotorB.setPower(.75);
        }
        else if (gamepad2.dpad_left){
            robot.rightMotorF.setPower(-.75);
            robot.rightMotorB.setPower(-.75);
        }
        // END OF HUG

        // Enhanced tank drive
        else {
            FINDLINE = false;
            leftFrontPower = Range.clip(gamepad1.left_stick_y + (-1 * gamepad1.left_stick_x), -1.0, 1.0);
            leftBackPower = Range.clip(gamepad1.left_stick_y + (1 * gamepad1.left_stick_x), -1.0, 1.0);
            rightFrontPower = Range.clip(gamepad1.right_stick_y + (1 * gamepad1.right_stick_x), -1.0, 1.0);
            rightBackPower = Range.clip(gamepad1.right_stick_y + (-1 * gamepad1.right_stick_x), -1.0, 1.0);

            robot.leftMotorF.setPower(leftFrontPower);
            robot.leftMotorB.setPower(leftBackPower);
            robot.rightMotorF.setPower(rightFrontPower);
            robot.rightMotorB.setPower(rightBackPower);
        }

        // Launching arm and loading mechanism code
        if (gamepad2.right_bumper) {
            launchAfterLoad = true;
            stillLoading = true;
            robot.loaderMotor.setPower(1);
            robot.launchingMotor.setPower(0);
        }
        else {
            stillLoading = false;
        }
        if (launchAfterLoad && !stillLoading) {
            launchAfterLoad = false;
            msAfterLaunch.reset();
            if (robot.launchingMotor.getMode() == RUN_TO_POSITION) {
                int oneMoreTurn = robot.launchingMotor.getTargetPosition() + 3360;
                robot.launchingMotor.setPower(0);
                robot.launchingMotor.setTargetPosition(oneMoreTurn);
                robot.launchingMotor.setPower(1.0);
            } else {
                robot.launchingMotor.setMode(STOP_AND_RESET_ENCODER);
                robot.launchingMotor.setTargetPosition(3360);
                robot.launchingMotor.setMode(RUN_TO_POSITION);
                robot.launchingMotor.setPower(1.0);
            }
        }
        launchPower = (gamepad2.right_trigger);
        if (launchPower > 0.1) {
            robot.launchingMotor.setMode(RUN_USING_ENCODER);
            robot.launchingMotor.setPower(launchPower);
        } else if (robot.launchingMotor.getMode() == RUN_USING_ENCODER) {
            robot.launchingMotor.setPower(0.0);
        }
        if (msAfterLaunch.time() > 750) {
            robot.launchingMotor.setPower(0.5);
        }

        loaderPower = gamepad2.right_stick_y;
        if (loaderPower > 0.15) {
            // loaderPower = loaderPower * loaderPower;
            robot.loaderMotor.setPower(loaderPower);
        } else if (loaderPower < -0.15) {
            // loaderPower = -loaderPower * loaderPower;
            robot.loaderMotor.setPower(loaderPower);
        } else {
            robot.loaderMotor.setPower(0.0);
        }

        /* //START OF GAMEPAD 2 MANUAL OVERRIDE
        if (gamepad2.left_stick_y > .1){
            DriveSideways(.5);
        }
        else if (gamepad2.left_stick_y < -.1){
            DriveSideways(-.5);
        }
        if (gamepad2.left_stick_x > 1){
            DrivePowerAll(.5);
        }
        else if (gamepad2.left_stick_x < 1){
            DrivePowerAll(-.5);
        } */

        //START OF SERVO BEACON PUSHER
        if (!FINDLINE) {
            if (gamepad2.left_trigger > .2) {
                servoPosition = gamepad2.left_trigger / 2;
            } else {
                servoPosition = .1;
            }
            robot.beaconpusher.setPosition(servoPosition);
        }
    }

    @Override
    public void stop () {
        // Code here runs ONCE after the driver hits stop

        // Stop launcher if launch in progress
        robot.launchingMotor.setPower(0.0);
    }

    /*
    FUNCTIONS------------------------------------------------------------------------------------------------------
     */
    void DrivePowerAll (double power)
    {
        robot.leftMotorF.setPower(power);
        robot.rightMotorF.setPower(power);
        robot.rightMotorB.setPower(power);
        robot.leftMotorB.setPower(power);
    }
    void DriveRobothug (double power)
    {
        power = Range.clip(power, -.9, .9);
        if (gamepad2.a) {
            ODSvalue = ods.getRawLightDetected();

            if (ODSvalue > .35) {
                FINDLINE = true;
                if (power > 0) {
                    int position = 2*90 ;
                    robot.leftMotorF.setMode(STOP_AND_RESET_ENCODER);
                    robot.leftMotorB.setMode(STOP_AND_RESET_ENCODER);
                    robot.rightMotorF.setMode(STOP_AND_RESET_ENCODER);
                    robot.rightMotorB.setMode(STOP_AND_RESET_ENCODER);

                    DrivePowerAll(1);

                    robot.leftMotorF.setTargetPosition(position);
                    robot.leftMotorB.setTargetPosition(position);
                    robot.rightMotorF.setTargetPosition(position);
                    robot.rightMotorB.setTargetPosition(position);

                    robot.leftMotorF.setMode(RUN_TO_POSITION);
                    robot.leftMotorB.setMode(RUN_TO_POSITION);
                    robot.rightMotorF.setMode(RUN_TO_POSITION);
                    robot.rightMotorB.setMode(RUN_TO_POSITION);
                }
            }
        }
        if (FINDLINE){
            if (power < 0) {
                DrivePowerAll(0);
                robot.beaconpusher.setPosition(.5);
            }
            else if (!robot.leftMotorF.isBusy() || !robot.leftMotorB.isBusy() || !robot.rightMotorF.isBusy() || !robot.rightMotorB.isBusy()) {
                robot.beaconpusher.setPosition(.5);
                DrivePowerAll(0);

                robot.leftMotorF.setMode(RUN_USING_ENCODER);
                robot.leftMotorB.setMode(RUN_USING_ENCODER);
                robot.rightMotorF.setMode(RUN_USING_ENCODER);
                robot.rightMotorB.setMode(RUN_USING_ENCODER);
            }

        }
        else {
            robot.leftMotorF.setMode(RUN_USING_ENCODER);
            robot.leftMotorB.setMode(RUN_USING_ENCODER);
            robot.rightMotorF.setMode(RUN_USING_ENCODER);
            robot.rightMotorB.setMode(RUN_USING_ENCODER);

            FINDLINE = false;
            counter=0;
            if (power > 0) {
                robot.leftMotorF.setPower(-power * .9);
                robot.rightMotorF.setPower(-power);
                robot.rightMotorB.setPower(-power * .9);
                robot.leftMotorB.setPower(-power);
            } else {
                robot.leftMotorF.setPower(-power);
                robot.rightMotorF.setPower(-power * .9);
                robot.rightMotorB.setPower(-power);
                robot.leftMotorB.setPower(-power * .9);

            }
        }
    }
    void DriveSideways (double power)
    {
        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (power > 0) // Drive right
        {
            robot.leftMotorF.setPower(-power);
            robot.leftMotorB.setPower(power);
            robot.rightMotorB.setPower(-power);
            robot.rightMotorF.setPower(power);
        }
        else // Drive left
        {
            robot.rightMotorF.setPower(power);
            robot.rightMotorB.setPower(-power);
            robot.leftMotorB.setPower(power);
            robot.leftMotorF.setPower(-power);
        }
    }
}
