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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

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

@Autonomous(name="Auto Shoot Only", group="Pushbot")
//@Disabled
public class Auto_Shoot_Only extends LinearOpMode {

    /* Declare OpMode members. */
    OurHardware         robot   = new OurHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // claw init
        robot.claw.setPosition(.3);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        robot.init(hardwareMap);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setTargetPosition(0);
        robot.armMotor.setPower(0.0);

        robot.init(hardwareMap);
        robot.launchingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launchingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.launchingMotor.setTargetPosition(0);
        robot.launchingMotor.setPower(0.0);
//        robot.armMotor.setMaxSpeed(1000);
//        robot.armMotor.setMaxSpeed(1000);
// TEST --------------------------------------------------------------------------------------------

//        RobotDriveTime(0.5,2500);
//        sleep(1000);
//        RobotDriveTime(-0.5,2500);
        RobotDrivePosition(5000, .25);


//--------------------------------------------------------------------------------------------------
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        //Nutn but net

        robot.armMotor.setPower(.1);
        sleep(1000);
        robot.armMotor.setPower(0);

        robot.launchingMotor.setTargetPosition(3350);
        robot.launchingMotor.setPower(.5);
        while (robot.launchingMotor.isBusy()) {
            telemetry.addData("encoder", "%d", robot.launchingMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.launchingMotor.setPower(0.0);

        robot.armMotor.setPower(-.1);
        sleep(1100);
        robot.armMotor.setPower(0);

        sleep(500);
        robot.claw.setPosition(0);
        sleep(500);

        robot.armMotor.setPower(.1);
        sleep(1000);
        robot.claw.setPosition(0.15);
        sleep(100);

        robot.armMotor.setPower(-.2);
        sleep(700);
        robot.armMotor.setPower(.2);
        sleep(700);
        robot.armMotor.setPower(0);
        sleep(500);

        robot.launchingMotor.setTargetPosition(3350*2);
        robot.launchingMotor.setPower(.5);
        while (robot.launchingMotor.isBusy()) {
            telemetry.addData("encoder", "%d", robot.launchingMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.launchingMotor.setPower(0.0);
        sleep(1000);




 /*       // Drive forward for 3 seconds
        robot.leftMotorF.setPower(0.2);
        robot.leftMotorB.setPower(0.2);
        robot.rightMotorF.setPower(0.2);
        robot.rightMotorB.setPower(0.2);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.leftMotorF.setPower(0.0);
        robot.leftMotorB.setPower(0.0);
        robot.rightMotorF.setPower(0.0);
        robot.rightMotorB.setPower(0.0);

        sleep(1000);
   */ }

    // FUNCTIONS -----------------------------------------------------------------------------------

    void RobotDriveTime(int time, double power)
    {
        robot.leftMotorF.setPower(-power);
        robot.rightMotorF.setPower(-power);
        robot.rightMotorB.setPower(-power);
        robot.leftMotorB.setPower(-power);

        sleep(time);

        robot.leftMotorF.setPower(0);
        robot.rightMotorF.setPower(0);
        robot.rightMotorB.setPower(0);
        robot.leftMotorB.setPower(0);

    }

    void RobotDrivePosition(int position, double power)
    {
        position = -position;

        robot.leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotorF.setPower(-power);
        robot.rightMotorF.setPower(-power);
        robot.rightMotorB.setPower(-power);
        robot.leftMotorB.setPower(-power);

        robot.leftMotorF.setTargetPosition(position);
        robot.rightMotorF.setTargetPosition(position);
        robot.rightMotorB.setTargetPosition(position);
        robot.leftMotorB.setTargetPosition(position);

        while (robot.leftMotorF.isBusy() || robot.rightMotorF.isBusy() || robot.rightMotorB.isBusy() || robot.leftMotorB.isBusy()) {
            telemetry.addData("encoder", "%d", robot.leftMotorF.getCurrentPosition());
            telemetry.update();
        }

        robot.leftMotorF.setPower(0);
        robot.rightMotorF.setPower(0);
        robot.rightMotorB.setPower(0);
        robot.leftMotorB.setPower(0);

    }

}
