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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="OurTeleOp: Teleop", group="Pushbot")
//@Disabled
public class OurTeleOp extends OpMode {

    /* Declare OpMode members. */
    HardwareTestDrive robot       = new HardwareTestDrive(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    double clawposition = 0.5;
    int armposition = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.armMotor.setTargetPosition(0);
        robot.armMotor.setPower(0.0);
//        robot.armMotor.setMaxSpeed(1000);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        //robot.launchingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double LF_y;
        double LB_y;
        double RB_y;
        double RF_y;

        double LF_x;
        double LB_x;
        double RF_x;
        double RB_x;

        double LFront;
        double LBack;
        double RFront;
        double RBack;

        double RotationLF;
        double RotationLB;
        double RotationRF;
        double RotationRB;

        double armPower;
        double launchPower;

        boolean RightBumper;

//START OF TANK DRIVE
        LF_y = (gamepad1.left_stick_y / 2.0);
        LB_y = (gamepad1.left_stick_y / 2.0);
        RF_y = (gamepad1.right_stick_y / 2.0);
        RB_y = (gamepad1.right_stick_y / 2.0);

        robot.leftMotorF.setPower(LF_y);
        robot.leftMotorB.setPower(LB_y);
        robot.rightMotorF.setPower(RF_y);
        robot.rightMotorB.setPower(RB_y);
//END OF TANK DRIVE

/* START OF MECANUM DRIVE
        //(note: The joystick goes negative when pushed forwards, so negate it)
        LF_y = (gamepad1.left_stick_y / 2.0);
        LB_y = (gamepad1.left_stick_y / 2.0);
        RF_y = (gamepad1.left_stick_y / 2.0);
        RB_y = (gamepad1.left_stick_y / 2.0);

        LF_x = -gamepad1.left_stick_x;
        LB_x =  gamepad1.left_stick_x;
        RF_x =  gamepad1.left_stick_x;
        RB_x = -gamepad1.left_stick_x;

        //RotationLF = -gamepad1.right_stick_x;
        RotationLB = -gamepad1.right_stick_x;
        RotationRF =  gamepad1.right_stick_x;
        RotationRB =  gamepad1.right_stick_x;

        LFront = (LF_y + LF_x);
        LBack  = (LB_y + LB_x);
        RFront = (RF_y + RF_x);
        RBack  = (RB_y + RB_x);

        if (LFront < -1) {

            LFront = LFront - (LFront + 1);
            LBack = LBack - (LFront + 1);
            RFront = RFront - (LFront + 1);
            RBack = RBack - (LFront + 1);

        }



        if (RBack > 1) {

            LFront = LFront - (RBack - 1);
            LBack = LBack - (RBack - 1);
            RFront = RFront - (RBack - 1);
            RBack = RBack - (RBack - 1);

        }

        if (gamepad1.right_stick_x > -0.1 && gamepad1.right_stick_x < 0.1 ) {
        //    robot.leftMotorF.setPower(LFront);
            robot.leftMotorB.setPower(LBack);
            robot.rightMotorF.setPower(RFront);
            robot.rightMotorB.setPower(RBack);
        }
        else {
         //   robot.leftMotorF.setPower(RotationLF);
            robot.leftMotorB.setPower(RotationLB);
            robot.rightMotorF.setPower(RotationRF);
            robot.rightMotorB.setPower(RotationRB);
        }
END OF MECANUM DRIVE */




        //Launching Arm Code And Claw Arm Code
        launchPower = (gamepad2.left_stick_y);
        armPower = (gamepad2.right_stick_y);
        if (armPower > 0.1)
            armPower = 0.05;
        else if(armPower < -0.1)
            armPower = -0.05;
        else
            armPower = 0.0;
//        if (armPower > 0.1)
//        {
//            armposition = armposition + 1;
//        }
//
//        if (armPower < -0.1)
//        {
//            armposition = armposition - 1;
//        }
//        armposition = Range.clip(armposition, -10, 400);
//        robot.armMotor.setTargetPosition(armposition);

        robot.launchingMotor.setPower(launchPower);
        robot.armMotor.setPower(armPower);


//       if (gamepad2.right_bumper) {
//            robot.launchingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.launchingMotor.setTargetPosition(1280);
//            robot.launchingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.launchingMotor.setPower(-1.0);
//            while (robot.launchingMotor.isBusy());
//            robot.launchingMotor.setPower(0);
//        }

        if (gamepad2.b)
        {
            clawposition = clawposition + 0.005;

        }

        if (gamepad2.a)
        {
            clawposition = clawposition - 0.005;

        }
        clawposition = Range.clip(clawposition, 0.0, 0.3);

    robot.claw.setPosition(clawposition);











        // Use gamepad left & right Bumpers to open and close the claw
//        if (gamepad1.right_bumper)
//            clawOffset += CLAW_SPEED;
//        else if (gamepad1.left_bumper)
//            clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
//        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//        robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
//        robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
//        if (gamepad1.y)
//            robot.armMotor.setPower(robot.ARM_UP_POWER);
//        else if (gamepad1.a)
//            robot.armMotor.setPower(robot.ARM_DOWN_POWER);
//        else
//            robot.armMotor.setPower(0.0);

        // Send telemetry message to signify robot running;
        telemetry.addData("claw position",  "Offset = %.2f", robot.claw.getPosition());
        telemetry.addData("arm target",  "%d", armposition);
        telemetry.addData("arm encoder", "%d %d", robot.armMotor.getCurrentPosition(), robot.armMotor.getMaxSpeed());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}