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
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *  Autonomous program for Red alliance
 */

@Autonomous(name="Auto Red", group="Ourbot")
//@Disabled
public class Auto_Red extends LinearOpMode {

    /* Declare OpMode members. */
    OurHardware robot   = new OurHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Clamp the Cool Claw's Clampers
        robot.claw.setPosition(.3);
        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        robot.armMotor.setPower(-.05);
        sleep(1000);
        robot.armMotor.setPower(0);

        //Nutn but net
        robot.launchingMotor.setPower(-.5);
        sleep(1470);
        robot.launchingMotor.setPower(0);

        robot.armMotor.setPower(.05);
        sleep(1100);
        robot.armMotor.setPower(0);

        sleep(500);
        robot.claw.setPosition(0);
        sleep(500);

        robot.armMotor.setPower(-.05);
        sleep(1000);
        robot.claw.setPosition(0.15);
        sleep(100);

        robot.armMotor.setPower(.1);
        sleep(700);
        robot.armMotor.setPower(-.1);
        sleep(700);
        robot.armMotor.setPower(0);
        sleep(500);

        //Nutn but net
        robot.launchingMotor.setPower(-.5);
        sleep(1470);
        robot.launchingMotor.setPower(0);

        sleep(1000);

        robot.leftMotorF.setPower(-.5);
        robot.rightMotorF.setPower(-.5);
        robot.leftMotorB.setPower(-.5);
        robot.rightMotorB.setPower(-.5);

        sleep(250);

        robot.leftMotorF.setPower(0);
        robot.rightMotorF.setPower(0);
        robot.leftMotorB.setPower(0);
        robot.rightMotorB.setPower(0);

        sleep(500);

        robot.leftMotorF.setPower(.5);
        robot.rightMotorF.setPower(-.5);
        robot.leftMotorB.setPower(.5);
        robot.rightMotorB.setPower(-.5);

        sleep(450);

        robot.leftMotorF.setPower(0);
        robot.rightMotorF.setPower(0);
        robot.leftMotorB.setPower(0);
        robot.rightMotorB.setPower(0);

        sleep(500);

        robot.leftMotorF.setPower(-.5);
        robot.rightMotorF.setPower(-.5);
        robot.leftMotorB.setPower(-.5);
        robot.rightMotorB.setPower(-.5);

        sleep(1000);

    }
}
